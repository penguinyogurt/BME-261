"""Teager-Kaiser energy operator onset detection, adapted to an ALREADY-ENVELOPED signal.

TKEO FORM AND LAG
The textbook operator psi(x[n]) = x[n]^2 - x[n-1]*x[n+1] needs x[n+1], so this
uses the ONE-SAMPLE-DELAY form: when sample x[n] arrives we emit psi for index
n-1 using x[n-2], x[n-1], x[n]. Output lags the stream by exactly 1 sample
(2 ms at 500 Hz). No other lookahead anywhere.

WHY THE OUTPUT IS *NOT* RECTIFIED BEFORE SMOOTHING
Expanding psi for a smooth signal gives psi ~= x'^2 - x*x''. chB is an analog
peak-detector envelope, not raw EMG, so the x'^2 term is tiny: the fastest
onsets here ramp ~40 counts/sample, i.e. x'^2 ~ 1600 counts^2. Meanwhile ADC
jitter e[n] on a DC level c contributes c*(2e[n] - e[n-1] - e[n+1]), which with
c ~ 2000 and sigma_e ~ 15 counts is ~70000 counts^2 -- 40x larger. Rectifying
first (the textbook recipe) locks that jitter floor in as a positive bias, and
measurement bears this out: |psi| smoothed separates a key onset from rest by
only ~4.6 sigma, and most of even that is the DC-level term (q ~ sqrt(c), and c
rises from 2000 to 4095 during a grip), not muscle information.

The jitter term is ZERO-MEAN, so smoothing the SIGNED psi cancels it while the
-x*x'' curvature term survives. That lifts onset separability to ~17 sigma. So:
smooth signed, then rectify. Rectifying first is what kills TKEO on this signal.

WHAT TKEO IS AND IS NOT USED FOR HERE
-psi is a curvature detector: it spikes at the CORNER where a rise begins and
is ~0 on the flat railed plateau of a saturated contraction (measured: median
psi is exactly 0 on 185723/185816). It therefore cannot sustain a grip. It is
used only to ARM a lower onset threshold; the activity the FSM latches on is
the plain envelope excess, which holds fine through saturation.

DOES ANY OF IT ACTUALLY HELP? NO -- MEASURED, NOT ASSUMED.
At the constants below (chosen on the FIT trials only) this detector scores
bit-identically to the same code with the arm disabled, i.e. a plain 340-count
envelope threshold:

    TKEO armed                          TEST  lat=116 FA=1.89 F1=86.5
    arm disabled, ON_LO = ON_HI = 340   TEST  lat=116 FA=1.89 F1=86.5
    arm forced permanently on           TEST  lat=116 FA=1.89 F1=86.5

ON_HI is likewise inert: 460, 520 and 600 all give the same numbers, because
any rise fast enough to reach ON_LO has already armed. Making the arm selective
enough to matter (K_TKEO ~14-20) does cut FA, but only by sliding along the
same threshold-vs-FA trade-off a plain threshold already traces -- it does not
move that curve by more than ~2 F1, and never on the FIT set.

So the TKEO stage is kept because it is what this experiment was built to test,
and it costs ~6 flops/sample, but it is NOT carrying the detection. On a signal
the analog front end has already rectified and smoothed, TKEO has nothing left
to key on. Do not port it to firmware expecting it to earn its place.
"""
NAME = "tkeo"
DOC = "signed TKEO curvature arms a low onset threshold; envelope excess sustains (arm measures as inert)"

DEAD = 300              # matches the shared FSM's safety floor
LOCKOUT_MS = 2400       # brown-out recovery: peak detector overshoots ~2 s
WARMUP_MS = 300         # arming is disabled until the rest stats have settled

TAU_PSI_MS = 8          # smoothing of the SIGNED operator output
TAU_ACT_MS = 15         # de-jitter of the envelope excess
TAU_STAT_MS = 1500      # rest mean/spread of the smoothed operator
A_BASE_DOWN = 0.02      # floor follows the signal down fast...
A_BASE_UP = 0.002       # ...up at ~1 s while we believe we are at rest...
A_BASE_CREEP = 0.0002   # ...and up at ~10 s even when we do not. Without this
                        # last leak a genuine STEP in the resting floor (171239
                        # jumps ~450 counts after its first hard contraction)
                        # latches "active" forever. 10 s is long against a 1-3 s
                        # grip and short against a floor that never comes back;
                        # BASE_HI stops it eating the 25-45 s railed trials.
BASE_LO, BASE_HI = 1400.0, 2600.0   # hardware prior for a plausible resting envelope

K_TKEO = 5.0            # arm when -psi exceeds rest mean + k*sigma
ARM_MS = 600            # an arm stays live this long after the curvature spike
REST_ACT = 240.0        # excess below this (and unarmed) = believed at rest

ON_HI = 520.0           # onset threshold with no TKEO evidence
ON_LO = 340.0           # onset threshold while armed -- this is TKEO's whole job
OFF = 155.0             # release; low so a grip holds through its plateau
FULL = 1500.0           # excess at which grip force saturates


def _alpha(tau_ms, fs):
    return 1.0 - 2.718281828459045 ** (-1.0 / (tau_ms / 1000.0 * fs))


class Detector:
    def __init__(self, fs=500):
        self.fs = fs
        self.a_psi = _alpha(TAU_PSI_MS, fs)
        self.a_act = _alpha(TAU_ACT_MS, fs)
        self.a_stat = _alpha(TAU_STAT_MS, fs)
        self.a_slope = _alpha(30.0, fs)
        self.lockout_n = int(LOCKOUT_MS / 1000.0 * fs)
        self.warmup_n = int(WARMUP_MS / 1000.0 * fs)
        self.arm_n = int(ARM_MS / 1000.0 * fs)

        self.x1 = self.x2 = None    # x[n-1], x[n-2] for the delayed operator
        self.g = 0.0                # smoothed SIGNED psi
        self.mu = 0.0               # its resting mean (~0 once jitter cancels)
        self.dev = 4000.0           # its resting mean-absolute-deviation. ~2x the
                                    # measured resting MAD of g (1900-2200), so the
                                    # arm starts conservative and relaxes as it learns
        self.slope = 0.0
        self.base = None
        self.act = 0.0
        self.arm = 0
        self.lock = 0
        self.seen = 0

    def update(self, chB, chA):
        # Brown-out, and the ~2 s recharge transient after it. Everything is
        # frozen and re-seated rather than merely ignored: the collapse ramps
        # chB down through the whole plausible rest band, and a baseline that
        # tracks that down pins itself at BASE_LO for the rest of the trial.
        if chB < DEAD or self.lock > 0:
            self.lock = self.lockout_n if chB < DEAD else self.lock - 1
            self.arm = self.seen = 0
            self.x1 = self.x2 = None    # the recovery edge is not an onset
            self.base = None            # re-seat from wherever it settles
            self.act = self.g = 0.0
            return 0.0, ON_HI, OFF, FULL

        self.seen += 1
        if self.base is None:
            self.base = min(BASE_HI, max(BASE_LO, float(chB)))

        # --- TKEO, one-sample-delay form: psi for x[n-1], available now ---
        if self.x2 is not None:
            psi = self.x1 * self.x1 - self.x2 * chB
            self.g += self.a_psi * (psi - self.g)
        self.x2, self.x1 = self.x1, float(chB)
        self.slope += self.a_slope * ((chB - (self.x2 or chB)) - self.slope)

        # --- envelope excess: the part that can actually sustain a grip ---
        if chB < self.base:
            self.base += A_BASE_DOWN * (chB - self.base)
        self.base = min(BASE_HI, max(BASE_LO, self.base))
        e = max(0.0, chB - self.base)
        self.act += self.a_act * (e - self.act)

        # --- arm on upward curvature that is large vs the RESTING spread ---
        sigma = max(1.25 * self.dev, 1.0)
        warm = self.seen > self.warmup_n
        if warm and (self.mu - self.g) > K_TKEO * sigma and self.slope > 0.0:
            self.arm = self.arm_n
        elif self.arm > 0:
            self.arm -= 1

        # --- learn rest stats ONLY while we believe we are at rest ---
        resting = self.arm == 0 and self.act < REST_ACT
        if resting:
            self.mu += self.a_stat * (self.g - self.mu)
            self.dev += self.a_stat * (abs(self.g - self.mu) - self.dev)
        if chB > self.base:
            self.base += (A_BASE_UP if resting else A_BASE_CREEP) * (chB - self.base)

        return self.act, (ON_LO if self.arm > 0 else ON_HI), OFF, FULL


if __name__ == "__main__":
    import math
    import os
    import random
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import replay_intent as R

    def run(sig):
        d = Detector(500)
        cols = [d.update(x, 0) for x in sig]
        assert all(all(math.isfinite(v) for v in c) for c in cols), "nan/inf leaked out"
        act = [c[0] for c in cols]
        _, st = R.run_fsm(act, [c[1] for c in cols], [c[2] for c in cols],
                          [c[3] for c in cols], raw=sig)
        return st

    random.seed(7)
    jit = lambda c: c + random.gauss(0, 15)

    rest = [jit(2000) for _ in range(5 * 500)]
    assert set(run(rest)) == {R.ST_OPEN}, "quiet rest must never trigger"

    # 2 s rest then a 3 s grip with a realistic ~120 ms peak-detector rise
    rise = [jit(2000 + 1600 * min(1.0, i / 60.0)) for i in range(3 * 500)]
    st = run(rest[:1000] + rise)
    assert R.ST_CLOSING in st, "a clear burst must close"
    on = next(i for i, s in enumerate(st) if s == R.ST_CLOSING) * R.TICK_DIV
    assert on - 1000 <= 100, f"onset must land within 200 ms, got {(on-1000)*2} ms"

    # saturated plateau: TKEO is exactly 0 there, so the envelope must hold it
    st = run(rest[:1000] + [4095] * (10 * 500))
    tail = st[len(st) // 2:]
    assert R.ST_OPEN not in tail, "a railed plateau must stay held, not released"

    # brown-out: 3 s dead, then the peak detector overshoots and decays ~2 s
    recov = [jit(1950 + 1250 * math.exp(-i / 500.0)) for i in range(6 * 500)]
    st = run(rest[:1000] + [0] * (3 * 500) + recov)
    edge = len(rest[:1000] + [0] * (3 * 500)) // R.TICK_DIV
    assert R.ST_CLOSING not in st[edge:edge + 250], "recovery transient is not an onset"

    print("ok")
