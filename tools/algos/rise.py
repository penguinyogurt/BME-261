"""Detect the CHARGE EVENT, not the amplitude.

chB is an analog peak detector: it charges fast while EMG is present and
otherwise only bleeds down through its RC. A sharp RISE in chB is therefore a
muscle event essentially by construction, whatever level it reaches — amplitude
says how HARD the contraction was, not whether one happened. So the small
sawtooth bumps an amplitude threshold writes off as noise are weak contractions,
and a graded hand needs them.

  onset    a causal slope estimate (difference of a 20 ms and a 100 ms EMA)
           crosses a bar set by the noise of that same slope signal, learned
           ONLY while quiet so a contraction cannot inflate its own floor.
  hold     the latch persists until the envelope has DECAYED most of the way
           back to the floor it started from. This is the crux: a peak detector
           stays elevated for a second or more after the muscle stops, so
           releasing on "no rise" would drop every grip instantly, and a plain
           timeout either over-holds or cuts long grips short.
  force    from the level actually reached, so t_full still means max effort.

Being rise-driven makes it immune to the resting floor drifting (171239 steps
~2050 -> ~2500 mid-trial): only the slope matters, and the floor is re-read at
each onset. Two things have no rise to catch and are handled explicitly:
  * chB already railed  — a peak detector cannot reach 4095 from any resting
    level, so >= SAT is taken as a contraction on its own. That is the only way
    to hold a trial that starts saturated (185816) or rails for 45 s (185723).
  * brown-out recovery  — chB reads 0 while the front end is unpowered and the
    recharge edge (0 -> ~3200) is the largest rise in the whole file. The EMAs
    are dropped on the way down and the detector stays muted through the
    overshoot, which is what latched a 50 s grip on 200828.

STRICTLY CAUSAL: one sample in, one decision out, no lookahead anywhere.
"""
NAME = "rise"
DOC = "onset on a noise-normalised chB rise, release once the envelope has decayed back"

# EMA rates are for the firmware's fixed 500 Hz sampling.
A_FAST, A_SLOW = 0.10, 0.02     # ~20 ms / ~100 ms; their difference is the slope
A_SIG, SIG_MIN = 0.0002, 4.0    # slope-noise tracker: ~30 s, floored off zero
K, D_MIN = 8.0, 60.0            # onset bar = max(D_MIN, K * noise), in counts
A_LO_DN, A_LO_UP = 0.05, 0.002  # resting floor: follow down fast, up slowly
LO_MIN, LO_MAX = 800.0, 2700.0  # hardware prior for a plausible resting envelope
REL = 0.35                      # release once decayed to base + REL*(peak-base)
MIN_HOLD_MS = 250.0
SAT = 4000.0                    # railed: unambiguously a maximal contraction
MUTE_MS = 2500.0                # peak-detector recharge overshoot after a brown-out
FORCE_SPAN = 1800.0             # counts above base that count as full effort
DEAD = 300                      # front end unpowered (the shared FSM also forces OPEN)
T_ON, T_OFF, T_FULL = 1.0, 0.5, 2.0


class Detector:
    def __init__(self, fs=500):
        self.mute_n = int(MUTE_MS / 1000 * fs)
        self.hold_n = int(MIN_HOLD_MS / 1000 * fs)
        self.f = None           # None = unseeded: stream start, or just browned out
        self.sig = SIG_MIN
        self.nq = 0             # quiet samples seen, for the noise tracker's warm-up
        self.mute = 0
        self.on = False
        self.held = 0

    def update(self, chB, chA=0):
        if chB < DEAD:
            self.f = None       # forget the EMAs so the recovery edge is not a rise
            self.on = False
            self.mute = self.mute_n
            return 0.0, T_ON, T_OFF, T_FULL
        if self.f is None:
            self.f = self.s = float(chB)
            self.lo = min(max(float(chB), LO_MIN), LO_MAX)
        self.f += A_FAST * (chB - self.f)
        self.s += A_SLOW * (chB - self.s)
        d = self.f - self.s
        if self.mute > 0:       # riding out the recharge transient: decide nothing
            self.mute -= 1
            return 0.0, T_ON, T_OFF, T_FULL

        if self.on:
            self.held += 1
            self.peak = max(self.peak, self.s)
            if (self.held > self.hold_n and
                    self.s < self.base + REL * (self.peak - self.base)):
                self.on = False
        else:
            # Floor and noise are learned only here, i.e. only while quiet.
            self.lo += (A_LO_DN if chB < self.lo else A_LO_UP) * (chB - self.lo)
            self.lo = min(max(self.lo, LO_MIN), LO_MAX)
            self.nq += 1
            # 1/n for the first samples so a fresh stream converges immediately,
            # then a long time constant so the bar stays put. The 3x clamp keeps
            # the leading edge of a contraction from raising its own bar.
            self.sig += max(A_SIG, 1.0 / self.nq) * (min(abs(d), 3 * self.sig) - self.sig)
            self.sig = max(self.sig, SIG_MIN)
            if d > max(D_MIN, K * self.sig) or chB >= SAT:
                self.on = True
                self.base = self.lo         # the floor this contraction left from
                self.peak = max(self.s, float(chB))
                self.held = 0

        if not self.on:
            return 0.0, T_ON, T_OFF, T_FULL
        # Held: sit just above t_on (so the FSM keeps CLOSING rather than
        # falling back to HOLDING) and scale up to t_full with effort.
        force = min(1.0, max(0.0, (self.s - self.base) / FORCE_SPAN))
        return T_ON + 0.02 + force * (T_FULL - T_ON - 0.02), T_ON, T_OFF, T_FULL


if __name__ == "__main__":
    import random

    FS = 500
    rng = random.Random(7)

    def run(sig):
        det = Detector(FS)
        return [det.update(x, 0)[0] for x in sig]

    def rest(n, level=2000, jit=40):
        return [level + rng.randint(-jit, jit) for _ in range(n)]

    def fired(a):
        return any(v > T_ON for v in a)

    # 20 s of resting envelope must never trigger
    assert not fired(run(rest(20 * FS))), "rest triggered"

    # a slow baseline drift, 2000 -> 2600 over 60 s, is not a contraction
    drift = [2000 + 600 * i / (60 * FS) + rng.randint(-40, 40) for i in range(60 * FS)]
    assert not fired(run(drift)), "baseline drift triggered"

    # ramp into a 3 s hold: must trigger on the ramp and stay held throughout
    ramp = ([2000 + 1400 * i / 75 for i in range(75)] +      # 150 ms charge
            rest(3 * FS, 3400))
    a = run(rest(5 * FS) + ramp)
    hold = a[5 * FS + 75:]
    assert fired(a), "ramp-and-hold did not trigger"
    assert all(v > T_ON for v in hold), "grip dropped during the hold"
    assert a[5 * FS + 75 + 60] > a[5 * FS + 20], "force must scale with level"

    # a brown-out and its recharge overshoot (~3200, decaying over ~2 s) is not a grip
    boot = [300 + 2900 * (0.9985 ** i) - 1200 * (1 - 0.9985 ** i) for i in range(4 * FS)]
    assert not fired(run(rest(2 * FS) + [0] * (3 * FS) + boot + rest(5 * FS))), \
        "brown-out recovery triggered"

    # a stream that starts already railed has no onset to see: hold it anyway
    a = run([4095] + [4095 - rng.randint(0, 150) for _ in range(10 * FS - 1)])
    assert a[0] > T_ON and all(v > T_ON for v in a), "saturated stream not held"

    print("ok")
