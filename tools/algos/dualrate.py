"""Dual-rate ratio detector: fast envelope / slow envelope.

Two EMAs of chB. FAST (~40 ms) follows the contraction; SLOW (seconds) is a
running estimate of the resting floor. The decision variable is their RATIO,
not their difference, so it is DIMENSIONLESS: thresholds are pure numbers
(1.16, not 300 counts), and when the resting floor drifts — 171239 steps
~2050 -> ~2500 mid-trial — numerator and denominator move together and the
threshold moves with them. No tare, no calibration, no per-wearer constants.

The compressed-dynamic-range worry, and why it is not one
--------------------------------------------------------
Rest here is ~1900 counts, not ~0, so a +300 count contraction is only a
1.16x ratio and the whole useful band is 1.0-2.15. That LOOKS fatal, but the
signal-to-noise ratio is untouched: dividing by `slow` divides the noise by
exactly the same number, so 300 counts of signal over 15 counts of fast-EMA
noise is 20:1 whether it is expressed as counts or as 1.16 vs 1.008. The
compression is a NUMERIC-FORMAT problem, not a detection problem — on
firmware compute it as `(fast << 8) / slow` in Q8 and the resolution is
0.004, four times finer than the resting noise.

Subtracting a DC pedestal first ((fast-P)/(slow-P)) does stretch the numbers
out, but it re-introduces exactly the calibration the ratio existed to avoid:
the sensitivity then depends on how far the floor sits above P, so a floor
drifting 1900 -> 2500 changes the effective threshold by 2.5x. PEDESTAL is
left as a knob and measured in the report; 0 (pure ratio) wins.

Three things that would otherwise break it
------------------------------------------
* SLOW dragged up by a long grip. Frozen whenever the ratio is clearly
  elevated (> FREEZE_R, comfortably above the ON threshold so a drifting
  floor does not latch the freeze), with a timeout so a genuine step in the
  floor cannot lock it out forever.
* Trials pinned at 4095 (185723 is one 45 s hold, so SLOW never sees a clean
  rest to learn from). SLOW is clamped to the front-end's physical idle band,
  so a railed signal always reads >= 4095/2600 = 1.57 no matter how long it
  is held. This is the one hardware prior in the method, and it is worth 7 F1
  points on the fit set — without it a long grip is eventually absorbed.
* Brown-outs. chB reads 0 for long stretches; those samples update nothing
  and the ratio is pinned at 1.0 (and `slow` can never reach 0, so the
  divide is always safe). When power returns the peak detector overshoots to
  ~3200 and decays for ~2 s — not a contraction — so SLOW re-seeds to the
  present value and output is held at 1.0 until it has settled.
"""
NAME = "dualrate"
DOC = "ratio of a fast to a slow envelope EMA — dimensionless, needs no tare"

DEAD_COUNTS = 300       # matches the shared FSM's safety floor
PEDESTAL = 0.0          # subtracted from both before dividing; 0 = pure ratio
REST_LO, REST_HI = 1500.0, 2600.0   # physical idle band of this peak detector
FAST_MS = 40.0          # short: any lag here is latency the scorer charges you for
SLOW_DOWN_S = 0.8       # floor falls fast (release, post-brown-out decay)
SLOW_UP_S = 6.0         # ...and rises slowly, so a grip is not absorbed
FREEZE_R = 1.22         # above this the floor estimate stops learning entirely
FREEZE_S = 6.0          # ...but not forever: a real step in the floor must win
SETTLE_S = 2.5          # blind window after power returns (peak-detector recharge)
WARM_S = 0.5            # floor = running mean this long, so power-on cannot latch
T_ON, T_OFF, T_FULL = 1.16, 1.075, 1.90


class Detector:
    def __init__(self, fs=500):
        self.a_fast = 1.0 / max(1.0, FAST_MS / 1000.0 * fs)
        self.a_dn = 1.0 / max(1.0, SLOW_DOWN_S * fs)
        self.a_up = 1.0 / max(1.0, SLOW_UP_S * fs)
        self.n_freeze = int(FREEZE_S * fs)
        self.n_settle = int(SETTLE_S * fs)
        self.n_warm = int(WARM_S * fs)
        self.fast = self.slow = None
        self.held = 0           # samples spent frozen
        self.settle = 0         # samples left in the post-brown-out blind window
        self.n = 0              # live samples seen, for the warm-up

    def update(self, chB, chA):
        th = (T_ON, T_OFF, T_FULL)
        if chB < DEAD_COUNTS:               # unpowered: learn nothing, report nothing
            self.fast = self.slow = None
            self.held = 0
            self.settle = self.n_settle     # arm the blind window for the recharge
            return 1.0, *th
        x = float(chB)
        if self.slow is None:               # cold start: trust the physical band,
            self.fast = self.slow = _clamp(x)   # so a trial opening railed still fires
            return 1.0, *th

        self.fast += self.a_fast * (x - self.fast)
        if self.settle:                     # peak detector still recharging: this is
            self.settle -= 1                # neither a contraction nor a usable floor,
            self.slow = self.fast           # so pin the floor to the present value and
            return 1.0, *th                 # emerge with the floor already correct
        self.slow = _clamp(self.slow)       # the front end's physical idle band
        r = (self.fast - PEDESTAL) / max(self.slow - PEDESTAL, 1.0)
        # Warm-up: for the first WARM_S the floor is just the running mean, which
        # converges in tens of samples and cannot be latched by the freeze. Seeding
        # it from one sample instead let a single low reading at power-on hold the
        # ratio at 1.29 and close the hand for two seconds (195519).
        self.n += 1
        warm = self.n <= self.n_warm

        # Freeze the floor while clearly contracting — but only for FREEZE_S. Once
        # that runs out the floor tracks again and KEEPS tracking until the ratio
        # falls back, otherwise a genuine step in the resting level (171239) would
        # hold the freeze on forever and latch the hand shut.
        self.held = self.held + 1 if (r > FREEZE_R and not warm) else 0
        if self.held == 0 or self.held > self.n_freeze:
            a = self.a_dn if self.fast < self.slow else self.a_up
            self.slow += max(a, 1.0 / self.n if warm else 0.0) \
                * (self.fast - self.slow)
        return r, *th


def _clamp(v):
    return REST_LO if v < REST_LO else (REST_HI if v > REST_HI else v)


if __name__ == "__main__":
    import random

    FS = 500

    def run(sig):
        d = Detector(FS)
        return [d.update(v, 0)[0] for v in sig]

    def fired(acts, need_ms=80):
        """True if activity held above T_ON long enough for the FSM to close."""
        n = 0
        for a in acts:
            n = n + 1 if a > T_ON else 0
            if n >= need_ms / 1000.0 * FS:
                return True
        return False

    def first_on(acts, need_ms=80):
        n = 0
        for i, a in enumerate(acts):
            n = n + 1 if a > T_ON else 0
            if n >= need_ms / 1000.0 * FS:
                return i
        return None

    random.seed(7)
    noise = lambda: random.gauss(0, 45)

    # 1. quiet rest never triggers
    rest = [1900 + noise() for _ in range(30 * FS)]
    assert not fired(run(rest)), "quiet rest must not trigger"

    # 1b. ...not even when the very first sample is a low outlier. Seeding the
    #     floor from one sample used to latch the hand shut for ~2 s at power-on.
    assert not fired(run([1350] + [1950 + noise() for _ in range(30 * FS)])), \
        "a low first sample must not latch the floor"

    # 2. a burst triggers within ~200 ms of its onset
    sig = [1900 + noise() for _ in range(10 * FS)] + \
          [2400 + noise() for _ in range(2 * FS)]
    i = first_on(run(sig))
    assert i is not None and i - 10 * FS < 0.2 * FS, f"burst latency {i}"

    # 3. a sustained hold does not decay back below the release threshold
    hold = [1900 + noise() for _ in range(10 * FS)] + \
           [3400 + noise() for _ in range(40 * FS)]
    a = run(hold)
    assert min(a[12 * FS:50 * FS]) > T_OFF, "sustained grip decayed to release"

    # 4. a drifting floor (1800 -> 2500 over 20 s) never triggers.
    #    A true instantaneous step is indistinguishable from an onset at the
    #    instant it happens and does fire; FREEZE_S bounds how long for.
    drift = [1800 + noise() for _ in range(10 * FS)] + \
            [1800 + 700 * i / (20 * FS) + noise() for i in range(20 * FS)] + \
            [2500 + noise() for _ in range(10 * FS)]
    assert not fired(run(drift)), "a drifting resting floor must not trigger"

    # 5. zeros: no activity, no divide-by-zero, and no poisoning of the floor
    z = [1900 + noise() for _ in range(5 * FS)] + [0] * (5 * FS) + \
        [3200 * (1 - i / (2.0 * FS)) + 1900 * min(1, i / (2.0 * FS)) + noise()
         for i in range(2 * FS)] + \
        [1900 + noise() for _ in range(5 * FS)] + \
        [2400 + noise() for _ in range(2 * FS)]
    a = run(z)
    assert all(v == v for v in a), "NaN in activity"
    assert not fired(a[:12 * FS]), "brown-out / recharge transient must not fire"
    assert fired(a[12 * FS:]), "must still detect after recovering from a brown-out"

    # 6. a trial that opens already railed must fire immediately
    assert first_on(run([4095] * (5 * FS))) is not None, \
        "trial starting saturated must fire"

    print("ok")
