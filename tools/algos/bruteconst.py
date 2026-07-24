"""The dumbest possible causal detector: three constants, chosen by brute force.

No baseline tracking, no tare, no calibration, no percentiles, no adaptation of
any kind. `activity` is chB itself, lightly de-jittered by one EMA, and the
three thresholds are fixed numbers in raw ADC counts that are identical for
every trial and every wearer.

This is the CONTROL EXPERIMENT. If an adaptive method cannot beat a constant,
the adaptation is not earning its complexity. The constants below were found by
exhaustive grid search over the FIT trials only (see __main__); the TEST trials
were never consulted while tuning.

WHAT THE SEARCH ACTUALLY OPTIMISED
The shared FSM's *state* depends only on t_on and t_off. t_full feeds nothing
but the intent magnitude inside CLOSING, and the scorer only reads states — so
t_full is UNIDENTIFIABLE from the scorer and sweeping it is provably flat. It
is set instead from the data: the 90th percentile of activity during FIT ACTIVE
ticks, i.e. where grip force should be maxed out on a real contraction.

WHERE IT BREAKS, AND WHY THAT IS THE POINT
T_ON = 2080 works only because all five FIT trials happen to rest between 1844
and 1952 counts. Two TEST trials do not: 171239 rests at 2550 (its floor steps
up ~450 counts two seconds in and never returns) and 202105 rests at 2216. On
both, the constant sits BELOW the resting floor and the hand latches shut for
the whole recording. F1 barely notices (77 on each, because a permanently
closed hand trivially covers every burst); quiet% collapses to 12% and 27%.
Read quiet%, not F1, when judging this detector.
"""
NAME = "bruteconst"
DOC = "brute-forced absolute constants on chB — no baseline, no adaptation"

# --- winning constants: grid search on FIT trials only, mean per-trial F1 ---
# 9380 candidates, FIT F1 = 89.7. The optimum is interior (not at a grid edge)
# but the top of the surface is a plateau: every one of the top 15 is within
# 0.6 F1 of the winner, spanning alpha 0.10-1.0 and t_on 2040-2120. So the
# de-jitter EMA and
# the hysteresis gap both earn ~nothing here — brute force chose alpha=1.0 (no
# filter at all) and t_off == t_on (no hysteresis). That is the honest answer,
# not a shortcut: the FSM's own 80 ms debounce / 150 ms relax already does the
# de-jittering, so a second filter is redundant.
ALPHA = 1.0             # de-jitter EMA (1.0 = no filtering at all)
T_ON = 2080.0           # close above this many raw counts
T_OFF = 2080.0          # release below this many raw counts
T_FULL = 4095.0         # full grip force here (not scored — see above)

DEAD = 300              # matches the FSM's brown-out floor


class Detector:
    """Constant thresholds. The kwargs exist only so __main__ can sweep them."""

    def __init__(self, fs=500, alpha=ALPHA, t_on=T_ON, t_off=T_OFF, t_full=T_FULL):
        self.a, self.on, self.off, self.full = alpha, t_on, t_off, t_full
        self.act = None

    def update(self, chB, chA):
        if chB >= DEAD:                 # brown-out zeros must not drag the EMA
            self.act = (float(chB) if self.act is None
                        else self.act + self.a * (chB - self.act))
        return (0.0 if self.act is None else self.act), self.on, self.off, self.full


# ------------------------------------------------------------------ the search
if __name__ == "__main__":
    import glob
    import os
    import sys
    import time

    import numpy as np

    TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ROOT = os.path.dirname(TOOLS)
    sys.path.insert(0, TOOLS)
    import replay_intent as R          # noqa: E402
    import score_intent as S           # noqa: E402

    ALPHAS = [1.0, 0.30, 0.10, 0.04, 0.01]
    ONS = range(1900, 3001, 20)
    OFFS = range(1800, 3001, 20)

    def load_trial(stem):
        csvs = glob.glob(os.path.join(ROOT, "data", f"*{stem}.csv"))
        keys = glob.glob(os.path.join(ROOT, "data", "answer_key", f"*{stem}_key.csv"))
        _, _, B, _ = R.load(csvs[0])
        return np.asarray(B), np.asarray(S.load_key(keys[0]))

    def activity(B, alpha):
        d = Detector(R.SAMPLE_HZ, alpha=alpha)
        return np.fromiter((d.update(b, 0)[0] for b in B), float, len(B))

    def runlen(m):
        """Length of the consecutive-True run ending at each index."""
        c = np.cumsum(m)
        return c - np.maximum.accumulate(np.where(m, 0, c))

    def closing_mask(actt, dead, on, off):
        """R.run_fsm's CLOSING ticks, without recomputing intent.

        ah/bo are pure run-length counters at 20 ms per tick, so the DEBOUNCE /
        RELAX / OPEN_DWELL tests are just run-length tests. A dead tick zeroes
        both counters, which is the same as breaking the run there.
        """
        live = ~dead
        a4 = (runlen((actt > on) & live) >= 4).tolist()          # DEBOUNCE 80 ms
        rb = runlen((actt < off) & live)
        b8 = (rb >= 8).tolist()                                  # RELAX 150 ms
        b100 = (rb >= 100).tolist()                              # DWELL 2000 ms
        dd = dead.tolist()
        st, open_t, out = 0, 0, []
        for i in range(len(a4)):
            if dd[i]:
                st = 0
                out.append(False)
                continue
            if st == 0:
                if a4[i]: st = 1
            elif st == 1:
                if b8[i]: st = 2
            elif st == 2:
                if a4[i]: st = 1
                elif b100[i]: st = 3; open_t = i
            else:
                if a4[i]: st = 1
                elif i - open_t >= 125: st = 0                   # OPEN_DRIVE 2500 ms
            out.append(st == 1)
        return np.asarray(out)

    def f1_of(closing, key):
        n = min(len(closing), len(key))
        c, k = closing[:n], key[:n]
        tp = int((c & (k == 1)).sum())
        fp = int((c & (k == 0)).sum())
        fn = int((~c & (k == 1)).sum())
        d = 2 * tp + fp + fn
        return 2 * tp / d * 100 if d else float("nan")

    # ---------------------------------------------------- self-check (asserts)
    B, key = load_trial(S.FIT[0])
    act = activity(B, 0.30)
    assert act[0] == B[0], "EMA must start at the first live sample"
    _, states = R.run_fsm(list(act), T_ON, T_OFF, T_FULL, raw=list(B))
    mine = closing_mask(act[::10], B[::10] < DEAD, T_ON, T_OFF)
    ref = np.asarray([s == R.ST_CLOSING for s in states])
    assert len(mine) == len(ref) and (mine == ref).all(), "fast FSM diverged"
    assert abs(f1_of(mine, key) - S.score_one(states, key.tolist())["f1"]) < 1e-9
    # t_full provably does not move any scored metric
    a = S.score_one(R.run_fsm(list(act), T_ON, T_OFF, 2600.0, raw=list(B))[1],
                    key.tolist())
    b = S.score_one(R.run_fsm(list(act), T_ON, T_OFF, 4095.0, raw=list(B))[1],
                    key.tolist())
    assert a == b, "t_full should be unidentifiable from the scorer"
    print("self-check ok\n")

    # ------------------------------------------------------ exhaustive search
    trials = []
    for stem in S.FIT:
        B, key = load_trial(stem)
        if not (key == 1).any():
            continue                                    # nothing to detect
        trials.append((stem, B, key))
    print(f"FIT trials: {', '.join(s for s, _, _ in trials)}")

    results = []
    t0 = time.time()
    for alpha in ALPHAS:
        prep = [(activity(B, alpha)[::10], B[::10] < DEAD, key)
                for _, B, key in trials]
        for on in ONS:
            for off in OFFS:
                if off > on:
                    break
                f1s = [f1_of(closing_mask(a, d, on, off), k) for a, d, k in prep]
                f1s = [x for x in f1s if x == x]
                results.append((sum(f1s) / len(f1s), alpha, on, off))
        best = max(results)
        print(f"  alpha={alpha:<5} done  best so far F1={best[0]:.2f} "
              f"@ alpha={best[1]} on={best[2]} off={best[3]}  "
              f"({time.time() - t0:.0f}s)")

    results.sort(reverse=True)
    print(f"\n{len(results)} candidates searched in {time.time() - t0:.0f}s")
    print("\ntop 15 on FIT (mean per-trial F1):")
    print(f"  {'F1':>6} {'alpha':>6} {'t_on':>6} {'t_off':>6}")
    for f1, alpha, on, off in results[:15]:
        print(f"  {f1:6.2f} {alpha:6.2f} {on:6d} {off:6d}")

    f1, alpha, on, off = results[0]
    # t_full: not scored, so take it from the data — where grip should be maxed.
    hot = np.concatenate([activity(B, alpha)[::10][key[:len(B[::10])] == 1]
                          for _, B, key in trials])
    print(f"\nWINNER  ALPHA={alpha}  T_ON={on}  T_OFF={off}  "
          f"T_FULL={np.percentile(hot, 90):.0f}  (FIT F1={f1:.2f})")
