"""Fixed threshold offsets above an asymmetric tracked floor, every constant
found by exhaustive grid search on the FIT trials only.

    activity = EMA( max(0, chB - floor) )
    floor   += (A_DN if chB < floor else A_UP) * (chB - floor),  capped

Nothing here is reasoned about. A_DN, A_UP, ALPHA, BASE_CAP, T_ON, T_OFF and
T_FULL are whatever won the search under `if __name__ == "__main__"`. Run this
file to reproduce it.
"""
NAME = "brutetared"
DOC = "grid-searched offsets above an asymmetric tracked floor"

# ---------------------------------------------------------------- WINNERS
# Found by the search below on FIT = 134851 182349 185937 193836 200457, over
# ~55k parameter combinations. TEST was never consulted while choosing them.
# The absurd precision is not significance, it is what a grid argmax looks like:
# J was flat to ~0.1 across A_DN 0.04-0.07 and ALPHA 0.29-0.64.
A_DN = 0.065625    # floor falls toward chB at this rate  (per sample)
A_UP = 0.0028      # ...and rises at this one
BASE_CAP = 1895.25 # floor may never exceed this — this is what does most of the
ALPHA = 0.32       # work, see the ablation. activity de-jitter EMA (~6 ms: the
T_ON = 280.0       # FSM's own 80 ms debounce is the real de-jitter)
T_OFF = 187.5      # counts above the floor
T_FULL = 2000.0    # score-invariant (stage 3); sets grip force only

# ponytail: BASE_CAP is a known ceiling, NOT a fix. The search coupled a fast
# A_UP (which would otherwise absorb a sustained grip into the floor) to a cap
# that stops it, and then set the cap to FIT's resting level. Every FIT trial
# rests at 1800-2100, so FIT cannot punish that; on TEST trials whose floor
# drifts ABOVE the cap (171239, 202105) the floor pins, ~600 counts of standing
# activity clear T_ON, and the hand never reopens (quiet% 12 and 31). Fixing it
# means decoupling the two — a floor that may rise anywhere in the hardware's
# resting band with a rise rate slow enough to not eat a grip on its own — and
# re-running the search. Deliberately NOT done here: it was diagnosed from TEST,
# and re-tuning on TEST would destroy the only honest evidence this file has.

DEAD = 300         # chB below this: front end unpowered (shared FSM forces OPEN)


class Detector:
    """Strictly causal: one sample in, one (activity, thresholds) out."""

    def __init__(self, fs=500):
        self.fs = fs
        self.base = None
        self.act = 0.0

    def update(self, chB, chA):
        if chB < DEAD:                  # brown-out: freeze, learn nothing
            return 0.0, T_ON, T_OFF, T_FULL
        if self.base is None:           # warm-up: never start above the band,
            self.base = min(float(chB), BASE_CAP)   # or a railed trial is deaf
        self.base += (A_DN if chB < self.base else A_UP) * (chB - self.base)
        if self.base > BASE_CAP:
            self.base = BASE_CAP
        a = chB - self.base
        self.act += ALPHA * ((a if a > 0.0 else 0.0) - self.act)
        return self.act, T_ON, T_OFF, T_FULL


def _selfcheck():
    d = Detector()
    for _ in range(1500):               # settle on a plausible resting floor
        a, on, off, full = d.update(2000, 0)
    assert off < on < full, "hysteresis and saturation order"
    assert a < off, f"rest must sit below t_off, got {a:.1f}"
    for _ in range(250):                # 0.5 s of hard contraction
        a, _, _, _ = d.update(4095, 0)
    assert a > on, f"a railed contraction must clear t_on, got {a:.1f}"
    b = d.base
    for _ in range(2500):               # 5 s of it: floor must not absorb it
        a, _, _, _ = d.update(4095, 0)
    assert a > on, f"sustained grip must stay above t_on, got {a:.1f}"
    assert d.base <= BASE_CAP + 1e-9, "floor escaped the cap"
    d2 = Detector()
    for _ in range(5000):               # railed from sample 0, no rest to learn
        a, _, _, _ = d2.update(4095, 0)
    assert a > on, "a trial that starts saturated must still fire"
    d3 = Detector()
    for _ in range(500):
        assert d3.update(0, 0)[0] == 0.0, "brown-out must produce no activity"
    assert d3.base is None, "brown-out must not seed the floor"
    print("selfcheck ok")


# ===================================================================== SEARCH
# Everything below runs only when this file is executed directly. It reads whole
# files, which is fine: that is TUNING, not detection. The Detector above still
# sees one sample at a time.
if __name__ == "__main__":
    import glob
    import itertools
    import os
    import sys
    import time

    import numpy as np
    from scipy.signal import lfilter

    HERE = os.path.dirname(os.path.abspath(__file__))
    TOOLS = os.path.dirname(HERE)
    ROOT = os.path.dirname(TOOLS)
    sys.path.insert(0, TOOLS)
    import replay_intent as R
    import score_intent as S

    TICK = R.TICK_DIV
    FIRE_TICKS = 4        # DEBOUNCE_MS 80 / 20 ms per tick
    RELAX_TICKS = 8       # RELAX_MS 150 / 20, rounded up

    # ------------------------------------------------ load once, into numpy
    def load_trials(stems):
        out = []
        for stem in stems:
            csvs = glob.glob(os.path.join(ROOT, "data", f"*{stem}.csv"))
            keys = glob.glob(os.path.join(ROOT, "data", "answer_key",
                                          f"*{stem}_key.csv"))
            if not csvs or not keys:
                continue
            key = np.array(S.load_key(keys[0]), dtype=np.int8)
            if not (key == 1).any():
                continue                       # nothing to detect; scorer skips
            _, A, B, _ = R.load(csvs[0])
            b = np.asarray(B, dtype=np.float64)
            n = min(len(b) // TICK + (len(b) % TICK > 0), len(key))
            key = key[:n]
            active, rest = key == 1, key == 0
            bursts = np.array(S.runs(active.tolist()), dtype=int).reshape(-1, 2)
            out.append(dict(
                stem=stem, b=b, live=b >= R.DEAD_COUNTS, n=n,
                dead=b[::TICK][:n] < R.DEAD_COUNTS,
                active=active, rest=rest, bursts=bursts,
                cum_active=np.concatenate(([0], np.cumsum(active))),
                na=int(active.sum()), nr=int(rest.sum()),
                rest_min=rest.sum() * R.TICK_S / 60.0,
                idx=np.arange(n)))
        return out

    # --------------------------------------------- front end (the slow part)
    # The floor loop is the whole cost of the search and does NOT depend on
    # alpha, so it is cached and every alpha reuses it.
    _cache = {}

    def excess(tr, a_dn, a_up, cap):
        """max(0, chB - floor) over the LIVE samples only."""
        k = (tr["stem"], a_dn, a_up, cap)
        exc = _cache.get(k)
        if exc is not None:
            return exc
        bl = tr["b"][tr["live"]]
        exc = np.empty(bl.size)
        base = None
        for i in range(bl.size):                   # inherently sequential
            x = bl[i]
            if base is None:
                base = x if x < cap else cap
            base += (a_dn if x < base else a_up) * (x - base)
            if base > cap:
                base = cap
            d = x - base
            exc[i] = d if d > 0.0 else 0.0
        _cache[k] = exc
        return exc

    def act_ticks(tr, a_dn, a_up, cap, alpha):
        """chB -> per-tick de-jittered activity, exactly as Detector would."""
        k = (tr["stem"], a_dn, a_up, cap, alpha)
        a = _cache.get(k)
        if a is None:
            # the EMA freezes across brown-outs, so filtering the live
            # subsequence and scattering back is exact, not an approximation
            act = np.zeros(tr["b"].size)
            act[tr["live"]] = lfilter([alpha], [1.0, alpha - 1.0],
                                      excess(tr, a_dn, a_up, cap))
            a = _cache[k] = act[::TICK][:tr["n"]]
        return a

    def runlen(m):
        """Length of the True run ending at each index."""
        i = np.arange(m.size)
        return i - np.maximum.accumulate(np.where(m, -1, i))

    def closing_mask(e, dead, t_on, t_off):
        """The shared FSM's ST_CLOSING, vectorised.

        Entering CLOSING needs 4 consecutive ticks above t_on and happens from
        EVERY other state; leaving it needs 8 below t_off. So CLOSING is exactly
        a set/reset latch, and OPEN/HOLDING/OPENING never enter the score.
        """
        live = ~dead
        fire = runlen(live & (e > t_on)) >= FIRE_TICKS
        reset = (runlen(live & (e < t_off)) >= RELAX_TICKS) | dead
        i = np.arange(e.size)
        return (np.maximum.accumulate(np.where(fire, i, -1)) >
                np.maximum.accumulate(np.where(reset, i, -1)))

    # ------------------------------------------------------- the scorer, fast
    BIG = 1 << 30

    def fast_score(tr, closing):
        """Same numbers as score_intent.score_one, without the Python loops."""
        n, act_m, rest_m = tr["n"], tr["active"], tr["rest"]
        nxt = np.minimum.accumulate(
            np.where(closing, tr["idx"], BIG)[::-1])[::-1]
        s, e = tr["bursts"][:, 0], tr["bursts"][:, 1]
        hit = nxt[np.maximum(0, s - S.GRACE_TICKS)]
        ok = hit < e
        miss = 100.0 * (1 - ok.mean()) if s.size else np.nan
        v = np.sort((hit[ok] - s[ok]) * R.TICK_S * 1000)   # score_one's median:
        lat = v[v.size // 2] if v.size else np.nan         # upper of the two
        d = np.diff(np.concatenate(([0], closing.view(np.int8), [0])))
        cs, ce = np.flatnonzero(d == 1), np.flatnonzero(d == -1)
        ca = tr["cum_active"]
        fa = int(np.count_nonzero(ca[ce] - ca[cs] == 0))
        fa_min = fa / tr["rest_min"] if tr["rest_min"] > 0.05 else np.nan
        tp = int(np.count_nonzero(act_m & closing))
        fp = int(np.count_nonzero(rest_m & closing))
        fn = tr["na"] - tp
        cover = 100.0 * tp / tr["na"] if tr["na"] else np.nan
        quiet = 100.0 * (tr["nr"] - fp) / tr["nr"] if tr["nr"] else np.nan
        den = 2 * tp + fp + fn
        return (lat, miss, fa_min, cover, quiet, 200.0 * tp / den if den else np.nan)

    def nanmean(v):
        v = [x for x in v if x == x]
        return sum(v) / len(v) if v else float("nan")

    # ONE objective, fixed before the search and never touched again.
    # F1 is the headline; a self-grabbing hand is punished 10 points per FA per
    # minute of rest; latency costs 1 point per 50 ms beyond the 200 ms target.
    def objective(rows):
        lat, miss, fa, cover, quiet, f1 = (nanmean([r[i] for r in rows])
                                           for i in range(6))
        j = f1 - 10.0 * fa - max(0.0, lat - 200.0) / 50.0
        return j, (lat, miss, fa, cover, quiet, f1)

    def evaluate(trials, a_dn, a_up, cap, alpha, t_on, t_off):
        return objective([
            fast_score(tr, closing_mask(act_ticks(tr, a_dn, a_up, cap, alpha),
                                        tr["dead"], t_on, t_off))
            for tr in trials])

    # ------------------------------------------------------------- the grids
    def sweep(trials, dns, ups, caps, alphas, ons, offs, label):
        best, t0, k = [], time.time(), 0
        for a_dn, a_up, cap in itertools.product(dns, ups, caps):
            _cache.clear()                     # bound memory: one floor at a time
            for alpha, t_on, t_off in itertools.product(alphas, ons, offs):
                if t_off >= t_on:
                    continue
                j, m = evaluate(trials, a_dn, a_up, cap, alpha, t_on, t_off)
                best.append((j, (a_dn, a_up, cap, alpha, t_on, t_off), m))
                k += 1
        _cache.clear()
        best.sort(key=lambda x: -x[0])
        print(f"\n{label}: {k} combos in {time.time() - t0:.1f}s")
        print(f"  {'J':>6}  {'A_DN':>6} {'A_UP':>8} {'cap':>5} {'alpha':>6} "
              f"{'t_on':>5} {'t_off':>5} | {'lat':>5} {'miss':>5} {'FA':>5} "
              f"{'cov':>4} {'qui':>4} {'F1':>4}")
        for j, p, m in best[:10]:
            print(f"  {j:6.1f}  {p[0]:6.3f} {p[1]:8.1e} {p[2]:5.0f} {p[3]:6.3f} "
                  f"{p[4]:5.0f} {p[5]:5.0f} | {m[0]:5.0f} {m[1]:5.0f} {m[2]:5.2f} "
                  f"{m[3]:4.0f} {m[4]:4.0f} {m[5]:4.0f}")
        return best[0]

    fit = load_trials(S.FIT)
    print(f"FIT trials: {', '.join(t['stem'] for t in fit)}  "
          f"({sum(t['b'].size for t in fit)} samples)")

    # --- stage 0: is the vectorised pipeline really the shared FSM + scorer?
    def verify(tr, p):
        global A_DN, A_UP, BASE_CAP, ALPHA, T_ON, T_OFF
        A_DN, A_UP, BASE_CAP, ALPHA, T_ON, T_OFF = p
        det = Detector(R.SAMPLE_HZ)
        act = [det.update(x, 0)[0] for x in tr["b"]]
        _, states = R.run_fsm(act, T_ON, T_OFF, T_FULL, raw=tr["b"].tolist())
        ref = np.array([s == R.ST_CLOSING for s in states])[:tr["n"]]
        e = act_ticks(tr, *p[:4])
        mine = closing_mask(e, tr["dead"], p[4], p[5])
        assert np.array_equal(ref, mine), f"{tr['stem']}: {np.sum(ref != mine)} ticks differ"
        got = fast_score(tr, mine)
        want = S.score_one(states, S.load_key(glob.glob(os.path.join(
            ROOT, "data", "answer_key", f"*{tr['stem']}_key.csv"))[0]))
        for a, b in zip(got, [want[k] for k in ("lat", "miss", "fa", "cover",
                                                "quiet", "f1")]):
            assert (a != a and b != b) or abs(a - b) < 1e-6, f"{a} != {b}"

    for tr in fit:
        verify(tr, (0.02, 2e-4, 2600.0, 0.04, 750.0, 350.0))
    print("vectorised FSM + scorer match replay_intent/score_intent exactly")

    # NO_CAP is above the ADC rail, so including it in the cap grid lets the
    # search switch the clamp OFF entirely. If it wins, the clamp is dead code.
    NO_CAP = 4200.0

    def around(v, lo, hi, mul):
        return sorted({min(max(v * m, lo), hi) for m in mul})

    def optimise(dns, ups, label):
        """One coarse pass, then refinements that re-centre on the winner.

        Re-centring is what lets a parameter that lands on a grid EDGE walk off
        it instead of being baked in; the neighbourhood shrinks as it converges.
        The tracked model and the frozen ablation both go through this exact
        pipeline, or the comparison between them means nothing.
        """
        win = sweep(fit, dns, ups,
                    caps=[1600.0, 1900.0, 2200.0, 2600.0, 3000.0, NO_CAP],
                    alphas=[0.01, 0.04, 0.16],
                    ons=[200.0, 400.0, 600.0, 900.0, 1300.0, 1800.0],
                    offs=[50.0, 150.0, 300.0, 600.0, 1000.0],
                    label=f"{label} coarse")
        for rnd, mul in enumerate(((0.25, 0.5, 1.0, 2.0, 4.0),
                                   (0.5, 0.7, 1.0, 1.4, 2.0),
                                   (0.8, 0.9, 1.0, 1.12, 1.25)), 1):
            a_dn, a_up, cap, alpha, t_on, t_off = win[1]
            r = sweep(
                fit,
                dns=around(a_dn, 1e-4, 0.9, mul) if a_dn else [0.0],
                ups=around(a_up, 1e-7, 0.5, mul) if a_up else [0.0],
                caps=sorted({NO_CAP} | {min(max(cap * m, 1400.0), NO_CAP)
                                        for m in (0.9, 0.95, 1.0, 1.05, 1.1)}),
                alphas=around(alpha, 0.002, 0.9, mul),
                ons=around(t_on, 50.0, 3000.0, mul),
                offs=around(t_off, 20.0, 2500.0, mul),
                label=f"{label} refine {rnd}")
            if r[0] > win[0]:
                win = r
        return win

    win = optimise([0.001, 0.003, 0.01, 0.03, 0.1, 0.3],
                   [1e-6, 1e-5, 1e-4, 1e-3], "TRACKED")
    a_dn, a_up, cap, alpha, t_on, t_off = win[1]

    # --- stage 3: t_full. The scorer reads only `states`, and t_full only
    # scales `intent` inside CLOSING, so this should come out perfectly flat.
    print("\nstage 3: t_full sweep (expected flat — the scorer cannot see it)")
    js = []
    for tf in [t_on * 1.2, 1500.0, 2000.0, 2500.0, 3000.0]:
        T_FULL = tf
        rows = [fast_score(tr, closing_mask(act_ticks(tr, a_dn, a_up, cap, alpha),
                                            tr["dead"], t_on, t_off))
                for tr in fit]
        js.append((tf, objective(rows)[0]))
        print(f"  t_full={tf:6.0f}  J={js[-1][1]:6.2f}")
    assert max(j for _, j in js) - min(j for _, j in js) < 1e-9, \
        "t_full changed the score — the latch derivation is wrong"
    # score-invariant, so it is set by the only thing that does depend on it:
    # grip force should reach maximum when chB is railed at 4095 above a
    # typical ~2050 rest, i.e. t_full ~ 2000 counts of excess.
    t_full = 2000.0

    # --- stage 4: ablation. Did the floor TRACKING earn its keep, or is this a
    # constant threshold in disguise? a_dn = a_up = 0 freezes the floor at its
    # warm-up value: a one-shot tare and nothing more. Same pipeline, so the
    # only difference between the two numbers is the tracking itself.
    frozen = optimise([0.0], [0.0], "FROZEN (ablation)")
    print(f"\n  tracked J={win[0]:.2f}  frozen J={frozen[0]:.2f}  "
          f"=> tracking is worth {win[0] - frozen[0]:+.2f} J on FIT")
    print(f"  frozen best: {frozen[1]}")

    print(f"\nWINNER  J={win[0]:.2f}")
    print(f"  A_DN = {a_dn}\n  A_UP = {a_up}\n  BASE_CAP = {cap}\n"
          f"  ALPHA = {alpha}\n  T_ON = {t_on}\n  T_OFF = {t_off}\n"
          f"  T_FULL = {t_full}")
    print("\nbake these in as the module constants, then:")
    print("  python tools/score_intent.py --algo brutetared --per-trial")

    A_DN, A_UP, BASE_CAP, ALPHA, T_ON, T_OFF, T_FULL = (
        a_dn, a_up, cap, alpha, t_on, t_off, t_full)
    _selfcheck()
