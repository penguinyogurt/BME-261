#!/usr/bin/env python3
"""Score a causal intent algorithm against the offline answer key.

    python tools/score_intent.py --algo hyst
    python tools/score_intent.py --algo hyst,quantile      # side by side
    python tools/score_intent.py --algo hyst --per-trial

Run tools/make_answer_key.py first; this reads data/answer_key/*.csv.

WHAT IS MEASURED (UNKNOWN ticks are excluded from everything)
  lat_ms    median delay from the key's onset to the hand starting to close.
            This is the number that decides whether the device feels responsive;
            under ~200 ms is the target.
  miss%     key bursts the detector never responded to at all.
  FA/min    false activations per minute of REST — the metric that decides
            whether the hand is usable, because a hand that grabs on its own is
            worse than one that is slow.
  cover%    of ACTIVE time, how much the hand was actually closing.
  quiet%    of REST time, how much the hand correctly stayed put.
  F1        harmonic mean of precision/recall on the ACTIVE class.

FIT vs TEST
Constants tuned by brute force will overfit whatever they are tuned on, so the
trials are split and both are reported. Only the TEST column is evidence. A
method whose FIT and TEST columns diverge has memorised the fit set.
"""
import argparse
import csv
import glob
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import replay_intent as R

# 185816 is JUNK, confirmed by the operator: chB is railed at 4095 throughout
# because the front end misbehaved, not because of a real contraction. The key
# therefore labels it 100% ACTIVE with no REST at all, so leaving it in would
# reward a detector for firing on railed noise and would contribute nothing to
# FA/min or quiet%. 185723 looks similar but IS a genuine sustained
# contraction, so it stays.
EXCLUDED = {"185816": "railed front end, not a real contraction"}

# Split fixed here, never per-experiment, so results stay comparable. FIT keeps
# trials with saturated BURSTS (185937, 193836) so saturation is still
# exercised; TEST holds the fully-railed genuine hold (185723) as a real
# generalisation challenge.
FIT = ["134851", "182349", "185937", "193836", "200457"]
TEST = ["134728", "171239", "185723", "190832", "195519", "200828", "202105"]
assert not (set(FIT) | set(TEST)) & set(EXCLUDED), "excluded trial left in a split"

GRACE_TICKS = 5          # a close up to 100 ms BEFORE the key onset still counts


def load_key(path):
    lab = []
    for row in list(csv.reader(open(path)))[1:]:
        if len(row) >= 2:
            lab.append(int(row[1]))
    return lab


def runs(mask):
    out, s = [], None
    for i, v in enumerate(mask):
        if v and s is None:
            s = i
        elif not v and s is not None:
            out.append((s, i)); s = None
    if s is not None:
        out.append((s, len(mask)))
    return out


def score_one(states, key):
    """Compare tick-aligned detector states to key labels."""
    n = min(len(states), len(key))
    states, key = states[:n], key[:n]
    # "the hand is responding to a contraction" = CLOSING. HOLDING is the latch
    # policy after the muscle relaxed, which the key says nothing about, so it
    # is treated as neither a hit nor a false alarm.
    closing = [s == R.ST_CLOSING for s in states]
    hold = [s == R.ST_HOLDING for s in states]
    active = [k == 1 for k in key]
    rest = [k == 0 for k in key]

    lat, missed, nb = [], 0, 0
    for s, e in runs(active):
        nb += 1
        hit = next((i for i in range(max(0, s - GRACE_TICKS), e) if closing[i]), None)
        if hit is None:
            missed += 1
        else:
            lat.append((hit - s) * R.TICK_S * 1000)

    # false activations: CLOSING runs that never overlap any ACTIVE tick, and
    # are not merely the hand still holding from a previous real grip
    fa = 0
    for s, e in runs(closing):
        if not any(active[i] for i in range(s, e)):
            fa += 1
    rest_min = sum(rest) * R.TICK_S / 60.0
    fa_min = fa / rest_min if rest_min > 0.05 else float("nan")

    na, nr = sum(active), sum(rest)
    cover = sum(1 for i in range(n) if active[i] and closing[i]) / na * 100 if na else float("nan")
    quiet = sum(1 for i in range(n) if rest[i] and not closing[i]) / nr * 100 if nr else float("nan")
    tp = sum(1 for i in range(n) if active[i] and closing[i])
    fp = sum(1 for i in range(n) if rest[i] and closing[i])
    fn = sum(1 for i in range(n) if active[i] and not closing[i])
    f1 = 2 * tp / (2 * tp + fp + fn) * 100 if (2 * tp + fp + fn) else float("nan")
    med = sorted(lat)[len(lat) // 2] if lat else float("nan")
    return dict(lat=med, miss=missed / nb * 100 if nb else float("nan"),
                fa=fa_min, cover=cover, quiet=quiet, f1=f1,
                bursts=nb, hold=sum(hold) / n * 100)


def agg(rows):
    """Pool trials by weighting each equally; nan-safe."""
    out = {}
    for k in ("lat", "miss", "fa", "cover", "quiet", "f1"):
        v = [r[k] for r in rows if r[k] == r[k]]
        out[k] = sum(v) / len(v) if v else float("nan")
    return out


def main():
    R.load_plugins()
    ap = argparse.ArgumentParser()
    ap.add_argument("--algo", default=None, help="comma-separated; default all plugins")
    ap.add_argument("--per-trial", action="store_true")
    args = ap.parse_args()

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    keydir = os.path.join(root, "data", "answer_key")
    if not os.path.isdir(keydir):
        sys.exit("no answer key yet — run: python tools/make_answer_key.py")

    names = ([a.strip() for a in args.algo.split(",")] if args.algo else
             [n for n, (fn, _) in R.ALGOS.items() if hasattr(fn, "module")])
    if not names:
        sys.exit("no causal algorithms found in tools/algos/")
    for n in names:
        if n not in R.ALGOS:
            sys.exit(f"unknown algo '{n}'. available: {', '.join(R.ALGOS)}")

    hdr = (f"{'algo':14} {'set':5} {'lat_ms':>7} {'miss%':>6} {'FA/min':>7} "
           f"{'cover%':>7} {'quiet%':>7} {'F1':>6}")
    print(hdr); print("-" * len(hdr))
    for name in names:
        per = {}
        for tag, group in (("FIT", FIT), ("TEST", TEST)):
            rows = []
            for stem in group:
                csvs = glob.glob(os.path.join(root, "data", f"*{stem}.csv"))
                keys = glob.glob(os.path.join(keydir, f"*{stem}_key.csv"))
                if not csvs or not keys:
                    continue
                key = load_key(keys[0])
                if not any(k == 1 for k in key):
                    continue                     # nothing to detect; skip
                _, A, B, _ = R.load(csvs[0])
                act, on, off, full = R.ALGOS[name][0](B, A, "chB")
                _, states = R.run_fsm(act, on, off, full, raw=B)
                r = score_one(states, key)
                rows.append(r); per[stem] = r
            if rows:
                a = agg(rows)
                print(f"{name:14} {tag:5} {a['lat']:7.0f} {a['miss']:6.0f} "
                      f"{a['fa']:7.1f} {a['cover']:7.0f} {a['quiet']:7.0f} {a['f1']:6.0f}")
        if args.per_trial:
            for stem in FIT + TEST:
                if stem in per:
                    r = per[stem]
                    print(f"    {stem:10} lat={r['lat']:6.0f} miss={r['miss']:4.0f}% "
                          f"FA/min={r['fa']:5.1f} cover={r['cover']:3.0f}% "
                          f"quiet={r['quiet']:3.0f}% F1={r['f1']:3.0f} "
                          f"({r['bursts']} bursts)")
        print()


if __name__ == "__main__":
    main()
