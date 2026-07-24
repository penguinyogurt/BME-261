#!/usr/bin/env python3
"""Build an offline 'answer key' of when the wearer was actually contracting.

There is NO recorded ground truth in this project — nobody logged "I squeezed
now" — so this is a strong offline ESTIMATE, not truth. What makes it better
than any causal detector, and therefore usable as a reference to score them
against, is that it is allowed to cheat in ways a live device cannot:

  * zero-phase (symmetric) smoothing, so onsets are not dragged late by filter
    lag the way every causal envelope drags them;
  * whole-file and cross-trial statistics for the rest level and the threshold;
  * duration priors applied in both directions (a 40 ms blip is not a grip, and
    a 60 ms dip does not end one);
  * onset refinement that walks BACKWARD from a confirmed burst to the moment
    the raw signal actually left rest.

Labels are written at the FSM tick rate so they line up 1:1 with run_fsm output:
    1  ACTIVE   contracting
    0  REST     relaxed
   -1  UNKNOWN  front-end browned out / unusable — excluded from all scoring

    python tools/make_answer_key.py              # all 7-23 trials + review plots
    python tools/make_answer_key.py --no-plot
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np
from scipy.ndimage import gaussian_filter1d, percentile_filter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import replay_intent as R

DEAD = 300              # chB below this: front-end unpowered
# Guards around a brown-out are ASYMMETRIC. Losing power is a fast collapse, but
# when power returns the analog peak detector charges through its RC and
# overshoots to ~3200 counts before settling over ~2 s. That transient is not a
# contraction — it is what made the firmware latch a full grip for 50 s on
# 200828 — so it must be excluded, not labelled ACTIVE.
PRE_DEAD_GUARD_MS = 300
POST_DEAD_GUARD_MS = 2500
SMOOTH_MS = 30          # zero-phase gaussian; wide enough to kill ADC jitter
REST_BAND = (1500, 2600)   # hardware prior: a plausible resting envelope
REST_WIN_S = 25.0       # centered window for the tracked rest level
REST_PCT = 25           # ...and the percentile taken within it
SAT = 4000              # at/above this chB is railed => unambiguously active
MIN_BURST_MS = 120      # shorter than this is not a deliberate grip
MIN_GAP_MS = 150        # a dip shorter than this is inside one contraction
ONSET_DELTA = 0.15      # onset = where raw first passes 15% of the burst step

# Thresholds are scaled to each trial's OWN resting noise, not chosen by Otsu.
# Otsu assumes a bimodal histogram; the pooled excess-over-rest distribution
# here is one rest peak plus a smooth monotonic tail to the rail, with no
# valley, so Otsu split rest-vs-saturation at +2195 counts and labelled whole
# trials of visible bursts as 0% active. Resting MAD is only 20-77 counts, so
# "clearly above resting noise" is both better founded and far more stable.
#
# A candidate burst is found at the LOW threshold (which fixes its edges), then
# QUALIFIED by how high it actually got:
#     peak >= HI    -> ACTIVE, we are confident this was a deliberate grip
#     peak >= AMB   -> UNKNOWN, too big to call rest and too small to be sure;
#                      excluded, so a detector is neither rewarded nor punished
#     otherwise     -> REST
# The middle band matters: those medium bumps have the same sharp-rise /
# slow-decay shape as real grips, so calling them REST would penalise a
# detector for behaviour that may well be correct.
#
# HI was set by review against the recordings themselves: at rest+900 the
# operator confirmed that bumps being dumped into the excluded band were in
# fact real contractions, so the bar came down to rest+300 (~7x MAD on a
# typical trial). The MAD scaling is kept so a noisier trial needs a
# proportionally higher bar, with 300 counts as the floor.
K_LOW, LOW_MIN = 3.0, 150.0     # burst EXTENT (edges)
K_AMB, AMB_MIN = 4.5, 200.0     # above this, no longer confidently rest
K_HI, HI_MIN = 7.0, 300.0       # above this, confidently a deliberate grip


def rest_mad(b, ok, rest):
    """Robust spread of the resting envelope, in counts (rest may be an array)."""
    near = (b - rest)[ok] if np.ndim(rest) else b[ok] - rest
    near = near[np.abs(near) < 150]
    if near.size < 200:
        return 45.0                     # typical for this front end
    return max(float(np.median(np.abs(near - np.median(near)))), 5.0)


def runs(mask):
    """[(start, stop)] for each True run in a boolean array."""
    d = np.diff(np.concatenate(([0], mask.view(np.int8), [0])))
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


def valid_mask(b, fs):
    """False wherever the front end was dead, plus its collapse and recharge."""
    ok = b >= DEAD
    pre = int(PRE_DEAD_GUARD_MS / 1000 * fs)
    post = int(POST_DEAD_GUARD_MS / 1000 * fs)
    for s, e in runs(~ok):
        ok[max(0, s - pre):min(len(ok), e + post)] = False
    return ok


def rest_level(b, ok, fallback):
    """Whole-trial resting envelope: the MODE inside the plausible band.

    A low percentile is wrong on a mostly-active trial — p10 then sits inside
    the contraction. The histogram peak within the hardware's plausible band is
    where the signal genuinely spends its idle time. Used only as the anchor for
    the tracked rest below, and for trials with no observable rest at all
    (185816 is railed from sample 0).
    """
    live = b[ok]
    live = live[(live >= REST_BAND[0]) & (live <= REST_BAND[1])]
    if live.size < 500:
        return fallback
    hist, edges = np.histogram(live, bins=110)      # ~10 counts per bin
    k = int(np.argmax(hist))
    return float((edges[k] + edges[k + 1]) / 2)


def rest_track(b, ok, fs, anchor):
    """Slowly-varying resting level, estimated acausally (CENTERED window).

    The resting floor does not just drift slowly — on 171239 it steps from
    ~2050 to ~2500 after the first hard contraction and stays there. One rest
    value per trial then sits hundreds of counts above true rest for the early
    part of the file and mislabels it.

    A centered rolling low-percentile follows that, and because the window is
    symmetric it introduces no lag, so burst edges stay where they belong. A
    causal detector could never do this — which is precisely why the key may.
    Computed on 0.2 s blocks and interpolated back, so the wide window is cheap.
    """
    blk = max(1, int(0.2 * fs))
    n = len(b) // blk
    if n < 3:
        return np.full(len(b), anchor)
    trimmed = slice(0, n * blk)
    bb = b[trimmed].reshape(n, blk)
    oo = ok[trimmed].reshape(n, blk)
    # A block counts only if it is almost ENTIRELY live. Accepting any block
    # with a single live sample let brown-out ramp values (952, 1212 counts on
    # 200828) in as if they were resting levels.
    blocks = np.full(n, np.nan)
    frac = oo.mean(axis=1)
    for i in np.flatnonzero(frac >= 0.8):
        blocks[i] = np.percentile(bb[i][oo[i]], 20)
    good = ~np.isnan(blocks)
    if not good.any():
        return np.full(len(b), anchor)
    # Dead stretches take the trial anchor, NOT interpolation: np.interp
    # extrapolates the edge value, which pushed one ramp artifact across every
    # dead block and dragged the whole tracked rest down to it. Those stretches
    # are excluded from scoring anyway, so a neutral fill costs nothing.
    blocks[~good] = anchor
    win = max(3, int(REST_WIN_S / 0.2) | 1)            # odd => truly centered
    tracked = percentile_filter(blocks, REST_PCT, size=win, mode="nearest")
    tracked = np.clip(tracked, REST_BAND[0], REST_BAND[1])
    centers = np.arange(n) * blk + blk / 2.0
    full = np.interp(np.arange(len(b)), centers, tracked)
    return full


def label_trial(b, fs, rest, low, amb, hi):
    """chB -> per-sample {1 ACTIVE, 0 REST, -1 UNKNOWN}, acausally."""
    ok = valid_mask(b, fs)
    sm = gaussian_filter1d(b.astype(float), SMOOTH_MS / 1000 * fs)

    act = np.zeros(len(b), bool)
    unsure = np.zeros(len(b), bool)
    for s, e in runs((sm > rest + low) & ok):     # candidates, edges at the low line
        # rest is a time series, so qualify each burst against the rest level
        # local to it rather than a single number for the whole file
        base = float(np.median(rest[s:e]))
        peak = float(sm[s:e].max())
        if peak >= base + hi or (b[s:e] >= SAT).any():
            act[s:e] = True                       # confident grip (or railed)
        elif peak >= base + amb:
            unsure[s:e] = True                    # too big for rest, too small to call

    # duration priors, applied in both directions
    for s, e in runs(act):
        if (e - s) < MIN_BURST_MS / 1000 * fs:
            act[s:e] = False
    for s, e in runs(~act & ok):
        if (e - s) < MIN_GAP_MS / 1000 * fs:
            act[s:e] = True

    # Onset refinement: the smoothed signal crosses late relative to the true
    # departure from rest. Walk back to where the RAW signal first rose past a
    # fraction of this burst's own step size. No causal method could do this,
    # which is exactly why the key is allowed to and they are scored on it.
    for s, e in runs(act):
        base = float(np.median(rest[s:e]))
        peak = float(np.percentile(b[s:e], 90))
        trip = base + ONSET_DELTA * max(peak - base, 1.0)
        j = s
        back = max(0, s - int(0.5 * fs))
        while j > back and b[j] > trip:
            j -= 1
        act[j:s] = ok[j:s]

    out = np.where(act, 1, 0).astype(np.int8)
    out[unsure & ~act] = -1                       # ambiguous: excluded from scoring
    out[~ok] = -1                                 # browned out: excluded too
    return out, sm


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--on", type=float, default=None,
                    help="override the pooled ON offset (counts above rest)")
    args = ap.parse_args()

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    files = sorted(glob.glob(os.path.join(root, "data", "emg_2026-07-23_*.csv")))
    fs = R.SAMPLE_HZ

    # ---- pass 1: a cross-trial rest level, for trials with no observable rest
    trials = []
    rests = []
    for f in files:
        _, _, B, I = R.load(f)
        b = np.asarray(B, dtype=float)
        ok = valid_mask(b, fs)
        live = b[ok]
        live = live[(live >= REST_BAND[0]) & (live <= REST_BAND[1])]
        if live.size >= 500:
            hist, edges = np.histogram(live, bins=110)
            k = int(np.argmax(hist))
            rests.append(float((edges[k] + edges[k + 1]) / 2))
        trials.append((f, b, ok))
    global_rest = float(np.median(rests)) if rests else 1900.0
    print(f"cross-trial rest = {global_rest:.0f} counts")

    outdir = os.path.join(root, "data", "answer_key")
    os.makedirs(outdir, exist_ok=True)
    print(f"\n{'trial':32} {'rest':>6} {'MAD':>5} {'HI':>6} {'ACTIVE%':>8} "
          f"{'REST%':>7} {'UNK%':>6} {'bursts':>7} {'med len':>8}")
    keys = []
    for f, b, ok in trials:
        anchor = rest_level(b, ok, global_rest)
        rest = rest_track(b, ok, fs, anchor)      # time-varying, centered
        mad = rest_mad(b, ok, rest)
        low = max(LOW_MIN, K_LOW * mad)
        amb = max(AMB_MIN, K_AMB * mad)
        hi = args.on if args.on is not None else max(HI_MIN, K_HI * mad)
        lab, sm = label_trial(b, fs, rest, low, amb, hi)
        tick = lab[::R.TICK_DIV]                 # align 1:1 with run_fsm output
        name = os.path.basename(f)
        with open(os.path.join(outdir, name.replace(".csv", "_key.csv")), "w",
                  newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["tick", "label"])
            w.writerows(enumerate(tick.tolist()))
        bursts = runs(tick == 1)
        lens = [(e - s) * R.TICK_S for s, e in bursts]
        print(f"{name:32} {np.median(rest):6.0f} {mad:5.0f} {hi:6.0f} "
              f"{np.mean(tick == 1) * 100:7.1f}% "
              f"{np.mean(tick == 0) * 100:6.1f}% {np.mean(tick == -1) * 100:5.1f}% "
              f"{len(bursts):7} {np.median(lens) if lens else 0:7.2f}s")
        keys.append((f, b, lab, sm, rest, hi, low))

    if not args.no_plot:
        plot_keys(keys, root, fs)
        print(f"\nreview plots in output/answer_key/")


def plot_keys(keys, root, fs):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    outdir = os.path.join(root, "output", "answer_key")
    os.makedirs(outdir, exist_ok=True)
    for f, b, lab, sm, rest, hi, low in keys:
        t = np.arange(len(b)) / fs
        fig, ax = plt.subplots(figsize=(14, 3.2))
        for s, e in runs(lab == 1):
            ax.axvspan(s / fs, e / fs, color="#eb6834", alpha=0.20, linewidth=0)
        for s, e in runs(lab == -1):
            ax.axvspan(s / fs, e / fs, color="#b0b0b0", alpha=0.30, linewidth=0)
        ax.plot(t, b, color="#9db8d2", linewidth=0.5, label="chB raw")
        ax.plot(t, sm, color="#2a78d6", linewidth=0.9, label="zero-phase smoothed")
        ax.plot(t, rest, color="#898781", linewidth=0.9, label="tracked rest")
        ax.plot(t, rest + hi, color="#c0392b", linewidth=0.9, linestyle="--",
                label="qualifying peak")
        ax.plot(t, rest + low, color="#7f8c8d", linewidth=0.8, linestyle=":",
                label="burst edge")
        ax.set_xlim(0, t[-1]); ax.set_ylim(0, 4300)
        ax.set_title(f"{os.path.basename(f)}   orange = ACTIVE, grey = EXCLUDED "
                     f"(brown-out or ambiguous),  rest~{np.median(rest):.0f}",
                     fontsize=10, loc="left")
        ax.set_xlabel("time (s)", fontsize=8)
        ax.grid(True, alpha=0.3); ax.tick_params(labelsize=8)
        ax.legend(fontsize=7, loc="upper right")
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, os.path.basename(f).replace(".csv", "_key.png")),
                    dpi=120)
        plt.close(fig)


if __name__ == "__main__":
    main()
