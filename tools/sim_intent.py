"""Replay a logged EMG CSV through a faithful port of the firmware's intent
logic, so we can see how the servo would be driven WITHOUT hardware.

It mirrors BME EMG Arduino Collection.ino exactly: the ~200 ms EMA envelope,
the calibration thresholds (fractions of rest..MVC span), and the latching
state machine (OPEN -> CLOSING -> HOLDING -> OPENING) with the same debounce,
dwell, and refresh-squeeze behaviour, ticked at 50 Hz.

Two differences forced by replaying a file instead of live hardware:
  1. --source picks which logged channel feeds the envelope. The firmware
     reads chA (PIN_RAW); in the captured data chA is the DEAD channel and the
     gesture lives in chB, so we can compare 'chA' (as-written) vs 'chB' (fix).
  2. Calibration can't be interactive, so rest/MVC are estimated from the
     file's own envelope distribution (20th / 99th percentile).

Usage:
    python sim_intent.py FILE.csv [--source chB|chA] [--no-plot]
    python sim_intent.py --all          # every csv in this folder, chB
"""
import argparse
import csv
import glob
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, "data")
OUT = os.path.join(ROOT, "output", "sim")

# ---- firmware constants (keep in sync with the .ino) ----
SAMPLE_HZ    = 500
TICK_DIV     = 10               # 50 Hz intent rate
TICK_MS      = 1000 // (SAMPLE_HZ // TICK_DIV)
ENV_ALPHA    = 0.01            # ~200 ms EMA at 500 Hz
T_LOW_FRAC   = 0.12
T_HIGH_FRAC  = 0.28
FULL_FRAC    = 0.80
DEBOUNCE_MS  = 80
RELAX_MS     = 150
OPEN_DWELL_MS = 2000
OPEN_DRIVE_MS = 2500
OPEN_SPEED   = 600
CLOSE_MIN    = 250
CAL_FLOOR    = 40             # span below this -> "not calibrated"

ST_OPEN, ST_CLOSING, ST_HOLDING, ST_OPENING = range(4)
STATE_NAMES = ["OPEN", "CLOSING", "HOLDING", "OPENING"]

# receiver-side mapping (servo-receiver/src/main.cpp), to show the pulse width
STOP_US, MIN_US, MAX_US, SPEED_SPAN, DEADBAND = 1500, 900, 2100, 450, 20


def load(path):
    t, a, b = [], [], []
    with open(path) as fh:
        r = csv.reader(fh)
        next(r, None)
        for row in r:
            if len(row) != 3:
                continue
            try:
                t.append(float(row[0])); a.append(int(row[1])); b.append(int(row[2]))
            except ValueError:
                continue
    t0 = t[0]
    return [x - t0 for x in t], a, b


def percentile(xs, p):
    s = sorted(xs)
    k = (len(s) - 1) * p / 100.0
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def intent_to_us(intent):
    if abs(intent) < DEADBAND:
        return STOP_US
    off = int(intent * SPEED_SPAN / 1000)
    return max(MIN_US, min(MAX_US, STOP_US + off))


def simulate(t, raw, source_signal, base):
    """Return per-tick arrays: time, env, intent, state. `base` is the rectify
    baseline (mean rest of the source); env = EMA(|source - base|)."""
    env = 0.0
    envs_all = []
    for x in source_signal:
        env += ENV_ALPHA * (abs(x - base) - env)
        envs_all.append(env)

    # calibration from the envelope distribution
    rest = percentile(envs_all, 20)
    mvc  = percentile(envs_all, 99)
    span = mvc - rest
    calibrated = span > CAL_FLOOR
    t_low  = rest + T_LOW_FRAC  * span
    t_high = rest + T_HIGH_FRAC * span
    full   = rest + FULL_FRAC   * span

    st = ST_OPEN
    above_high = below_high = below_low = 0
    open_start = 0.0
    out = {"t": [], "env": [], "intent": [], "state": [],
           "cal": calibrated, "rest": rest, "mvc": mvc,
           "t_low": t_low, "t_high": t_high, "full": full,
           "transitions": []}

    for i in range(0, len(source_signal), TICK_DIV):
        e = envs_all[i]
        now = t[i] * 1000.0
        intent = 0
        if calibrated:
            above_high = above_high + TICK_MS if e > t_high else 0
            below_high = below_high + TICK_MS if e < t_high else 0
            below_low  = below_low  + TICK_MS if e < t_low  else 0
            prev = st
            if st == ST_OPEN:
                if above_high >= DEBOUNCE_MS:
                    st = ST_CLOSING
            elif st == ST_CLOSING:
                frac = max(0.0, min(1.0, (e - t_high) / (full - t_high)))
                intent = CLOSE_MIN + int(frac * (1000 - CLOSE_MIN))
                if below_high >= RELAX_MS:
                    st = ST_HOLDING
            elif st == ST_HOLDING:
                if above_high >= DEBOUNCE_MS:
                    st = ST_CLOSING
                elif below_low >= OPEN_DWELL_MS:
                    st = ST_OPENING; open_start = now
            elif st == ST_OPENING:
                intent = -OPEN_SPEED
                if above_high >= DEBOUNCE_MS:
                    st = ST_CLOSING
                elif now - open_start >= OPEN_DRIVE_MS:
                    st = ST_OPEN
            if st != prev:
                out["transitions"].append((t[i], STATE_NAMES[st]))
        out["t"].append(t[i]); out["env"].append(e)
        out["intent"].append(intent); out["state"].append(st)
    return out


def summarize(name, out):
    dur = out["t"][-1] if out["t"] else 0
    n = len(out["state"])
    print(f"\n=== {name} ===")
    if not out["cal"]:
        print(f"  NOT CALIBRATED: rest={out['rest']:.0f} mvc={out['mvc']:.0f} "
              f"span={out['mvc']-out['rest']:.0f} (< {CAL_FLOOR}) -> intent flatlines at 0.")
        print("  (this is the 'dead channel' outcome — no usable gesture in this signal)")
        return
    frac = {s: out["state"].count(s) / n * 100 for s in range(4)}
    peak = max(out["intent"]); trough = min(out["intent"])
    print(f"  dur={dur:.1f}s  rest={out['rest']:.0f} mvc={out['mvc']:.0f} | "
          f"T_low={out['t_low']:.0f} T_high={out['t_high']:.0f} full={out['full']:.0f}")
    print(f"  time in state: OPEN {frac[0]:.0f}%  CLOSING {frac[1]:.0f}%  "
          f"HOLDING {frac[2]:.0f}%  OPENING {frac[3]:.0f}%")
    print(f"  intent range: {trough}..{peak}  ->  servo us "
          f"{intent_to_us(trough)}..{intent_to_us(peak)}")
    print(f"  {len(out['transitions'])} state changes:")
    for tt, nm in out["transitions"]:
        print(f"    {tt:6.2f}s -> {nm}")


def plot(name, t, chA, chB, out, source, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch

    SURF, INK, INK2, MUTE = "#fcfcfb", "#0b0b0b", "#52514e", "#898781"
    GRID, BASE, BLUE = "#e1e0d9", "#c3c2b7", "#2a78d6"
    BAND = {ST_CLOSING: "#eb6834", ST_HOLDING: "#1baf7a", ST_OPENING: "#eda100"}

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 6.2), sharex=True,
                                   height_ratios=[2, 1])
    fig.patch.set_facecolor(SURF)
    for ax in (ax1, ax2):
        ax.set_facecolor(SURF)
        ax.grid(True, color=GRID, linewidth=0.8)
        for s in ax.spines.values():
            s.set_color(BASE)
        ax.tick_params(colors=MUTE, labelsize=9)

    # state bands across both panels
    st = out["state"]; tt = out["t"]
    i = 0
    while i < len(st):
        j = i
        while j + 1 < len(st) and st[j + 1] == st[i]:
            j += 1
        if st[i] in BAND:
            x0 = tt[i]; x1 = tt[j] + (tt[1]-tt[0] if len(tt) > 1 else 0.02)
            for ax in (ax1, ax2):
                ax.axvspan(x0, x1, color=BAND[st[i]], alpha=0.13, linewidth=0)
        i = j + 1

    src = chB if source == "chB" else chA
    ax1.plot(t, src, color=BLUE, linewidth=0.7, label=f"{source} (source)")
    ax1.plot(out["t"], out["env"], color=INK, linewidth=1.4, label="envelope (EMA)")
    for lvl, lab in [(out["t_low"], "T_low"), (out["t_high"], "T_high"),
                     (out["full"], "full")]:
        ax1.axhline(lvl, color=MUTE, linewidth=1.0, linestyle=(0, (4, 3)))
        ax1.text(t[-1], lvl, f" {lab}", color=INK2, fontsize=8, va="center")
    ax1.set_ylabel("counts", color=INK2, fontsize=9)
    ax1.set_title(f"{name}  —  intent replay (source: {source})",
                  color=INK, fontsize=11, loc="left")

    ax2.axhline(0, color=BASE, linewidth=1.0)
    ax2.plot(out["t"], out["intent"], color=BLUE, linewidth=1.4)
    ax2.fill_between(out["t"], out["intent"], 0, color=BLUE, alpha=0.10)
    ax2.set_ylim(-1050, 1050)
    ax2.set_ylabel("intent", color=INK2, fontsize=9)
    ax2.set_xlabel("time (s)", color=INK2, fontsize=9)
    ax2.text(0.01, 0.90, "close +", transform=ax2.transAxes, color=MUTE, fontsize=8)
    ax2.text(0.01, 0.05, "open −", transform=ax2.transAxes, color=MUTE, fontsize=8)

    handles = ([Patch(facecolor=BLUE, alpha=0.6, label=f"{source} source")] +
               [Patch(facecolor=BAND[s], alpha=0.4, label=STATE_NAMES[s])
                for s in (ST_CLOSING, ST_HOLDING, ST_OPENING)])
    ax1.legend(handles=handles, loc="upper right", fontsize=8, framealpha=0.9,
               facecolor=SURF, edgecolor=GRID)

    fig.tight_layout()
    fig.savefig(path, dpi=130, facecolor=SURF)
    plt.close(fig)


def run(path, source, do_plot):
    name = os.path.basename(path)
    t, chA, chB = load(path)
    src = chB if source == "chB" else chA
    base = sum(src[:250]) / min(250, len(src))   # rest baseline (like self-test)
    out = simulate(t, src, src, base)
    summarize(f"{name} [{source}]", out)
    if do_plot and out["cal"]:
        outdir = OUT
        os.makedirs(outdir, exist_ok=True)
        png = os.path.join(outdir, name.replace(".csv", f"_{source}.png"))
        plot(name, t, chA, chB, out, source, png)
        print(f"  plot -> {os.path.relpath(png)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file", nargs="?")
    ap.add_argument("--source", choices=["chA", "chB"], default="chB")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    if args.all:
        files = sorted(glob.glob(os.path.join(DATA, "*.csv")))
    elif args.file:
        files = [args.file]
    else:
        print("give a FILE.csv or --all"); sys.exit(1)

    for f in files:
        run(f, args.source, not args.no_plot)


if __name__ == "__main__":
    main()
