"""Plot a debug_session log: EMG envelope vs the intent it produced vs what the
servo actually did. This is the "does it line up with what we expect" view.

    python tools/plot_debug.py                 # newest output/debug_*.log
    python tools/plot_debug.py output/debug_20260723_174324.log

Reads the three streams the log interleaves:
  [EMG] "a,b"  - chA,chB while in plot mode; env,intent once 'i' was sent
  [SRV] "RX .. intent=.. state=.. | us=.. grip=..%"
  [EMG] "saved ch X: rest=.. mvc=.. | T_low=.. T_high=.. full=.."
and draws them on one shared time axis.
"""
import glob
import os
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SURF, INK, INK2, MUTE = "#fcfcfb", "#0b0b0b", "#52514e", "#898781"
GRID, BASE, BLUE, ORANGE = "#e1e0d9", "#c3c2b7", "#2a78d6", "#eb6834"

RX = re.compile(r"intent=(-?\d+) state=(\w+) cal=(\d+) \| us=(\d+) grip=(-?\d+)%")
THR = re.compile(r"T_low=(\d+) T_high=(\d+) full=(\d+)")


def parse(path):
    mode = "p"                      # firmware boots into plot mode
    raw, env, srv = [], [], []      # (t,chA,chB) / (t,env,intent) / (t,intent,us,grip,state)
    thr = None
    for line in open(path, encoding="utf-8", errors="ignore"):
        m = re.match(r"\s*([\d.]+) \[(\w+)\] (.*)", line)
        if not m:
            continue
        t, tag, body = float(m.group(1)), m.group(2), m.group(3)
        if tag == "CMD":
            if body.startswith("i"):
                mode = "i"
            elif body.startswith("p"):
                mode = "p"
            continue
        if tag == "EMG":
            s = THR.search(body)
            if s:                       # keep the LAST set (the one in force)
                thr = tuple(int(x) for x in s.groups())
            parts = body.split(",")
            if len(parts) == 2:
                try:
                    x, y = int(parts[0]), int(parts[1])
                except ValueError:
                    continue
                # disambiguate: intent is bounded +/-1000, chB rests in the 1000s
                if mode == "i" and -1000 <= y <= 1000:
                    env.append((t, x, y))
                else:
                    raw.append((t, x, y))
        elif tag == "SRV":
            s = RX.search(body)
            if s:
                srv.append((t, int(s.group(1)), int(s.group(4)),
                            int(s.group(5)), s.group(2)))
    return raw, env, srv, thr


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        logs = sorted(glob.glob(os.path.join(root, "output", "debug_*.log")))
        if not logs:
            sys.exit("no output/debug_*.log found")
        path = logs[-1]
    raw, env, srv, thr = parse(path)
    name = os.path.basename(path)
    print(f"{name}: {len(raw)} plot samples, {len(env)} intent samples, "
          f"{len(srv)} servo records, thresholds={thr}")

    fig, axes = plt.subplots(4, 1, figsize=(12, 9), sharex=True,
                             height_ratios=[1, 1, 1, 1])
    fig.patch.set_facecolor(SURF)
    for ax in axes:
        ax.set_facecolor(SURF); ax.grid(True, color=GRID, linewidth=0.8)
        for s in ax.spines.values(): s.set_color(BASE)
        ax.tick_params(colors=MUTE, labelsize=8)

    # 1: raw channels while in plot mode
    if raw:
        axes[0].plot([r[0] for r in raw], [r[2] for r in raw],
                     color=BLUE, linewidth=0.7, label="chB (analog envelope)")
        axes[0].plot([r[0] for r in raw], [r[1] for r in raw],
                     color=MUTE, linewidth=0.6, label="chA (raw)")
        axes[0].legend(loc="upper right", fontsize=7.5, facecolor=SURF, edgecolor=GRID)
    axes[0].set_ylabel("counts", color=INK2, fontsize=8)
    axes[0].set_title(f"{name}  —  EMG signal vs intent vs servo", color=INK,
                      fontsize=12, loc="left")

    # 2: the envelope the state machine actually keys off, with its thresholds
    if env:
        axes[1].plot([e[0] for e in env], [e[1] for e in env],
                     color=INK, linewidth=1.1)
    if thr:
        for lvl, lab in zip(thr, ("T_low", "T_high", "full")):
            axes[1].axhline(lvl, color=MUTE, linewidth=0.9, linestyle=(0, (4, 3)))
            axes[1].annotate(lab, (0.995, lvl), xycoords=("axes fraction", "data"),
                             color=INK2, fontsize=7, va="center", ha="right")
    axes[1].set_ylabel("envelope", color=INK2, fontsize=8)

    # 3: intent the EMG board sent
    if env:
        axes[2].plot([e[0] for e in env], [e[2] for e in env],
                     color=ORANGE, linewidth=1.2)
        axes[2].fill_between([e[0] for e in env], [e[2] for e in env], 0,
                             color=ORANGE, alpha=0.12)
    axes[2].axhline(0, color=BASE, linewidth=0.9)
    axes[2].set_ylim(-1100, 1100); axes[2].set_yticks([-1000, 0, 1000])
    axes[2].set_ylabel("intent", color=INK2, fontsize=8)

    # 4: what the servo was actually commanded
    if srv:
        axes[3].plot([s[0] for s in srv], [s[2] for s in srv],
                     color=BLUE, linewidth=1.2)
        axes[3].axhline(1500, color=BASE, linewidth=0.9)
        axes[3].set_ylim(850, 2150)
    axes[3].set_ylabel("servo us", color=INK2, fontsize=8)
    axes[3].set_xlabel("time (s)", color=INK2, fontsize=8)

    out = os.path.join(root, "output", name.replace(".log", ".png"))
    fig.savefig(out, dpi=130, facecolor=SURF, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {os.path.relpath(out, root)}")


if __name__ == "__main__":
    main()
