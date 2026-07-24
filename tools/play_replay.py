#!/usr/bin/env python3
"""Play a logged trial back through intent algorithms as if it were live.

Same rolling-window feel as emg_bringup_viewer.py, but the samples come from a
CSV instead of the serial port, and one or more detectors are driven through
them sample-by-sample in arrival order. Nothing is precomputed, so what you see
is exactly what the firmware would have had at that instant.

    python tools/play_replay.py data/emg_2026-07-23_193836.csv
    python tools/play_replay.py data/emg_2026-07-23_185816.csv --algo hyst,quantile,slope
    python tools/play_replay.py data/emg_2026-07-23_193836.csv --speed 4

KEYS
    space   pause / resume          up / down   speed x2 / /2
    right   skip forward 5 s        left        back 5 s (re-runs from the
    r       restart                             start; detectors are causal)
    q       quit
"""
import argparse
import copy
import os
import sys
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import replay_intent as R

WINDOW_S = 6.0                  # seconds of history on screen
SNAP_S = 5.0                    # detector-state snapshot interval, for seeking back
Y_TOP = 3.5                     # activity axis, in multiples of t_on
# State is shown as the label's TEXT colour, not a background box: a rounded
# bbox is a FancyBboxPatch that re-computes its path on every draw, and with
# one per algorithm that measured 91% of the total frame time.
STATE_COLORS = ["#52514e", "#c0392b", "#b8860b", "#2a78d6"]   # OPEN/CLOS/HOLD/OPNG


def main():
    R.load_plugins()
    ap = argparse.ArgumentParser()
    ap.add_argument("file")
    ap.add_argument("--algo", default="hyst",
                    help="comma-separated; one activity panel each")
    ap.add_argument("--src", default="chB", choices=["chB", "chA"])
    ap.add_argument("--speed", type=float, default=1.0, help="1.0 = real time")
    ap.add_argument("--window", type=float, default=WINDOW_S, help="seconds on screen")
    args = ap.parse_args()

    names = [a.strip() for a in args.algo.split(",") if a.strip()]
    for n in names:
        if n not in R.ALGOS:
            sys.exit(f"unknown algo '{n}'. available: {', '.join(R.ALGOS)}")
    # only streaming plugins carry .module; the legacy batch algos derive their
    # thresholds from whole-file percentiles and have no live form at all
    bad = [n for n in names if not hasattr(R.ALGOS[n][0], "module")]
    if bad:
        sys.exit(f"{', '.join(bad)}: non-causal (whole-file percentiles), so there "
                 f"is nothing to play live. Use a plugin from tools/algos/.")

    T, A, B, I = R.load(args.file)
    n_total = len(B)
    recorded = [x for x in I if x is not None]
    win = int(args.window * R.SAMPLE_HZ)
    # Draw ~1200 points per line. A 6 s window at 500 Hz is 3000 samples across
    # roughly 1000 px, so plotting every sample costs 3x the draw time for
    # detail no screen can show. The detectors still see every sample; this is
    # display only.
    # ponytail: plain striding, so a 1-sample spike can fall between drawn
    # points. chB is already an analog envelope so it reads fine; switch to
    # min/max-per-bin decimation if a sharper signal ever gets plotted here.
    dec = max(1, win // 1200)
    print(f"{os.path.basename(args.file)}: {n_total} samples, "
          f"{n_total / R.SAMPLE_HZ:.1f} s")

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    # ---- live state: one detector + one FSM per algorithm, rebuilt on seek
    class Track:
        """One detector + FSM, plus the rolling traces drawn for it.

        Activity is plotted in multiples of t_on, so the trigger sits at a fixed
        1.0 no matter what units the algorithm works in. That keeps the y-axis
        static (which is what lets the figure blit) and makes algorithms with
        wildly different scales directly comparable.
        """

        def __init__(self, name):
            self.name = name
            self.reset()

        def reset(self):
            self.det = R.ALGOS[self.name][0].module.Detector(R.SAMPLE_HZ)
            self.fsm = R.Machine()
            self.act = deque([0.0] * win, maxlen=win)      # act / t_on
            self.off = deque([0.0] * win, maxlen=win)      # t_off / t_on
            self.intent = deque([0] * win, maxlen=win)

        def feed(self, i):
            """One sample in, thresholds and (every TICK_DIV) one FSM tick out."""
            b, a = B[i], A[i]
            v, on, off, full = self.det.update(b, a)
            if i % R.TICK_DIV == 0:
                self.fsm.step(v, on, off, full, i / R.SAMPLE_HZ * 1000,
                              b < R.DEAD_COUNTS)
            # during warm-up / brown-out a detector parks t_on at an unreachable
            # sentinel; normalising by it correctly collapses the trace to ~0
            scale = on if 1e-9 < on < 1e6 else None
            self.act.append(v / scale if scale else 0.0)
            self.off.append(off / scale if scale else 0.0)
            self.intent.append(self.fsm.intent)

        def snapshot(self):
            return copy.deepcopy((self.det, self.fsm, self.act, self.off, self.intent))

        def restore(self, s):
            self.det, self.fsm, self.act, self.off, self.intent = copy.deepcopy(s)

    tracks = [Track(n) for n in names]
    bufB = deque([0] * win, maxlen=win)
    bufR = deque([0] * win, maxlen=win)
    pos = {"i": 0, "playing": True, "speed": args.speed}
    snap_every = int(SNAP_S * R.SAMPLE_HZ)
    snaps = {}                       # sample index -> full state, for instant seek-back

    def advance(n):
        """Feed n more samples, snapshotting periodically so seeks stay cheap."""
        for _ in range(n):
            i = pos["i"]
            if i >= n_total:
                pos["playing"] = False
                return
            if i % snap_every == 0 and i not in snaps:
                snaps[i] = ([t.snapshot() for t in tracks], bufB.copy(), bufR.copy())
            for t in tracks:
                t.feed(i)
            bufB.append(B[i])
            bufR.append(recorded[i] if i < len(recorded) else 0)
            pos["i"] = i + 1

    def rebuild(target):
        """Detectors are causal, so seeking back means replaying forward again --
        but only from the nearest snapshot, not from zero."""
        target = max(0, min(target, n_total - 1))
        base = max((k for k in snaps if k <= target), default=None)
        if base is None:
            for t in tracks:
                t.reset()
            bufB.extend([0] * win); bufR.extend([0] * win)
            pos["i"] = 0
        else:
            states, sb, sr = snaps[base]
            for t, s in zip(tracks, states):
                t.restore(s)
            bufB.clear(); bufB.extend(sb)
            bufR.clear(); bufR.extend(sr)
            pos["i"] = base
        advance(target - pos["i"])

    # ---- figure: chB, recorded intent, then activity+intent per algorithm
    rows = 2 + 2 * len(tracks)
    fig, axes = plt.subplots(rows, 1, sharex=True,
                             figsize=(11, 1.5 * rows + 1),
                             gridspec_kw={"height_ratios": [2, 1] + [2, 1] * len(tracks)})
    fig.canvas.manager.set_window_title(os.path.basename(args.file))
    xs = [i / R.SAMPLE_HZ for i in range(win)][::dec]

    axes[0].set_ylim(0, 4200); axes[0].set_ylabel("chB env", fontsize=8)
    lineB, = axes[0].plot(xs, list(bufB)[::dec], color="#2a78d6", linewidth=0.8)
    axes[1].set_ylim(-1150, 1150); axes[1].set_ylabel("recorded", fontsize=8)
    axes[1].axhline(0, color="0.6", linewidth=0.8)
    lineR, = axes[1].plot(xs, list(bufR)[::dec], color="#898781", linewidth=1.0)

    art = []
    for k, t in enumerate(tracks):
        ax_a, ax_i = axes[2 + 2 * k], axes[3 + 2 * k]
        # t_on is fixed at 1.0 by the normalisation, so it lives in the static
        # background; only t_off can still move, so that one stays a live line.
        ax_a.axhline(1.0, color="#c0392b", linewidth=0.9, linestyle="--")
        la, = ax_a.plot(xs, list(t.act)[::dec], color="#eb6834", linewidth=1.1)
        lof, = ax_a.plot(xs, list(t.off)[::dec], color="#7f8c8d", linewidth=0.8,
                         linestyle=":")
        ax_a.set_ylim(0, Y_TOP)
        ax_a.set_yticks([0, 1, 2, 3])
        ax_a.set_ylabel(f"{t.name}\n(x t_on)", fontsize=8)
        li, = ax_i.plot(xs, list(t.intent)[::dec], color="#eb6834", linewidth=1.2)
        ax_i.set_ylim(-1150, 1150); ax_i.axhline(0, color="0.6", linewidth=0.8)
        ax_i.set_ylabel("intent", fontsize=8)
        badge = ax_i.text(0.008, 0.86, "", transform=ax_i.transAxes, fontsize=9,
                          va="top", family="monospace", weight="bold")
        art.append((t, la, lof, li, badge))

    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)
    axes[-1].set_xlabel("seconds on screen (newest at right)", fontsize=8)
    axes[0].set_title("fires above the dashed line at 1.0 · releases below the "
                      "dotted line", fontsize=9, loc="left", color="#52514e")
    clock = axes[0].text(0.008, 0.92, "", transform=axes[0].transAxes, fontsize=9,
                         va="top", family="monospace", color="#0b0b0b")
    fig.tight_layout()

    def on_key(ev):
        if ev.key == " ":
            pos["playing"] = not pos["playing"]
        elif ev.key == "up":
            pos["speed"] = min(64.0, pos["speed"] * 2)
        elif ev.key == "down":
            pos["speed"] = max(0.125, pos["speed"] / 2)
        elif ev.key == "right":
            advance(min(5 * R.SAMPLE_HZ, n_total - 1 - pos["i"]))
        elif ev.key == "left":
            rebuild(pos["i"] - 5 * R.SAMPLE_HZ)
        elif ev.key == "r":
            rebuild(0)
        elif ev.key == "q":
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_):
        if pos["playing"]:
            advance(max(1, int(R.SAMPLE_HZ * 0.033 * pos["speed"])))

        lineB.set_ydata(list(bufB)[::dec]); lineR.set_ydata(list(bufR)[::dec])
        drawn = [lineB, lineR, clock]
        for t, la, lof, li, badge in art:
            la.set_ydata(list(t.act)[::dec]); lof.set_ydata(list(t.off)[::dec])
            li.set_ydata(list(t.intent)[::dec])
            st = t.fsm.st
            badge.set_text(f"{t.name}  {R.NAMES[st]:8} {t.fsm.intent:+5d}")
            badge.set_color(STATE_COLORS[st])
            drawn += [la, lof, li, badge]
        clock.set_text(f"{pos['i'] / R.SAMPLE_HZ:7.2f} / {n_total / R.SAMPLE_HZ:.1f} s"
                       f"   x{pos['speed']:g}"
                       f"{'' if pos['playing'] else '  [PAUSED]'}")
        return drawn

    # blit=True is the difference between 2.7 fps and 70 fps here: without it
    # every frame redraws all axes, grids and ticks. It only works because the
    # y-limits are now static, so nothing may call set_ylim() per frame.
    _ani = FuncAnimation(fig, update, interval=33, blit=True,
                         cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()
