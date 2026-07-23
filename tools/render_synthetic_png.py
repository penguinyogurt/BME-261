"""Render the synthetic EMG scenario CSVs to PNG graphs.

One PNG per scenario (two stacked panels sharing the time axis) plus a single
overview contact sheet of all envelopes. Design notes:

- chA and chB live on different count scales (~500 vs ~3000), so they get
  SEPARATE panels, never a dual-axis overlay.
- chB_env is the meaningful channel (blue, prominent, taller panel). Spans where
  the muscle is active (envelope above a rest threshold) are shaded so gesture
  timing reads at a glance.
- chA_raw is the dead channel, drawn thin in muted ink to signal "noise only."

Run:  python render_synthetic_png.py   (after make_synthetic_emg.py)
Writes png/emg_synth_<scenario>.png and png/overview_all_scenarios.png.
"""
import csv
import glob
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HERE = os.path.join(ROOT, "data")          # CSVs live in data/
PNG_DIR = os.path.join(ROOT, "output", "synthetic")

# --- validated palette (light mode) ---
SURFACE = "#fcfcfb"
PLANE = "#f9f9f7"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
SIGNAL = "#2a78d6"   # series-1 blue: the meaningful envelope
DEAD = "#a3a29c"     # muted: dead raw channel

CHB_REST = 3051
ACTIVE_THRESHOLD = 3200   # envelope above this ~= muscle engaged

# human-readable titles, in a sensible reading order
TITLES = [
    ("single_close",          "Single close  ·  contract, hold, release"),
    ("open_close",            "Open, then close"),
    ("close_hold_open",       "Close  →  hold  →  open"),
    ("rapid_open_close_reps", "Rapid open / close reps"),
    ("sustained_grip_fatigue","Sustained grip with fatigue decline"),
    ("graded_ramp",           "Graded ramp up / down"),
]


def load(name):
    t, a, b = [], [], []
    with open(os.path.join(HERE, f"emg_synth_{name}.csv")) as f:
        r = csv.reader(f)
        next(r)
        for row in r:
            t.append(float(row[0])); a.append(int(row[1])); b.append(int(row[2]))
    t0 = t[0]
    return [x - t0 for x in t], a, b


def active_spans(t, b, thr=ACTIVE_THRESHOLD):
    """Contiguous [start, end] time spans where envelope exceeds threshold."""
    spans, start = [], None
    for i, v in enumerate(b):
        if v > thr and start is None:
            start = t[i]
        elif v <= thr and start is not None:
            spans.append((start, t[i])); start = None
    if start is not None:
        spans.append((start, t[-1]))
    return [(s, e) for s, e in spans if e - s > 0.05]   # drop noise blips


def style_axes(ax):
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("bottom", "left"):
        ax.spines[side].set_color(AXIS)
        ax.spines[side].set_linewidth(0.8)
    ax.tick_params(colors=MUTED, labelsize=8, length=3)
    ax.grid(axis="y", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def render_one(name, title):
    t, a, b = load(name)
    fig, (ax_env, ax_raw) = plt.subplots(
        2, 1, figsize=(9, 4.6), sharex=True,
        gridspec_kw={"height_ratios": [2.0, 1.0], "hspace": 0.18},
    )
    fig.patch.set_facecolor(PLANE)

    # --- top: envelope (the signal) ---
    for s, e in active_spans(t, b):
        ax_env.axvspan(s, e, color=SIGNAL, alpha=0.07, linewidth=0)
    ax_env.axhline(CHB_REST, color=MUTED, linewidth=0.8, linestyle=(0, (4, 3)))
    ax_env.plot(t, b, color=SIGNAL, linewidth=1.3)
    style_axes(ax_env)
    ax_env.set_ylabel("chB envelope\n(ADC counts)", color=INK2, fontsize=8.5)
    ax_env.set_title(title, color=INK, fontsize=12, fontweight="bold",
                     loc="left", pad=8)
    ax_env.margins(x=0.01)

    # --- bottom: dead raw channel (noise only) ---
    ax_raw.plot(t, a, color=DEAD, linewidth=0.6)
    style_axes(ax_raw)
    ax_raw.set_ylabel("chA raw\n(dead — noise)", color=MUTED, fontsize=8.5)
    ax_raw.set_xlabel("time (s)", color=INK2, fontsize=9)

    fig.text(0.99, 0.015, "synthetic · shaded = muscle active",
             ha="right", va="bottom", color=MUTED, fontsize=7.5)
    fig.tight_layout(rect=(0, 0.02, 1, 1))
    out = os.path.join(PNG_DIR, f"emg_synth_{name}.png")
    fig.savefig(out, dpi=150, facecolor=PLANE)
    plt.close(fig)
    return out


def render_overview():
    fig, axes = plt.subplots(3, 2, figsize=(11, 8.2))
    fig.patch.set_facecolor(PLANE)
    fig.suptitle("Synthetic EMG scenarios — chB envelope",
                 color=INK, fontsize=14, fontweight="bold", x=0.02, ha="left")
    for ax, (name, title) in zip(axes.flat, TITLES):
        t, a, b = load(name)
        for s, e in active_spans(t, b):
            ax.axvspan(s, e, color=SIGNAL, alpha=0.07, linewidth=0)
        ax.axhline(CHB_REST, color=MUTED, linewidth=0.7, linestyle=(0, (4, 3)))
        ax.plot(t, b, color=SIGNAL, linewidth=1.1)
        style_axes(ax)
        ax.set_title(title, color=INK2, fontsize=10, loc="left", pad=4)
        ax.set_ylim(2900, 4150)
        ax.margins(x=0.01)
    for ax in axes[-1]:
        ax.set_xlabel("time (s)", color=INK2, fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    out = os.path.join(PNG_DIR, "overview_all_scenarios.png")
    fig.savefig(out, dpi=150, facecolor=PLANE)
    plt.close(fig)
    return out


if __name__ == "__main__":
    os.makedirs(PNG_DIR, exist_ok=True)
    for name, title in TITLES:
        print("wrote", os.path.relpath(render_one(name, title), HERE))
    print("wrote", os.path.relpath(render_overview(), HERE))
