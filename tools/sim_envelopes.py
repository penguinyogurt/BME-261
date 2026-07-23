"""Compare ENVELOPE front-ends feeding the SAME proportional-latch control.

The control scheme is fixed (proportional latching FSM). What changes is how we
turn chB into the envelope the FSM keys off:

  A. ema        - symmetric ~200 ms EMA (CURRENT firmware). Lags and rounds off
                  the sharp onset of each contraction -> loses the "beginning
                  peaks", so brief/fast flexes make weak intent.
  B. follower   - asymmetric envelope follower: FAST attack, SLOW release. Snaps
                  up to catch onsets, decays slowly. Preserves sensitivity.
  C. staircase  - the follower quantized into discrete steps (with hysteresis):
                  "treat each peak as an increase/decrease" of the envelope.
                  Same onset capture as B, but a clean stepped target.

Everything is done in NORMALIZED units (0 = rest, 1 = MVC, per each envelope's
own 20th/99th percentile) so the FSM sees the same fractional thresholds and the
only difference is envelope SHAPE. Emits intent + grip position for each.

Firmware is NOT touched. Usage:
    python sim_envelopes.py [FILE.csv]      # default: real capture + 2 synthetics
"""
import csv
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, "data")
OUT = os.path.join(ROOT, "output", "envelopes")

SAMPLE_HZ, TICK_DIV = 500, 10
TICK_S = TICK_DIV / SAMPLE_HZ
EMA_ALPHA = 0.01                 # symmetric ~200 ms
ATT, REL = 0.6, 0.008            # follower: ~3 ms attack, ~250 ms release
N_LEVELS = 8                     # staircase steps
HYST = 0.6                       # step change needs 0.6*step of movement
MED_N = 3                        # tiny spike reject before the follower

# proportional-latch FSM (normalized thresholds) + servo model
T_LOW, T_HIGH, FULL = 0.12, 0.28, 0.80
DEBOUNCE_MS, RELAX_MS = 80, 150
OPEN_DWELL_MS, OPEN_DRIVE_MS = 2000, 2500
OPEN_SPEED, CLOSE_MIN = 600, 250
FULL_CLOSE_S = 1.5
ST_OPEN, ST_CLOSING, ST_HOLDING, ST_OPENING = range(4)


def load(path):
    t, b = [], []
    with open(path) as fh:
        r = csv.reader(fh); next(r, None)
        for row in r:
            if len(row) != 3:
                continue
            try:
                t.append(float(row[0])); b.append(int(row[2]))
            except ValueError:
                continue
    return [x - t[0] for x in t], b


def pct(xs, p):
    s = sorted(xs); k = (len(s) - 1) * p / 100.0
    lo = int(k); hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def rectify(chB):
    base = sum(chB[:250]) / min(250, len(chB))
    out, hist = [], []
    for x in chB:
        hist.append(max(0.0, x - base))
        if len(hist) > MED_N: hist.pop(0)
        out.append(sorted(hist)[len(hist) // 2])   # median-3 spike reject
    return out


def env_ema(r):
    e, out = 0.0, []
    for x in r:
        e += EMA_ALPHA * (x - e); out.append(e)
    return out


def env_follower(r):
    e, out = 0.0, []
    for x in r:
        e += (ATT if x > e else REL) * (x - e)      # fast up, slow down
        out.append(e)
    return out


def env_staircase(r):
    """Follower, then snap to discrete levels with hysteresis -> a staircase."""
    cont = env_follower(r)
    p20, p99 = pct(cont, 20), pct(cont, 99)
    span = max(1e-6, p99 - p20)
    step = 1.0 / N_LEVELS
    level = 0.0; out = []
    for c in cont:
        n = (c - p20) / span                        # normalized follower
        target = round(n * N_LEVELS) / N_LEVELS
        if abs(target - level) >= step * HYST:      # hysteresis: resist chatter
            level = target
        out.append(p20 + level * span)              # back to raw units for calib
    return out


def normalize(env):
    p20, p99 = pct(env, 20), pct(env, 99)
    span = max(1e-6, p99 - p20)
    return [(e - p20) / span for e in env], (p99 - p20)


def prop_latch(env_n):
    """Proportional latch on a normalized envelope. Returns intents + #closes."""
    st = ST_OPEN; ah = bh = bl = 0; open_t = 0.0
    intents, closes = [], 0
    for i, e in enumerate(env_n):
        now = i * TICK_S * 1000
        intent = 0
        ah = ah + 20 if e > T_HIGH else 0
        bh = bh + 20 if e < T_HIGH else 0
        bl = bl + 20 if e < T_LOW else 0
        if st == ST_OPEN:
            if ah >= DEBOUNCE_MS: st = ST_CLOSING; closes += 1
        elif st == ST_CLOSING:
            frac = max(0.0, min(1.0, (e - T_HIGH) / (FULL - T_HIGH)))
            intent = CLOSE_MIN + int(frac * (1000 - CLOSE_MIN))
            if bh >= RELAX_MS: st = ST_HOLDING
        elif st == ST_HOLDING:
            if ah >= DEBOUNCE_MS: st = ST_CLOSING; closes += 1
            elif bl >= OPEN_DWELL_MS: st = ST_OPENING; open_t = now
        elif st == ST_OPENING:
            intent = -OPEN_SPEED
            if ah >= DEBOUNCE_MS: st = ST_CLOSING; closes += 1
            elif now - open_t >= OPEN_DRIVE_MS: st = ST_OPEN
        intents.append(intent)
    return intents, closes


def integrate(intents):
    pos, out = 0.0, []
    for it in intents:
        pos = max(0.0, min(1.0, pos + (it / 1000.0) * (TICK_S / FULL_CLOSE_S)))
        out.append(pos)
    return out


VARIANTS = [
    ("ema",       "A. EMA  (symmetric ~200 ms — current)"),
    ("follower",  "B. follower  (fast attack / slow release)"),
    ("staircase", "C. staircase  (peaks stepped up/down)"),
]
BUILDERS = {"ema": env_ema, "follower": env_follower, "staircase": env_staircase}


def run(path):
    t, chB = load(path)
    r = rectify(chB)
    out = {}
    for key, _ in VARIANTS:
        env = BUILDERS[key](r)
        env_n, span = normalize(env)
        env_t = env_n[::TICK_DIV]
        intents, closes = prop_latch(env_t)
        out[key] = {"env_n": env_t, "intents": intents,
                    "pos": integrate(intents), "closes": closes}
    return t, chB, out


def plot(name, t, results, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    SURF, INK, INK2, MUTE = "#fcfcfb", "#0b0b0b", "#52514e", "#898781"
    GRID, BASE, BLUE, ORANGE, AQUA = "#e1e0d9", "#c3c2b7", "#2a78d6", "#eb6834", "#1baf7a"
    ENVC = {"ema": MUTE, "follower": ORANGE, "staircase": AQUA}
    n_t = len(results["ema"]["env_n"])
    tt = [i * TICK_S for i in range(n_t)]

    def style(ax):
        ax.set_facecolor(SURF); ax.grid(True, color=GRID, linewidth=0.8)
        for s in ax.spines.values(): s.set_color(BASE)
        ax.tick_params(colors=MUTE, labelsize=8)

    fig = plt.figure(figsize=(13.5, 9.5)); fig.patch.set_facecolor(SURF)
    gs = fig.add_gridspec(len(VARIANTS) + 1, 2, height_ratios=[1.7] + [1] * len(VARIANTS),
                          hspace=0.30, wspace=0.14)

    # top: the three normalized envelopes overlaid -> shows onset capture
    ax0 = fig.add_subplot(gs[0, :]); style(ax0)
    for key, lab in VARIANTS:
        ax0.plot(tt, results[key]["env_n"], color=ENVC[key], linewidth=1.2, label=lab)
    for lvl, nm in [(T_LOW, "T_low"), (T_HIGH, "T_high"), (FULL, "full")]:
        ax0.axhline(lvl, color=MUTE, linewidth=0.9, linestyle=(0, (4, 3)))
        ax0.text(tt[-1], lvl, f" {nm}", color=INK2, fontsize=7, va="center")
    ax0.set_ylim(-0.1, 1.35); ax0.set_ylabel("envelope (norm.)", color=INK2, fontsize=8)
    ax0.set_title(f"{name}  —  envelope front-ends into the SAME proportional latch",
                  color=INK, fontsize=12, loc="left")
    ax0.legend(loc="upper right", fontsize=7.5, framealpha=0.9, facecolor=SURF, edgecolor=GRID)

    for row, (key, lab) in enumerate(VARIANTS):
        d = results[key]
        axi = fig.add_subplot(gs[row + 1, 0], sharex=ax0); style(axi)
        axp = fig.add_subplot(gs[row + 1, 1], sharex=ax0); style(axp)

        axi.axhline(0, color=BASE, linewidth=0.9)
        axi.plot(tt, d["intents"], color=ENVC[key], linewidth=1.2)
        axi.fill_between(tt, d["intents"], 0, color=ENVC[key], alpha=0.12)
        axi.set_ylim(-1150, 1150); axi.set_yticks([-1000, 0, 1000])
        axi.set_ylabel("intent", color=INK2, fontsize=8)
        axi.text(0.012, 0.90, lab, transform=axi.transAxes, color=INK, fontsize=8.5,
                 va="top", bbox=dict(boxstyle="round,pad=0.25", fc=SURF, ec=GRID, lw=0.8))

        ppos = [p * 100 for p in d["pos"]]
        axp.plot(tt, ppos, color=INK, linewidth=1.3)
        axp.fill_between(tt, ppos, 0, color=BLUE, alpha=0.14)
        axp.set_ylim(-5, 105); axp.set_yticks([0, 50, 100])
        axp.set_ylabel("grip %", color=INK2, fontsize=8)

        if row == 0:
            axi.set_title("intent  (servo speed cmd)", color=INK2, fontsize=9, loc="left")
            axp.set_title("grip position", color=INK2, fontsize=9, loc="left")
        if row < len(VARIANTS) - 1:
            plt.setp(axi.get_xticklabels(), visible=False)
            plt.setp(axp.get_xticklabels(), visible=False)
        else:
            axi.set_xlabel("time (s)", color=INK2, fontsize=8)
            axp.set_xlabel("time (s)", color=INK2, fontsize=8)

    fig.savefig(path, dpi=130, facecolor=SURF, bbox_inches="tight")
    plt.close(fig)


def main():
    if len(sys.argv) > 1:
        files = [sys.argv[1]]
    else:
        files = [os.path.join(DATA, "emg_2026-07-21_140447.csv"),
                 os.path.join(DATA, "emg_synth_rapid_open_close_reps.csv"),
                 os.path.join(DATA, "emg_synth_single_close.csv")]
    outdir = OUT
    os.makedirs(outdir, exist_ok=True)
    print(f"{'file':40} {'variant':10} {'peak%':>5} {'final%':>6} {'closes':>6}")
    for f in files:
        name = os.path.basename(f)
        t, chB, results = run(f)
        png = os.path.join(outdir, name.replace(".csv", "_env.png"))
        plot(name, t, results, png)
        for key, _ in VARIANTS:
            d = results[key]
            print(f"{name:40} {key:10} {round(max(d['pos'])*100):>5} "
                  f"{round(d['pos'][-1]*100):>6} {d['closes']:>6}")
        print(f"  -> {os.path.relpath(png)}")


if __name__ == "__main__":
    main()
