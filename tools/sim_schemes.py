"""Compare candidate EMG->grip control schemes on logged data, WITHOUT touching
any firmware. Pure replay + plots, for a design review.

All schemes share one front-end: envelope = |chB - rest| (chB is the analog
peak-detector channel; chA is dead). Calibration rest/MVC are estimated from
each file's own envelope distribution (20th / 99th percentile), thresholds are
the same fractions the firmware uses (T_low .12, T_high .28, full .80 of span).

To make schemes comparable we express every one as an estimated GRIP POSITION
over time (0 = fully open, 100% = fully closed), using a simple servo model:
at full command the grip travels open<->closed in FULL_CLOSE_S seconds.

Schemes:
  1 prop_latch_heavy  - CURRENT firmware logic: proportional-speed latching FSM
                        on a heavy ~200 ms EMA envelope. Baseline.
  2 prop_latch_light  - same FSM, but a light 10 ms median instead of the heavy
                        EMA (the "don't smooth the already-smooth signal" fix).
  3 ratchet           - "read peaks as steps": each flex event adds a fixed
                        grip increment; sustained relax releases. Discrete,
                        bounded, no hold effort.
  4 staircase         - one flex sets a discrete grip LEVEL by its peak height
                        (4 levels); sustained relax releases.
  5 bangbang          - Schmitt trigger, non-latching: flex closes, relax opens.
                        Simplest; shows the "must keep flexing to hold" contrast.

Usage:
    python sim_schemes.py            # real capture + synthetics -> PNGs + metrics.csv
    python sim_schemes.py FILE.csv
"""
import csv
import glob
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, "data")
OUT = os.path.join(ROOT, "output", "schemes")

# ---- shared front-end / calibration (mirrors the firmware fractions) ----
SAMPLE_HZ, TICK_DIV = 500, 10
TICK_S = TICK_DIV / SAMPLE_HZ            # 0.02 s (50 Hz)
EMA_ALPHA = 0.01                         # ~200 ms heavy EMA
MED_N = 5                                # ~10 ms light median
T_LOW_F, T_HIGH_F, FULL_F = 0.12, 0.28, 0.80
CAL_FLOOR = 40

# ---- FSM timings (firmware) ----
DEBOUNCE_MS, RELAX_MS = 80, 150
OPEN_DWELL_MS, OPEN_DRIVE_MS = 2000, 2500
OPEN_SPEED, CLOSE_MIN = 600, 250

# ---- servo / grip model ----
FULL_CLOSE_S = 1.5                       # open->closed time at full command
# ratchet / staircase params
STEP = 0.25                              # grip added per flex (ratchet)
REFRACTORY_MS = 400                      # min gap between counted flexes
N_LEVELS = 4                             # staircase discrete grip levels

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


def envelopes(chB):
    """Return (env_heavy, env_light) at full 500 Hz, both = deviation from rest."""
    base = sum(chB[:250]) / min(250, len(chB))
    rect = [abs(x - base) for x in chB]
    heavy, e = [], 0.0
    for r in rect:
        e += EMA_ALPHA * (r - e); heavy.append(e)
    light = []
    for i in range(len(rect)):
        w = rect[max(0, i - MED_N + 1):i + 1]
        light.append(sorted(w)[len(w) // 2])
    return heavy, light


def calib(env_light):
    rest = pct(env_light, 20); mvc = pct(env_light, 99); span = mvc - rest
    return {
        "rest": rest, "mvc": mvc, "cal": span > CAL_FLOOR,
        "t_low": rest + T_LOW_F * span, "t_high": rest + T_HIGH_F * span,
        "full": rest + FULL_F * span,
    }


def ticks(env_full):
    return env_full[::TICK_DIV]


# ---------- schemes ----------
# Every scheme emits an INTENT series (the -1000..+1000 servo speed command the
# firmware would actually send at 50 Hz). Grip POSITION is then the honest
# integral of that same intent through one shared servo model, so the intent
# panel and the position panel are always physically consistent.

def integrate(intents):
    """intent (servo speed cmd) -> grip position 0..1, via the servo model."""
    pos = 0.0; out = []
    for it in intents:
        pos = max(0.0, min(1.0, pos + (it / 1000.0) * (TICK_S / FULL_CLOSE_S)))
        out.append(pos)
    return out


def follow_target(target):
    """Position-style schemes (ratchet, staircase) name a TARGET grip; the servo
    can only be commanded a speed, so drive full-speed toward the target and
    stop when there. Returns the intent series that chases `target`."""
    pos = 0.0; intents = []
    step = TICK_S / FULL_CLOSE_S              # max position change per tick
    for tgt in target:
        err = tgt - pos
        it = 0 if abs(err) < step * 0.5 else (1000 if err > 0 else -1000)
        pos = max(0.0, min(1.0, pos + (it / 1000.0) * step))
        intents.append(it)
    return intents


def prop_latch(env, th):
    """Proportional latching FSM -> intent (servo speed). Used for heavy & light."""
    st = ST_OPEN; ah = bh = bl = 0; open_t = 0.0
    intents = []
    for i, e in enumerate(env):
        now = i * TICK_S * 1000
        intent = 0
        ah = ah + 20 if e > th["t_high"] else 0
        bh = bh + 20 if e < th["t_high"] else 0
        bl = bl + 20 if e < th["t_low"] else 0
        if st == ST_OPEN:
            if ah >= DEBOUNCE_MS: st = ST_CLOSING
        elif st == ST_CLOSING:
            frac = max(0.0, min(1.0, (e - th["t_high"]) / (th["full"] - th["t_high"])))
            intent = CLOSE_MIN + int(frac * (1000 - CLOSE_MIN))
            if bh >= RELAX_MS: st = ST_HOLDING
        elif st == ST_HOLDING:
            if ah >= DEBOUNCE_MS: st = ST_CLOSING
            elif bl >= OPEN_DWELL_MS: st = ST_OPENING; open_t = now
        elif st == ST_OPENING:
            intent = -OPEN_SPEED
            if ah >= DEBOUNCE_MS: st = ST_CLOSING
            elif now - open_t >= OPEN_DRIVE_MS: st = ST_OPEN
        intents.append(intent)
    return intents, []


def ratchet(env, th):
    """Each flex event bumps a target grip by STEP; servo chases it -> intent."""
    tgt = 0.0; refr = 0; armed = True; bl = 0
    target, events = [], []
    for i, e in enumerate(env):
        if refr > 0: refr -= 20
        bl = bl + 20 if e < th["t_low"] else 0
        if e < th["t_low"]:
            armed = True
        if armed and e > th["t_high"] and refr <= 0:      # rising flex event
            tgt = min(1.0, tgt + STEP); refr = REFRACTORY_MS; armed = False
            events.append(i)
        if bl >= OPEN_DWELL_MS:                            # sustained relax -> release
            tgt = 0.0
        target.append(tgt)
    return follow_target(target), events


def staircase(env, th):
    """One flex sets a discrete target LEVEL by its peak; servo chases it."""
    tgt = 0.0; peak = 0.0; bl = 0
    target = []
    for e in env:
        bl = bl + 20 if e < th["t_low"] else 0
        if e > th["t_low"]:
            peak = max(peak, e)
            frac = max(0.0, min(1.0, (peak - th["t_high"]) / (th["full"] - th["t_high"])))
            level = round(frac * (N_LEVELS - 1)) / (N_LEVELS - 1)
            tgt = max(tgt, level)                          # peak sets the level
        if bl >= OPEN_DWELL_MS:
            tgt = 0.0; peak = 0.0
        target.append(tgt)
    return follow_target(target), []


def bangbang(env, th):
    """Schmitt trigger, non-latching: flex -> full close, relax -> full open."""
    sign = 0; intents = []
    for e in env:
        if e > th["t_high"]: sign = 1        # flex -> close
        elif e < th["t_low"]: sign = -1      # relax -> open (no latch)
        intents.append(sign * 1000)
    return intents, []


SCHEMES = ["prop_latch_heavy", "prop_latch_light", "ratchet", "staircase", "bangbang"]
LABELS = {
    "prop_latch_heavy": "1. proportional latch  (heavy EMA — current)",
    "prop_latch_light": "2. proportional latch  (light median — fix)",
    "ratchet":          "3. ratchet  (peaks = steps)",
    "staircase":        "4. staircase  (peak = discrete level)",
    "bangbang":         "5. bang-bang  (flex close / relax open)",
}


def run_schemes(env_heavy_t, env_light_t, th):
    raw = {
        "prop_latch_heavy": prop_latch(env_heavy_t, th),
        "prop_latch_light": prop_latch(env_light_t, th),
        "ratchet":          ratchet(env_light_t, th),
        "staircase":        staircase(env_light_t, th),
        "bangbang":         bangbang(env_light_t, th),
    }
    # attach the integrated grip position for each -> (intents, positions, events)
    return {k: (intents, integrate(intents), events)
            for k, (intents, events) in raw.items()}


def metrics(name, scheme, intents, pos, extra):
    n = len(pos); dur = n * TICK_S
    peak = max(pos); final = pos[-1]
    gripped = sum(1 for p in pos if p > 0.5) / n * 100
    # first time crossing 50% closed
    t50 = next((i * TICK_S for i, p in enumerate(pos) if p >= 0.5), None)
    # count close "events": rises through 50%
    closes = sum(1 for i in range(1, n) if pos[i - 1] < 0.5 <= pos[i])
    opens = final < 0.10
    # flex_events is only meaningful for the ratchet (extra = list of event ticks)
    flex = len(extra) if scheme == "ratchet" else ""
    return {
        "file": name, "scheme": scheme,
        "peak_grip_%": round(peak * 100),
        "final_grip_%": round(final * 100),
        "%time_gripped": round(gripped),
        "time_to_50%_s": round(t50, 2) if t50 is not None else "",
        "close_events": closes,
        "flex_events": flex,
        "ends_open": "yes" if opens else "no",
    }


def plot(name, t, chB, env_light_t, th, results, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    SURF, INK, INK2, MUTE = "#fcfcfb", "#0b0b0b", "#52514e", "#898781"
    GRID, BASE, BLUE, ORANGE = "#e1e0d9", "#c3c2b7", "#2a78d6", "#eb6834"
    tt = [i * TICK_S for i in range(len(env_light_t))]

    def style(ax):
        ax.set_facecolor(SURF); ax.grid(True, color=GRID, linewidth=0.8)
        for s in ax.spines.values(): s.set_color(BASE)
        ax.tick_params(colors=MUTE, labelsize=8)

    fig = plt.figure(figsize=(13.5, 13))
    fig.patch.set_facecolor(SURF)
    gs = fig.add_gridspec(len(SCHEMES) + 1, 2, height_ratios=[1.5] + [1] * len(SCHEMES),
                          hspace=0.30, wspace=0.14)

    # envelope spans both columns
    ax0 = fig.add_subplot(gs[0, :]); style(ax0)
    ax0.plot(t, chB, color=BLUE, linewidth=0.6, label="chB (analog envelope)")
    ax0.plot(tt, env_light_t, color=INK, linewidth=1.2, label="digital envelope")
    for lvl, lab in [(th["t_low"], "T_low"), (th["t_high"], "T_high"), (th["full"], "full")]:
        ax0.axhline(lvl, color=MUTE, linewidth=0.9, linestyle=(0, (4, 3)))
        ax0.text(t[-1], lvl, f" {lab}", color=INK2, fontsize=7, va="center")
    ax0.set_ylabel("counts", color=INK2, fontsize=8)
    ax0.set_title(f"{name}  —  intent (left) vs resulting grip position (right)",
                  color=INK, fontsize=12, loc="left")
    ax0.legend(loc="upper right", fontsize=7.5, framealpha=0.9, facecolor=SURF, edgecolor=GRID)

    for row, sch in enumerate(SCHEMES):
        intents, pos, events = results[sch]
        axi = fig.add_subplot(gs[row + 1, 0], sharex=ax0); style(axi)   # intent
        axp = fig.add_subplot(gs[row + 1, 1], sharex=ax0); style(axp)   # position

        axi.axhline(0, color=BASE, linewidth=0.9)
        axi.plot(tt, intents, color=ORANGE, linewidth=1.2)
        axi.fill_between(tt, intents, 0, color=ORANGE, alpha=0.12)
        axi.set_ylim(-1150, 1150); axi.set_yticks([-1000, 0, 1000])
        axi.set_ylabel("intent", color=INK2, fontsize=8)
        if sch == "ratchet":
            for ev in events:
                axi.axvline(ev * TICK_S, color=MUTE, linewidth=0.6, alpha=0.5)
        axi.text(0.012, 0.90, LABELS[sch], transform=axi.transAxes, color=INK,
                 fontsize=8.5, va="top",
                 bbox=dict(boxstyle="round,pad=0.25", fc=SURF, ec=GRID, lw=0.8))

        ppos = [p * 100 for p in pos]
        axp.plot(tt, ppos, color=INK, linewidth=1.3)
        axp.fill_between(tt, ppos, 0, color=BLUE, alpha=0.14)
        axp.set_ylim(-5, 105); axp.set_yticks([0, 50, 100])
        axp.set_ylabel("grip %", color=INK2, fontsize=8)

        if row == 0:
            axi.set_title("intent  (servo speed cmd, −1000…+1000)", color=INK2,
                          fontsize=9, loc="left")
            axp.set_title("grip position  (0 = open, 100 = closed)", color=INK2,
                          fontsize=9, loc="left")
        if row < len(SCHEMES) - 1:
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
        files = [os.path.join(DATA, "emg_2026-07-21_140447.csv")] + \
                sorted(glob.glob(os.path.join(DATA, "emg_synth_*.csv")))

    outdir = OUT
    os.makedirs(outdir, exist_ok=True)
    rows = []
    for f in files:
        name = os.path.basename(f)
        t, chB = load(f)
        eh, el = envelopes(chB)
        th = calib(el)
        if not th["cal"]:
            print(f"skip {name}: not calibratable (span<{CAL_FLOOR})"); continue
        results = run_schemes(ticks(eh), ticks(el), th)
        png = os.path.join(outdir, name.replace(".csv", "_schemes.png"))
        plot(name, t, chB, ticks(el), th, results, png)
        for sch in SCHEMES:
            intents, pos, events = results[sch]
            rows.append(metrics(name, sch, intents, pos, events))
        print(f"{name}: plotted -> {os.path.relpath(png)}")

    csvp = os.path.join(outdir, "metrics.csv")
    with open(csvp, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print(f"\nmetrics -> {os.path.relpath(csvp)}")
    # pretty print
    hdr = list(rows[0].keys())
    widths = {h: max(len(h), *(len(str(r[h])) for r in rows)) for h in hdr}
    print("  ".join(h.ljust(widths[h]) for h in hdr))
    for r in rows:
        print("  ".join(str(r[h]).ljust(widths[h]) for h in hdr))


if __name__ == "__main__":
    main()
