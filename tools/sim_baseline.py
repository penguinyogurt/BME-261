"""Does an adaptive baseline fix the drift-induced perma-close? Replay only.

The firmware today does:      env = EMA(|x - baseSrc|)   with baseSrc a one-shot tare
The proposed change:          baseSrc tracks the resting floor continuously, and
                              a unipolar channel clamps at 0 instead of rectifying.

The test that matters is not "does it look nice on clean data" - it is whether it
RECOVERS when the baseline is wrong, since that is the observed failure. So every
variant is run twice: once tared correctly, once tared 400 counts off (the drift
measured between two self-tests in debug_20260723_182511.log).

    python tools/sim_baseline.py [data/xxx.csv]
"""
import csv
import os
import sys

SAMPLE_HZ, TICK_DIV = 500, 10
TICK_S = TICK_DIV / SAMPLE_HZ
ENV_ALPHA = 0.01
T_LOW_F, T_HIGH_F, FULL_F = 0.12, 0.28, 0.80
DEBOUNCE_MS, RELAX_MS = 80, 150
OPEN_DWELL_MS, OPEN_DRIVE_MS = 2000, 2500
OPEN_SPEED, CLOSE_MIN = 600, 250
ST_OPEN, ST_CLOSING, ST_HOLDING, ST_OPENING = range(4)
NAMES = ["OPEN", "CLOSING", "HOLDING", "OPENING"]

DRIFT_TOTAL = 800       # counts of DC drift to ramp across the session


def load(path):
    B = []
    with open(path) as f:
        r = csv.reader(f); next(r, None)
        for row in r:
            if len(row) >= 3:          # newer logs carry env,intent too
                try: B.append(int(row[2]))
                except ValueError: pass
    return B


def pct(v, p):
    s = sorted(v); k = (len(s) - 1) * p / 100.0
    lo = int(k); hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def envelope(B, base0, adaptive, a_up=0.0002, a_dn=0.02):
    """Return the 500 Hz envelope for one variant."""
    base, env, out = float(base0), 0.0, []
    for x in B:
        if adaptive:
            base += (a_dn if x < base else a_up) * (x - base)
            r = max(0.0, x - base)          # unipolar: below the floor = no activity
        else:
            r = abs(x - base)               # current firmware
        env += ENV_ALPHA * (r - env)
        out.append(env)
    return out


def fsm(env_t, th):
    st = ST_OPEN; ah = bh = bl = 0; open_t = 0.0
    intents, states = [], []
    for i, e in enumerate(env_t):
        now = i * TICK_S * 1000
        intent = 0
        ah = ah + 20 if e > th[1] else 0
        bh = bh + 20 if e < th[1] else 0
        bl = bl + 20 if e < th[0] else 0
        if st == ST_OPEN:
            if ah >= DEBOUNCE_MS: st = ST_CLOSING
        elif st == ST_CLOSING:
            frac = max(0.0, min(1.0, (e - th[1]) / (th[2] - th[1])))
            intent = CLOSE_MIN + int(frac * (1000 - CLOSE_MIN))
            if bh >= RELAX_MS: st = ST_HOLDING
        elif st == ST_HOLDING:
            if ah >= DEBOUNCE_MS: st = ST_CLOSING
            elif bl >= OPEN_DWELL_MS: st = ST_OPENING; open_t = now
        elif st == ST_OPENING:
            intent = -OPEN_SPEED
            if ah >= DEBOUNCE_MS: st = ST_CLOSING
            elif now - open_t >= OPEN_DRIVE_MS: st = ST_OPEN
        intents.append(intent); states.append(st)
    return intents, states


CAL_S = 15.0            # thresholds are fixed by a trial early in the session


def run(B, label, base0, adaptive, a_up=0.0002):
    """Calibrate from the FIRST CAL_S seconds (as the firmware's trial does),
    then keep those thresholds for the whole session while the DC drifts."""
    env = envelope(B, base0, adaptive, a_up=a_up)
    env_t = env[::TICK_DIV]
    ncal = int(CAL_S / TICK_S)
    cal = env_t[:ncal]
    rest, mvc = pct(cal, 20), pct(cal, 99)
    th = (rest + T_LOW_F * (mvc - rest),
          rest + T_HIGH_F * (mvc - rest),
          rest + FULL_F * (mvc - rest))
    intents, states = fsm(env_t, th)
    n = len(states)
    late = env_t[ncal:]                       # after the trial, where drift bites
    floor = pct(late, 10) if late else 0
    pinned = sum(1 for i in intents[ncal:] if i == 1000) / max(1, len(late)) * 100
    stuck = sum(1 for s in states[ncal:] if s == ST_CLOSING) / max(1, len(late)) * 100
    ok = floor < th[1]
    print(f"{label:32} T_high={th[1]:6.0f}  late floor(p10)={floor:6.0f}  "
          f"{'relaxes ok' if ok else '** NEVER RELAXES **':19}  "
          f"CLOSING={stuck:5.1f}%  intent==1000:{pinned:5.1f}%")
    return {"label": label, "env": env_t, "states": states, "th": th,
            "floor": floor, "closing": stuck, "ncal": ncal}


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        root, "data", "emg_2026-07-23_171239.csv")
    B = load(path)
    good = sum(B[:250]) / 250
    n = len(B)
    # emulate the DC drift measured on hardware (~394 counts between two
    # self-tests 12 s apart) as a linear ramp across the recording
    drift = [B[i] + DRIFT_TOTAL * i / n for i in range(n)]
    print(f"{os.path.basename(path)}: {n} samples, rest baseline ~{good:.0f}")
    print(f"thresholds fixed by a trial over the first {CAL_S:.0f}s, then held.")
    print(f"'+drift' rows add a {DRIFT_TOTAL}-count DC ramp across the session.\n")

    print("--- current firmware: fixed baseline + fabs() ---")
    r1 = run(B,     "fixed, no drift", good, False)
    r2 = run(drift, "fixed, +drift", good, False)
    print("\n--- proposed: adaptive baseline + max(0,..) ---")
    r3 = run(B,     "adaptive, no drift", good, True)
    r4 = run(drift, "adaptive, +drift", good, True)
    run(drift, "adaptive(faster up), +drift", good, True, a_up=0.0005)

    out = os.path.join(root, "output", "sim_baseline.png")
    plot([r1, r2, r3, r4], out)
    print(f"\nwrote {os.path.relpath(out, root)}")


def plot(runs, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    SURF, INK, INK2, MUTE = "#fcfcfb", "#0b0b0b", "#52514e", "#898781"
    GRID, BASE, ORANGE = "#e1e0d9", "#c3c2b7", "#eb6834"

    fig, axes = plt.subplots(len(runs), 1, figsize=(12, 9), sharex=True)
    fig.patch.set_facecolor(SURF)
    for ax, r in zip(axes, runs):
        ax.set_facecolor(SURF); ax.grid(True, color=GRID, linewidth=0.8)
        for s in ax.spines.values(): s.set_color(BASE)
        ax.tick_params(colors=MUTE, labelsize=8)
        t = [i * TICK_S for i in range(len(r["env"]))]

        # shade every stretch the FSM spent driving the grip closed
        st = r["states"]; i = 0
        while i < len(st):
            j = i
            while j + 1 < len(st) and (st[j + 1] == ST_CLOSING) == (st[i] == ST_CLOSING):
                j += 1
            if st[i] == ST_CLOSING:
                ax.axvspan(t[i], t[j], color=ORANGE, alpha=0.16, linewidth=0)
            i = j + 1

        ax.plot(t, r["env"], color=INK, linewidth=1.0)
        ax.axhline(r["th"][1], color=MUTE, linewidth=1.0, linestyle=(0, (4, 3)))
        ax.annotate("T_high", (0.997, r["th"][1]), xycoords=("axes fraction", "data"),
                    color=INK2, fontsize=7, va="bottom", ha="right")
        ax.axvline(r["ncal"] * TICK_S, color=BASE, linewidth=1.0)
        ax.set_ylabel("envelope", color=INK2, fontsize=8)
        ax.text(0.008, 0.92,
                f"{r['label']}  —  {r['closing']:.0f}% of session CLOSING "
                f"(resting floor {r['floor']:.0f} vs T_high {r['th'][1]:.0f})",
                transform=ax.transAxes, color=INK, fontsize=9, va="top",
                bbox=dict(boxstyle="round,pad=0.28", fc=SURF, ec=GRID, lw=0.8))
    axes[0].set_title("Fixed vs adaptive baseline under DC drift "
                      "(orange = grip being driven closed; line = end of calibration)",
                      color=INK, fontsize=11, loc="left")
    axes[-1].set_xlabel("time (s)", color=INK2, fontsize=8)
    fig.savefig(path, dpi=130, facecolor=SURF, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
