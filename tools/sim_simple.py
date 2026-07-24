"""A deliberately simple alternative to the adaptive-baseline envelope.

Observation from data/emg_2026-07-23_193836.csv: chB's resting floor moved only
83 counts over 95 s (1772..1855). It does not need continuous tracking. And chB
is ALREADY a peak-detector envelope, so it does not need a second envelope on
top of it either.

So: measure the floor ONCE, set two absolute thresholds above it, and hysteresis
between them. No adaptive baseline, no settling phase, no MVC trial.

    floor  = median chB over the first FLOOR_S seconds (relaxed)
    T_ON   = floor + ON_OFF     -> start closing, speed scales up to FULL_OFF
    T_OFF  = floor + OFF_OFF    -> release to HOLDING (hysteresis)

    python tools/sim_simple.py            # every capture in data/
    python tools/sim_simple.py FILE.csv
"""
import csv
import glob
import os
import sys

SAMPLE_HZ, TICK_DIV = 500, 10
TICK_S = TICK_DIV / SAMPLE_HZ
SMOOTH_A = 0.04                 # ~50 ms; chB is already an envelope, just de-jitter

FLOOR_S = 2.0                   # how much relaxed signal to measure the floor from
ON_OFF, OFF_OFF, FULL_OFF = 750, 350, 1750    # thresholds as offsets above floor
DEAD_MIN = 400                  # chB below floor-this = front-end unpowered

DEBOUNCE_MS, RELAX_MS = 80, 150
OPEN_DWELL_MS, OPEN_DRIVE_MS = 2000, 2500
OPEN_SPEED, CLOSE_MIN = 600, 250
ST_OPEN, ST_CLOSING, ST_HOLDING, ST_OPENING = range(4)
NAMES = ["OPEN", "CLOSING", "HOLDING", "OPENING"]


def load(path):
    B, I = [], []
    for row in list(csv.reader(open(path)))[1:]:
        if len(row) < 3:
            continue
        try:
            B.append(int(row[2]))
            I.append(int(row[4]) if len(row) >= 5 else None)
        except ValueError:
            pass
    return B, I


def med(v):
    s = sorted(v); return s[len(s) // 2]


def run(path):
    B, firmware_intent = load(path)
    live = [b for b in B if b > 0]
    if len(live) < SAMPLE_HZ * 5:
        print(f"{os.path.basename(path):34} (too little powered data, skipped)")
        return None
    # Floor = a low percentile of the powered signal, not the median of the first
    # 2 s: starting mid-contraction (or saturated) puts the naive version at 4095
    # and no threshold is reachable. p10 tolerates activity during measurement.
    s = sorted(live)
    floor = s[int(len(s) * 0.10)]
    t_on, t_off, full = floor + ON_OFF, floor + OFF_OFF, floor + FULL_OFF

    sig = 0.0
    st = ST_OPEN; ah = bl = bo = 0; open_t = 0.0
    intents, states = [], []
    for i, x in enumerate(B):
        sig += SMOOTH_A * (x - sig)
        if i % TICK_DIV:
            continue
        now = i / SAMPLE_HZ * 1000
        intent = 0
        if x < floor - DEAD_MIN:            # unpowered / disconnected -> safe
            st = ST_OPEN; ah = bl = bo = 0
            intents.append(0); states.append(st); continue
        ah = ah + 20 if sig > t_on else 0
        bl = bl + 20 if sig < t_off else 0
        if st == ST_OPEN:
            if ah >= DEBOUNCE_MS: st = ST_CLOSING
        elif st == ST_CLOSING:
            frac = max(0.0, min(1.0, (sig - t_on) / (full - t_on)))
            intent = CLOSE_MIN + int(frac * (1000 - CLOSE_MIN))
            if bl >= RELAX_MS: st = ST_HOLDING          # hysteresis: release at T_OFF
        elif st == ST_HOLDING:
            if ah >= DEBOUNCE_MS: st = ST_CLOSING
            elif bl >= OPEN_DWELL_MS: st = ST_OPENING; open_t = now
        elif st == ST_OPENING:
            intent = -OPEN_SPEED
            if ah >= DEBOUNCE_MS: st = ST_CLOSING
            elif now - open_t >= OPEN_DRIVE_MS: st = ST_OPEN
        intents.append(intent); states.append(st)

    n = len(states)
    frac = {s: states.count(s) / n * 100 for s in range(4)}
    changes = sum(1 for i in range(1, n) if states[i] != states[i - 1])
    fw = [x for x in firmware_intent if x is not None]
    fw_pos = 100 * sum(1 for x in fw if x > 0) / len(fw) if fw else float("nan")
    print(f"{os.path.basename(path):34} floor={floor:5d} T_on={t_on:5d} T_off={t_off:5d} | "
          f"OPEN {frac[0]:4.0f}%  CLOSING {frac[1]:4.0f}%  HOLDING {frac[2]:4.0f}%  "
          f"OPENING {frac[3]:4.0f}% | {changes:3d} changes | firmware was {fw_pos:4.0f}% closing")
    return {"path": path, "sig_floor": floor, "th": (t_on, t_off, full),
            "intents": intents, "states": states, "B": B}


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    files = [sys.argv[1]] if len(sys.argv) > 1 else sorted(
        glob.glob(os.path.join(root, "data", "emg_2026-07-2[13]_*.csv")))
    print(f"offsets: T_on=floor+{ON_OFF}  T_off=floor+{OFF_OFF}  full=floor+{FULL_OFF}\n")
    for f in files:
        run(f)


if __name__ == "__main__":
    main()
