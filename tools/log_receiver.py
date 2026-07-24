#!/usr/bin/env python3
"""Log the servo receiver's telemetry to CSV, to diagnose grip travel.

    python tools/log_receiver.py --port COM7

Ctrl-C to stop; the CSV is flushed on exit and a summary is printed.

The receiver prints one telemetry line ~5 Hz:

    RX #123 seq=456 50Hz | intent=700 state=CLOSING cal=1 | us=1920 grip=48%

This records t, seq, intent, state, us and grip so the OPEN phases can be
checked: when the sender stops commanding open (intent returns to 0 and state
leaves OPENING), grip% should be 0. Anything above 0 at that moment means the
sender's gripEst hit zero while the receiver still believed the hand was
part-closed — the two dead-reckoned estimates have diverged.
"""
import argparse
import csv
import os
import re
import sys
import time
from datetime import datetime

import serial

LINE = re.compile(
    r"seq=(\d+).*?intent=(-?\d+)\s+state=(\w+).*?us=(\d+)\s+grip=(-?\d+)%")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="receiver port, e.g. COM7")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    outdir = os.path.join(root, "data")
    os.makedirs(outdir, exist_ok=True)
    path = os.path.join(outdir,
                        datetime.now().strftime("receiver_%Y-%m-%d_%H%M%S.csv"))

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    time.sleep(2.0)                 # board auto-resets when the port opens
    ser.reset_input_buffer()

    fh = open(path, "w", newline="")
    w = csv.writer(fh)
    w.writerow(["t_s", "seq", "intent", "state", "us", "grip_pct"])
    print(f"logging to {os.path.relpath(path, root)}   (ctrl-c to stop)")
    print(f"{'t':>7} {'intent':>7} {'state':>9} {'us':>6} {'grip':>6}")

    rows, buf, t0 = [], "", time.time()
    prev_state = None
    try:
        while True:
            n = ser.in_waiting
            if not n:
                time.sleep(0.02)
                continue
            buf += ser.read(n).decode("ascii", "ignore")
            *lines, buf = buf.split("\n")
            for ln in lines:
                m = LINE.search(ln)
                if not m:
                    continue
                t = time.time() - t0
                seq, intent, state, us, grip = m.groups()
                row = [f"{t:.2f}", int(seq), int(intent), state, int(us), int(grip)]
                w.writerow(row); rows.append(row)
                # only print on change or every ~1 s, so the console stays readable
                if state != prev_state:
                    print(f"{t:7.2f} {int(intent):7} {state:>9} {int(us):6} "
                          f"{int(grip):5}%   <- state change")
                    prev_state = state
                elif len(rows) % 5 == 0:
                    print(f"{t:7.2f} {int(intent):7} {state:>9} {int(us):6} "
                          f"{int(grip):5}%")
            fh.flush()
    except KeyboardInterrupt:
        pass
    finally:
        ser.close(); fh.close()

    if not rows:
        print("\nno telemetry parsed — is the receiver powered and on this port?")
        return

    grips = [int(r[5]) for r in rows]
    print(f"\nwrote {os.path.relpath(path, root)}  ({len(rows)} rows)")
    print(f"grip%: min {min(grips)}  max {max(grips)}  final {grips[-1]}")

    # every OPENING run: what was grip% when the sender gave up?
    ends = [i for i in range(1, len(rows))
            if rows[i - 1][3] == "OPENING" and rows[i][3] != "OPENING"]
    if ends:
        print("\ngrip% at the moment each OPEN gave up (should be 0):")
        for i in ends:
            print(f"   t={rows[i][0]:>7}s  -> {rows[i][5]}%")
        worst = max(int(rows[i][5]) for i in ends)
        print(f"\nworst residual: {worst}%")
        print("open fully unwinds the close" if worst <= 2 else
              "OPEN IS QUITTING EARLY — sender gripEst and receiver gripPos disagree")
    else:
        print("\nno completed OPENING phase captured — relax for >2 s to trigger one")


if __name__ == "__main__":
    main()
