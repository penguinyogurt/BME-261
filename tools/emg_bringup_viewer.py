#!/usr/bin/env python3
"""
EMG bring-up viewer for the ESP32 test firmware.

Reads "chA,chB" lines over serial, shows a live rolling plot of both
channels, and logs every sample to a timestamped CSV in the current folder.

Setup (once):
    pip install pyserial matplotlib

Run:
    python emg_bringup_viewer.py --port COM5            # Windows
    python emg_bringup_viewer.py --port /dev/ttyUSB0    # Linux
    python emg_bringup_viewer.py --port /dev/cu.usbserial-XXXX   # macOS

Close the plot window to stop; the CSV is flushed and saved on exit.
"""
import argparse
import csv
import os
import threading
import time
from collections import deque
from datetime import datetime

import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

WINDOW = 1500   # samples on screen (~3 s at 500 Hz)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(2.0)              # allow the board to auto-reset
    ser.reset_input_buffer()
    ser.write(b"p")             # ask firmware for the plot stream

    # always land in data/ next to the other captures, whatever the cwd is
    data_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")
    os.makedirs(data_dir, exist_ok=True)
    fname = os.path.join(data_dir,
                         datetime.now().strftime("emg_%Y-%m-%d_%H%M%S.csv"))
    csv_file = open(fname, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow(["host_time_s", "chA_raw", "chB_env", "env", "intent"])
    print(f"logging to {fname}  (close the plot window to stop)")

    bufA = deque([0] * WINDOW, maxlen=WINDOW)
    bufB = deque([0] * WINDOW, maxlen=WINDOW)
    bufI = deque([0] * WINDOW, maxlen=WINDOW)   # intent the firmware is sending
    lock = threading.Lock()
    running = {"go": True}

    def reader():
        # Drain the whole serial buffer each pass (not one readline per sample):
        # at 500 Hz, line-by-line reads fall behind and the OS buffer backs up,
        # so the plot shows ever-older data. Bulk-draining keeps the reader ahead;
        # the display deque (maxlen=WINDOW) then keeps only the newest samples, so
        # latency can't accumulate. Every sample is still logged, in batches.
        buf = b""
        pending = []                 # CSV rows waiting to be flushed
        prev_t = time.time()
        last_flush = prev_t
        while running["go"]:
            try:
                n = ser.in_waiting
                chunk = ser.read(n if n else 1)   # all pending, or block for 1 byte
            except Exception:
                continue
            now = time.time()
            if chunk:
                buf += chunk
                lines = buf.split(b"\n")
                buf = lines.pop()                 # keep the incomplete remainder
                samples = []
                for raw in lines:
                    line = raw.decode("ascii", "ignore").strip()
                    if "," not in line:
                        continue                  # skip self-test / status text
                    parts = line.split(",")
                    # "chA,chB,env,intent"; older firmware sent just "chA,chB"
                    if len(parts) not in (2, 4):
                        continue
                    try:
                        vals = [int(p) for p in parts]
                    except ValueError:
                        continue
                    if len(vals) == 2:
                        vals += [0, 0]
                    samples.append(tuple(vals))
                if samples:
                    with lock:
                        bufA.extend(s[0] for s in samples)
                        bufB.extend(s[1] for s in samples)
                        bufI.extend(s[3] for s in samples)
                    # spread receipt timestamps evenly across this chunk so the
                    # CSV keeps ~uniform dt instead of collapsing to one time
                    dt = (now - prev_t) / len(samples)
                    for k, (a, b, e, i) in enumerate(samples):
                        pending.append((f"{prev_t + (k + 1) * dt:.4f}", a, b, e, i))
                    prev_t = now
            if pending and now - last_flush >= 0.1:
                writer.writerows(pending)
                csv_file.flush()
                pending.clear()
                last_flush = now
        if pending:                              # final flush on shutdown
            writer.writerows(pending)
            csv_file.flush()

    threading.Thread(target=reader, daemon=True).start()

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(9, 8))
    ax1.set_title("ch A - raw bandpass (biased, should rest near 2048)")
    ax2.set_title("ch B - peak detector envelope")
    ax3.set_title("intent  (+ close / - open, 0 = hold)")
    ax3.set_xlabel("samples")
    line1, = ax1.plot(range(WINDOW), list(bufA))
    line2, = ax2.plot(range(WINDOW), list(bufB))
    line3, = ax3.plot(range(WINDOW), list(bufI), color="tab:orange")
    for ax in (ax1, ax2):
        ax.set_ylim(0, 4095)     # full ADC range makes clipping obvious
    ax3.set_ylim(-1100, 1100)    # intent is bounded, so use its own scale
    ax3.axhline(0, color="0.6", linewidth=0.8)
    for ax in (ax1, ax2, ax3):
        ax.grid(True, alpha=0.3)
    fig.tight_layout()

    def update(_):
        with lock:
            a, b, i = list(bufA), list(bufB), list(bufI)
        line1.set_ydata(a)
        line2.set_ydata(b)
        line3.set_ydata(i)
        return line1, line2, line3

    _ani = FuncAnimation(fig, update, interval=33, blit=True,
                         cache_frame_data=False)
    try:
        plt.show()
    finally:
        running["go"] = False
        time.sleep(0.2)
        ser.close()
        csv_file.close()
        print(f"saved {fname}")


if __name__ == "__main__":
    main()
