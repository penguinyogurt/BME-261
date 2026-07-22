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

    fname = datetime.now().strftime("emg_%Y-%m-%d_%H%M%S.csv")
    csv_file = open(fname, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow(["host_time_s", "chA_raw", "chB_env"])
    print(f"logging to {fname}  (close the plot window to stop)")

    bufA = deque([0] * WINDOW, maxlen=WINDOW)
    bufB = deque([0] * WINDOW, maxlen=WINDOW)
    lock = threading.Lock()
    running = {"go": True}

    def reader():
        while running["go"]:
            try:
                line = ser.readline().decode("ascii", "ignore").strip()
            except Exception:
                continue
            if "," not in line:
                continue                     # skip self-test / status text
            parts = line.split(",")
            if len(parts) != 2:
                continue
            try:
                a, b = int(parts[0]), int(parts[1])
            except ValueError:
                continue
            with lock:
                bufA.append(a)
                bufB.append(b)
            writer.writerow([f"{time.time():.4f}", a, b])

    threading.Thread(target=reader, daemon=True).start()

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
    ax1.set_title("ch A - raw bandpass (biased, should rest near 2048)")
    ax2.set_title("ch B - peak detector envelope")
    ax2.set_xlabel("samples")
    line1, = ax1.plot(range(WINDOW), list(bufA))
    line2, = ax2.plot(range(WINDOW), list(bufB))
    for ax in (ax1, ax2):
        ax.set_ylim(0, 4095)     # full ADC range makes clipping obvious
        ax.grid(True, alpha=0.3)

    def update(_):
        with lock:
            a, b = list(bufA), list(bufB)
        line1.set_ydata(a)
        line2.set_ydata(b)
        return line1, line2

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
