"""Monitor two (or more) serial ports at once, interleaved in one console.

Works around Arduino IDE 1.x sharing a single global port selection across
windows, which makes two simultaneous Serial Monitors impossible.

Usage:
    python dual_serial_monitor.py                    # COM6 + COM7 @ 115200
    python dual_serial_monitor.py COM6 COM7          # explicit ports
    python dual_serial_monitor.py --reset            # reset boards on open
    python dual_serial_monitor.py --seconds 10       # exit after 10 s

Each line is prefixed with its port, e.g.:
    [COM6] RX from 7C:87:CE:30:FA:88  seq=12  they-heard=11  rssi=-35 dBm

Stop with Ctrl+C. Note: a port already opened by the Arduino Serial Monitor
(or any other program) cannot be opened here too - close it there first.
"""
import argparse
import sys
import threading
import time

import serial

def reader(port, baud, do_reset, stop):
    try:
        s = serial.Serial(port, baud, timeout=0.5)
    except serial.SerialException as e:
        print(f"[{port}] cannot open: {e} (close any monitor using it)")
        return
    try:
        if do_reset:
            s.setDTR(False)
            s.setRTS(True)
            time.sleep(0.1)
        s.setRTS(False)  # make sure nothing holds the board in reset/boot
        s.setDTR(False)
        while not stop.is_set():
            line = s.readline().decode(errors="replace").rstrip()
            if line:
                print(f"[{port}] {line}")
    finally:
        s.close()

def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("ports", nargs="*", default=["COM6", "COM7"],
                    help="serial ports to watch (default: COM6 COM7)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--reset", action="store_true",
                    help="pulse DTR/RTS so each board reboots on connect")
    ap.add_argument("--seconds", type=float, default=None,
                    help="exit automatically after this many seconds")
    args = ap.parse_args()
    ports = args.ports or ["COM6", "COM7"]

    stop = threading.Event()
    threads = [threading.Thread(target=reader, args=(p, args.baud, args.reset, stop),
                                daemon=True) for p in ports]
    for t in threads:
        t.start()
    print(f"watching {', '.join(ports)} @ {args.baud} - Ctrl+C to quit")
    try:
        if args.seconds is not None:
            time.sleep(args.seconds)
        else:
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        pass
    stop.set()
    for t in threads:
        t.join(timeout=1)

if __name__ == "__main__":
    main()
