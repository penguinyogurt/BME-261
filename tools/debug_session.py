"""Log both boards during a live test so a failed run can be debugged after.

Run this, then drive the EMG board by typing commands (t, c, ca, cb, i, p) and
pressing Enter - they are forwarded to it. Every line from BOTH boards is
timestamped into output/debug_<stamp>.log with a shared clock, so the EMG side
and the servo side can be lined up. The console hides the 50 Hz numeric stream
so the calibration prompts stay readable; the log keeps everything.

    python tools/debug_session.py                  # COM6 emg, COM7 servo
    python tools/debug_session.py --emg COM6 --servo COM7

Type 'quit' (or Ctrl-C) to stop. Opening a port resets that board, so expect a
boot + self-test first; wait for "EMG collector ready." before typing.
"""
import argparse
import os
import sys
import threading
import time

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--emg", default="COM6")
    ap.add_argument("--servo", default="COM7")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--source", default="cb", choices=["ca", "cb"],
                    help="which channel drives intent (default cb)")
    ap.add_argument("--auto", action="store_true",
                    help="force the timed sequence even on a tty")
    ap.add_argument("--no-cal", action="store_true",
                    help="skip the calibration trial and test the stored thresholds")
    args = ap.parse_args()

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    outdir = os.path.join(root, "output")
    os.makedirs(outdir, exist_ok=True)
    path = os.path.join(outdir, time.strftime("debug_%Y%m%d_%H%M%S.log"))
    log = open(path, "w", encoding="utf-8")

    t0 = time.time()
    lock = threading.Lock()
    stop = threading.Event()

    def open_port(p):
        try:
            s = serial.Serial(p, args.baud, timeout=0.2)
            s.setDTR(False); s.setRTS(False)   # don't hold the board in reset
            return s
        except serial.SerialException as e:
            print(f"cannot open {p}: {e}")
            print("  -> close the Arduino Serial Monitor (or any other monitor) on it first")
            sys.exit(1)

    emg = open_port(args.emg)
    srv = open_port(args.servo)

    def record(tag, line):
        rec = f"{time.time() - t0:8.3f} [{tag}] {line}"
        with lock:
            log.write(rec + "\n")
            log.flush()
        return rec

    def pump(ser, tag):
        # Bulk-drain: one readline() per sample cannot keep up with the 500 Hz
        # plot stream, the OS buffer backs up, and every timestamp ends up
        # seconds late (countdowns appeared 2 ms apart). Read all pending bytes.
        buf = b""
        while not stop.is_set():
            try:
                n = ser.in_waiting
                chunk = ser.read(n if n else 1)
            except serial.SerialException:
                break
            if not chunk:
                continue
            buf += chunk
            lines = buf.split(b"\n")
            buf = lines.pop()                     # keep incomplete remainder
            for raw in lines:
                line = raw.decode("ascii", "ignore").rstrip()
                if not line:
                    continue
                rec = record(tag, line)
                # console: hide the high-rate "n,n" stream, keep status/prompts
                parts = line.split(",")
                numeric = len(parts) == 2 and all(
                    x.strip().lstrip("-").isdigit() for x in parts)
                if not numeric:
                    print(rec, flush=True)

    for ser, tag in ((emg, "EMG"), (srv, "SRV")):
        threading.Thread(target=pump, args=(ser, tag), daemon=True).start()

    print(f"logging to {os.path.relpath(path, root)}")

    def send(cmd):
        emg.write(cmd.encode())
        record("CMD", cmd)

    # No interactive stdin (e.g. launched from a non-tty)? Drive the whole run on
    # a fixed timeline instead, so the operator just follows the clock.
    auto = args.auto or not sys.stdin.isatty()

    try:
        if auto and args.no_cal:
            # test the built-in thresholds as-is. The adaptive baseline settles
            # on its own, so no 't' either - just pick the channel and flex.
            plan = [
                (9,  args.source, "select intent source"),
                (12, "i",         "intent stream on - stay RELAXED a few seconds"),
                (18, None,        "now GRIP ~2 s, relax ~3 s, repeat"),
                (60, None,        "finishing"),
            ]
        elif auto:
            plan = [
                (9,  args.source, "select intent source"),
                (13, "t",         "re-baseline"),
                (17, "i",         "intent stream on"),
                (20, "c",         "CALIBRATE -> RELAX now, stay relaxed until t=26"),
                (26, None,        "SQUEEZE HARD now - hold it until t=32"),
                (32, None,        "relax - calibration done, now do normal grips"),
                (60, None,        "finishing"),
            ]
        if auto:
            print("AUTO MODE - follow this clock (t=0 now):")
            for t, cmd, why in plan:
                print(f"  t={t:>3}s  {why}")
            print()
            for t, cmd, why in plan:
                while not stop.is_set() and time.time() - t0 < t:
                    time.sleep(0.1)
                if stop.is_set():
                    break
                record("PLAN", why)
                print(f"--> t={t}s  {why}", flush=True)
                if cmd:
                    send(cmd)
        else:
            print("wait for 'EMG collector ready.', then type commands "
                  "(t / c / ca / cb / i / p); 'quit' to stop\n")
            for line in sys.stdin:
                cmd = line.strip()
                if cmd.lower() in ("quit", "exit"):
                    break
                if cmd:
                    send(cmd)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        time.sleep(0.3)
        emg.close(); srv.close(); log.close()
        print(f"\nsaved {os.path.relpath(path, root)}")


if __name__ == "__main__":
    main()
