#!/usr/bin/env python3
"""Refresh demo/ from the canonical firmware and tools.

    python tools/sync_demo.py            # copy canonical -> demo/
    python tools/sync_demo.py --check    # report drift, change nothing (exit 1 if stale)

demo/ holds COPIES so a demo can be run without hunting through the repo. Copies
drift, and this one already did: demo/ was carrying the pre-dualrate collector
(814 lines behind) and the pre-measurement receiver, so anyone flashing from it
would have got the firmware that creeps shut. That is worse than having no copy
at all, hence this script.

Canonical always wins. Edit the real file, then run this.
"""
import argparse
import filecmp
import os
import shutil
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# (canonical, demo copy). Straight byte-for-byte copies.
PAIRS = [
    ("firmware/emg-collector/emg-collector.ino",
     "demo/firmware/emg-collector/emg-collector.ino"),
    ("firmware/servo-receiver/src/main.cpp",
     "demo/firmware/servo-receiver/src/main.cpp"),
    ("firmware/servo-receiver/platformio.ini",
     "demo/firmware/servo-receiver/platformio.ini"),
    ("firmware/espnow-link-test/espnow-link-test.ino",
     "demo/firmware/espnow-link-test/espnow-link-test.ino"),
    ("tools/emg_bringup_viewer.py", "demo/emg_bringup_viewer.py"),
    ("tools/motor_debug.py", "demo/motor_debug.py"),
    ("tools/dual_serial_monitor.py", "demo/dual_serial_monitor.py"),
]

# The Arduino-IDE sketch is main.cpp with a header prepended, because a .ino
# must live in a folder matching its name. Same code, different packaging.
INO_HEADER = """\
// ARDUINO IDE COPY of firmware/servo-receiver/src/main.cpp - byte-identical
// below this header. Same code, packaged as a sketch so it can be flashed
// without PlatformIO. Needs the ESP32Servo library (Library Manager).
// If you edit the receiver, edit main.cpp and re-copy; do not fork this.
"""
INO_SRC = "firmware/servo-receiver/src/main.cpp"
INO_DST = "demo/firmware/servo-receiver-ino/servo-receiver-ino.ino"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true",
                    help="report drift without writing anything")
    args = ap.parse_args()

    stale = []
    for src, dst in PAIRS:
        s, d = os.path.join(ROOT, src), os.path.join(ROOT, dst)
        if not os.path.exists(s):
            print(f"  !! canonical missing: {src}")
            stale.append(dst)
            continue
        same = os.path.exists(d) and filecmp.cmp(s, d, shallow=False)
        if same:
            continue
        stale.append(dst)
        if not args.check:
            os.makedirs(os.path.dirname(d), exist_ok=True)
            shutil.copyfile(s, d)

    # the .ino: header + canonical body
    s, d = os.path.join(ROOT, INO_SRC), os.path.join(ROOT, INO_DST)
    want = INO_HEADER + open(s, encoding="utf-8").read()
    have = open(d, encoding="utf-8").read() if os.path.exists(d) else None
    if have != want:
        stale.append(INO_DST)
        if not args.check:
            os.makedirs(os.path.dirname(d), exist_ok=True)
            with open(d, "w", encoding="utf-8", newline="") as fh:
                fh.write(want)

    if not stale:
        print("demo/ is in sync with canonical")
        return 0
    verb = "STALE" if args.check else "updated"
    for p in stale:
        print(f"  {verb}: {p}")
    if args.check:
        print(f"\n{len(stale)} file(s) stale — run: python tools/sync_demo.py")
        return 1
    print(f"\n{len(stale)} file(s) refreshed from canonical")
    return 0


if __name__ == "__main__":
    sys.exit(main())
