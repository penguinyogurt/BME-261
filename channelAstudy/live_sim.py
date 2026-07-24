"""Live-stream simulation: replays a recording in real time through the
detector and prints actuation events as they fire.

Usage:
    python3 live_sim.py <recording.csv> [--speed N] [--quiet]

--speed 0  runs as fast as possible (no sleeping)
--speed 1  true real time (default)
"""
import sys, time, csv, argparse
sys.path.insert(0, '.')
from emg_detector import EMGIntentDetector


def stream(path, speed=1.0, quiet=False):
    det = EMGIntentDetector()
    prev = False
    t_on = None
    n = 0
    events = []
    t0 = time.perf_counter()
    with open(path) as fh:
        for row in csv.DictReader(fh):
            ts = float(row['host_time_s'])
            a = int(float(row['chA_raw']))
            b = int(float(row['chB_env']))
            if n == 0:
                base_ts = ts
            rel = ts - base_ts

            active = det.push(a, b)

            if active and not prev:
                t_on = rel
                if not quiet:
                    print(f"[{rel:8.3f}s] ACTUATION START   env={det.envelope:7.1f} "
                          f"thr={det.hi:7.1f} base={det.baseline:6.1f}")
            elif prev and not active:
                dur = rel - (t_on if t_on is not None else rel)
                events.append((t_on, rel, dur))
                if not quiet:
                    print(f"[{rel:8.3f}s] ACTUATION END     duration={dur:.3f}s")
            prev = active
            n += 1

            if speed > 0:
                target = t0 + rel / speed
                now = time.perf_counter()
                if target > now:
                    time.sleep(target - now)

    if prev and t_on is not None:
        events.append((t_on, rel, rel - t_on))
    wall = time.perf_counter() - t0
    print(f"\n{len(events)} actuations in {rel:.1f}s of signal "
          f"({n} samples, {wall:.2f}s wall, {wall/n*1e6:.1f} us/sample)")
    return events


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('--speed', type=float, default=1.0,
                    help='0 = as fast as possible, 1 = real time')
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args()
    stream(args.csv, args.speed, args.quiet)
