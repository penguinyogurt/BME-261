"""Generate synthetic EMG bring-up CSVs for scenario testing.

These mimic the format and statistics of a real capture
(emg_2026-07-21_140447.csv) so the Python viewer / any downstream analysis
can be exercised on more gestures than were physically recorded.

Output columns match the real logs exactly: host_time_s,chA_raw,chB_env
  - chA_raw : DEAD channel in the real capture - the raw bandpass output never
              carried real signal, so here it is pure resting noise around its
              DC bias (~498 counts) and is NOT driven by the gesture.
  - chB_env : peak-detector envelope - the only meaningful channel. Rests ~3051
              counts and RISES with activation, occasionally railing at 4095.

Only chB is driven by the activation timeline act(t) in [0,1]. A "gesture" is a
burst of activation shaped by a fast-attack / slow-release peak-detector
envelope. chA is generated independently as background noise.

Run:  python make_synthetic_emg.py
Writes emg_synth_<scenario>.csv next to this script. Deterministic per
scenario (seeded), so re-running reproduces identical files.
"""
import csv
import math
import os
import random

# ---- ADC / signal constants tuned to emg_2026-07-21_140447.csv ----
ADC_MAX = 4095
CHA_BASE = 498      # raw resting DC bias (counts)
CHA_REST_SIGMA = 6  # raw resting noise
CHB_BASE = 3051     # envelope resting level
CHB_REST_SIGMA = 14
CHB_FLOOR = 2949    # lowest envelope value seen at rest
CHB_GAIN = 830      # envelope rise at full activation (-> ~3881, occasional rail)

DT = 0.0025         # nominal sample period (~400 Hz effective, matches capture)
DT_JITTER = 0.0003  # +/- uniform timing jitter
GAP_PROB = 0.012    # chance of a serial-hiccup gap on any given sample
GAP_MIN, GAP_MAX = 0.006, 0.090  # gap length range (s)

START_EPOCH = 1784660000.0  # arbitrary plausible epoch (~July 2026)

# Envelope smoothing (asymmetric, emulates a peak detector):
ATTACK_TAU = 0.045   # s, fast rise
RELEASE_TAU = 0.170  # s, slow fall


def smooth_activation(targets, dt=DT):
    """Asymmetric one-pole smoothing of a target activation series."""
    out = []
    a = 0.0
    for tgt in targets:
        tau = ATTACK_TAU if tgt > a else RELEASE_TAU
        alpha = 1.0 - math.exp(-dt / tau)
        a += alpha * (tgt - a)
        out.append(a)
    return out


def build_target(segments, dt=DT):
    """Expand [(duration_s, level), ...] into a per-sample target array."""
    tgt = []
    for dur, level in segments:
        n = max(1, int(round(dur / dt)))
        tgt.extend([level] * n)
    return tgt


def render(name, segments, seed):
    """Turn an activation script into (chA_raw, chB_env) samples + timestamps."""
    rng = random.Random(seed)
    target = build_target(segments)
    act = smooth_activation(target)

    rows = []
    t = START_EPOCH
    for a in act:
        # chB envelope: baseline + activation lift + activity-scaled noise
        env = CHB_BASE + a * CHB_GAIN + rng.gauss(0.0, CHB_REST_SIGMA + a * 38.0)
        env = int(round(max(CHB_FLOOR, min(ADC_MAX, env))))

        # chA raw: dead channel - background noise only, no gesture coupling.
        raw = CHA_BASE + rng.gauss(0.0, CHA_REST_SIGMA)
        if rng.random() < 0.01:                 # rare small movement-artifact pop
            raw += rng.uniform(20.0, 90.0)
        raw = int(round(max(0, min(ADC_MAX, raw))))

        rows.append((t, raw, env))

        # advance host clock with jitter + occasional serial gap
        step = DT + rng.uniform(-DT_JITTER, DT_JITTER)
        if rng.random() < GAP_PROB:
            step += rng.uniform(GAP_MIN, GAP_MAX)
        t += step

    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "data", f"emg_synth_{name}.csv")
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["host_time_s", "chA_raw", "chB_env"])
        for t, raw, env in rows:
            w.writerow([f"{t:.4f}", raw, env])
    dur = rows[-1][0] - rows[0][0]
    print(f"wrote {os.path.basename(out):32s} {len(rows):5d} samples  {dur:5.1f}s")


# ---- Scenario library: activation scripts (duration_s, level 0..1) ----
# level 0 = muscle at rest, ~0.9 = strong contraction.
SCENARIOS = {
    # single clean grip: one contraction, brief hold, release
    "single_close": ([
        (2.0, 0.0), (0.30, 0.90), (1.5, 0.88), (0.5, 0.0), (2.0, 0.0),
    ], 101),

    # open gesture, relax, then a close gesture
    "open_close": ([
        (2.0, 0.0),
        (0.30, 0.80), (0.8, 0.78), (0.45, 0.0),   # open
        (1.5, 0.0),
        (0.25, 0.92), (1.0, 0.90), (0.5, 0.0),    # close
        (2.0, 0.0),
    ], 102),

    # close, hold under load (slight fatigue droop), then open, release
    "close_hold_open": ([
        (2.0, 0.0),
        (0.30, 0.88),                              # close onset
        (2.0, 0.85), (2.0, 0.74),                  # hold w/ fatigue droop
        (0.40, 0.0),                               # release
        (0.8, 0.0),
        (0.25, 0.70), (0.5, 0.68), (0.4, 0.0),     # open
        (2.0, 0.0),
    ], 103),

    # rapid alternating taps (open/close reps) of varying strength
    "rapid_open_close_reps": ([
        (1.5, 0.0),
        (0.20, 0.85), (0.30, 0.0),
        (0.20, 0.70), (0.30, 0.0),
        (0.20, 0.95), (0.30, 0.0),
        (0.20, 0.65), (0.30, 0.0),
        (0.20, 0.88), (0.30, 0.0),
        (1.5, 0.0),
    ], 104),

    # sustained grip with progressive fatigue decline
    "sustained_grip_fatigue": ([
        (2.0, 0.0),
        (0.35, 0.92),
        (2.5, 0.90), (2.5, 0.78), (2.5, 0.64), (2.5, 0.52),  # slow decline
        (0.6, 0.0),
        (2.0, 0.0),
    ], 105),

    # slow graded ramp up then down - good for threshold/calibration testing
    "graded_ramp": ([
        (1.5, 0.0),
    ] + [(0.25, lvl / 100.0) for lvl in range(0, 96, 5)]      # ramp up
      + [(0.25, lvl / 100.0) for lvl in range(95, -1, -5)]    # ramp down
      + [(1.5, 0.0)],
      106),
}


if __name__ == "__main__":
    for name, (segments, seed) in SCENARIOS.items():
        render(name, segments, seed)
