# EMG Actuation-Intent Detection — Recommended Method

**Result: leave-one-out F1 = 0.763** (precision 0.714, recall 0.779, median onset latency 74 ms) across 12 recordings, 684 reference bursts.

---

## The algorithm

Channel A only. Channel B is used *solely* for dropout detection.

```
1. DC removal      causal running median, 101 samples (202 ms)
2. Bandpass        Butterworth 4th order, 20–200 Hz, causal (sosfilt)
3. TKEO            y[n]² − y[n−1]·y[n+1], one-sample lag, clip negatives
4. Envelope        trailing mean of √TKEO, 80 ms

5. Baseline        rolling 20th percentile over 3 s
   Sigma           1.4826 × rolling median |env − baseline| over 3 s
   Amplitude       rolling 99th percentile over 20 s, minus baseline

6. Hybrid threshold
      hi = max(base + 0.8·sigma,  base + 0.12·amp)
      lo = max(base + 0.2·sigma,  base + 0.06·amp)

7. Schmitt trigger  confirm ON after 50 ms above hi
                    release OFF after 100 ms below lo

8. Dropout gate     chA == 0 AND chB == 0 → force OFF (50 ms hold)
```

O(1) state per sample. No training, no matrix ops, no allocation in steady state.
**12.4 µs/sample in pure Python — 161× real-time headroom at 500 Hz.**

---

## Why the hybrid threshold is the key idea

This is the one non-obvious part, and it is what makes weak signals detectable without flooding on strong ones.

A **pure σ-multiple threshold fails** because σ is unstable across recordings — measured 3.6 to 17.8 across your files — and collapses toward zero whenever quiet valleys dominate the sample distribution. When that happens even an 8σ threshold sits near the noise floor and the detector latches on permanently.

A **pure amplitude-fraction threshold fails** on low-gain recordings, where the whole signal is compressed and a fixed fraction of peak sits above the real bursts.

Taking `max()` of both means the σ term governs on weakly-amplified channels (keeping sensitivity) while the amplitude term takes over whenever σ collapses (preventing flood). Same fix corrected the offline labeler, which is why it transferred cleanly.

---

## Channel B: do not use it for intent

Tested four ways against the tuned channel-A detector:

| Configuration | Precision | Recall | F1 |
|---|---|---|---|
| **chA only** | 0.706 | 0.788 | **0.745** |
| chA OR chB | 0.706 | 0.759 | 0.732 |
| chA AND chB | 0.784 | 0.529 | 0.632 |
| chB rescues weak A | 0.703 | 0.788 | 0.743 |
| chB alone | 0.897 | 0.154 | 0.262 |

Nothing beats channel A alone. On its own chB detects **one burst in seven** — its hardware peak-detector rails at 4095 on a large fraction of samples, and its slow decay envelope smears onsets, pushing latency to 92–168 ms versus 74 ms.

Keep it for the dropout flag. The both-channels-zero condition is a reliable sensor-disconnect signal and that gate is worth having.

---

## Validation

**Leave-one-out**: tune on 11 recordings, test on the held-out 1, repeated 12×.

| Held out | F1 | | Held out | F1 |
|---|---|---|---|---|
| 134851 | 0.941 | | 190832 | 0.642 |
| 193836 | 0.825 | | 185937 | 0.730 |
| 185816 | 0.820 | | 182349 | 0.735 |
| 195519 | 0.797 | | 185723 | 0.743 |
| 200457 | 0.797 | | 171239 | 0.762 |
| 200828 | 0.771 | | 202105 | 0.588 |

**Mean 0.763, min 0.588, std 0.087.** The same parameter set won on **11 of 12 folds** — that stability is the important number. It means the tuning is not overfit and an unseen recording should land near 0.76.

**Causality proofs** (all pass on the shipping code):
- Randomizing all future samples leaves every past output bit-identical
- Incremental replay equals batch replay
- Truncating the stream does not change outputs on the prefix
- `reset()` restores fresh-instance behaviour

**Filter check**: the dependency-free Butterworth matches `scipy.signal.butter` to 4×10⁻⁷ in magnitude response.

**Standalone vs research pipeline**: 97.3% sample agreement (residual is exact rolling quantiles vs pandas interpolation).

---

## Operating points

Tune `frac_hi` (and keep `frac_lo = frac_hi / 2`) to trade precision against recall:

| frac_hi | Precision | Recall | F1 | Latency |
|---|---|---|---|---|
| 0.06 | 0.658 | 0.763 | 0.707 | 68 ms |
| 0.08 | 0.687 | 0.787 | 0.733 | 70 ms |
| 0.10 | 0.706 | 0.788 | 0.745 | 72 ms |
| **0.12** | **0.716** | **0.785** | **0.749** | **72 ms** |
| 0.14 | 0.722 | 0.766 | 0.743 | 74 ms |
| 0.18 | 0.723 | 0.722 | 0.723 | 78 ms |
| 0.28 | 0.728 | 0.627 | 0.674 | 90 ms |
| 0.35 | 0.756 | 0.566 | 0.647 | 104 ms |

Precision saturates around 0.73 — pushing past it costs recall fast. If a false actuation is much more expensive than a missed one, `0.18` is the sensible stop; beyond that you lose real bursts for very little gain.

For **lower latency**, drop `on_ms` from 50 to 20–30 ms. Costs a few points of precision (more brief noise excursions confirm) but pulls median onset to ~40 ms.

---

## Reference standard (how the labels were made)

The ground truth is **acausal and not deployable** — that is the point of it.

- Zero-phase bandpass (`sosfiltfilt`), so onsets are not time-shifted
- TKEO envelope, 40 ms centred smoothing
- **Iterative sigma-clipping** for the noise floor: repeatedly re-estimate μ/σ from samples below μ+3σ, so bursts cannot inflate the scale estimate
- Two-pass locally-adaptive baseline over ±10 s windows, excluding pass-1 activity
- Hybrid threshold (same idea as live), hysteresis, 60 ms minimum burst, 120 ms gap merge

The first version of this labeler was **badly under-sensitive** — it found 312 bursts because whole-recording MAD was inflated ~5× by the bursts themselves, putting a 5σ threshold near the 97th percentile. The corrected version finds **684**. Two recordings originally scored as "artifact-only, zero bursts" turned out to contain 33 each.

---

## Caveats worth weighing

- **The 0.763 is agreement with my labeler, not with truth.** Everything downstream inherits any bias in those labels. You reviewed and approved them, but the detector is tuned to reproduce one specific labeling philosophy.
- **Worst folds are 202105 (0.588) and 190832 (0.642)** — the two heaviest-dropout recordings (37% and 52% zero samples). Expect degradation with poor electrode contact.
- **Precision is the weaker side.** 214 false positives across ~13 minutes of activity.
- **The detector needs ~2 s of warm-up** before it will fire at all, and the 20 s amplitude window means it takes that long to fully settle after a gain change or electrode reseat.
- **Reference labels merge bursts closer than 120 ms** and drop anything under 60 ms, so genuine rapid double-taps are counted as one event. If you need to resolve those, both the labeler and `off_ms` need retuning.
- Single-session data only — all 12 recordings are from one day. Cross-day and cross-subject generalization is untested.

---

## Files

| File | Purpose |
|---|---|
| `emg_detector.py` | The detector. Dependency-free, drop into your codebase. |
| `live_sim.py` | Real-time replay demo; prints events as they fire. |
| `ground_truth_bursts.csv` | 684 reference bursts with SNR and timing. |
| `operating_points.csv` | Full precision/recall curve. |
| `live_vs_truth/` | Per-recording: offline truth (green) vs live detector (blue). |
| `verify/` | Per-recording ground-truth audit sheets. |

```bash
# real-time replay
python3 live_sim.py recording.csv

# as fast as possible
python3 live_sim.py recording.csv --speed 0
```

```python
from emg_detector import EMGIntentDetector

det = EMGIntentDetector()                 # or EMGIntentDetector(frac_hi=0.18)
for chA, chB in stream:
    if det.push(chA, chB):
        fire_actuator()
```
