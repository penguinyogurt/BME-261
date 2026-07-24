"""
EMG actuation-intent detector -- streaming reference implementation.

Channel A only (bandpass-filtered, amplified raw EMG).
Channel B is used solely to detect sensor dropout.

Contract:
    det = EMGIntentDetector(fs=500.0)
    active = det.push(chA_raw, chB_env)   # -> bool, one sample at a time

Strictly causal: output at sample n depends only on samples <= n.
O(1) state per sample. No numpy required. No training. No allocation
in the steady state.

Tuned config (leave-one-out F1 = 0.763 over 12 recordings):
    envelope 80 ms | floor window 3 s | amp window 20 s
    hi = max(base + 0.8*sigma, base + 0.12*amp)
    lo = max(base + 0.2*sigma, base + 0.06*amp)
    confirm 50 ms | release 100 ms
    median onset latency 72 ms
"""

import math
from collections import deque

__all__ = ["EMGIntentDetector", "DEFAULTS"]


DEFAULTS = dict(
    fs=500.0,
    bp_lo=20.0,          # bandpass low cutoff  (Hz)
    bp_hi=200.0,         # bandpass high cutoff (Hz)
    med_n=101,           # DC-removal running-median length (samples)
    env_ms=80.0,         # envelope smoothing window (ms)
    floor_s=3.0,         # baseline / scale estimation window (s)
    amp_s=20.0,          # amplitude-reference window (s)
    k_hi=0.8,            # sigma multiple, ON threshold
    k_lo=0.2,            # sigma multiple, OFF threshold
    frac_hi=0.12,        # amplitude fraction, ON threshold
    frac_lo=0.06,        # amplitude fraction, OFF threshold
    on_ms=50.0,          # confirm time above hi (ms)
    off_ms=100.0,        # release time below lo (ms)
    warmup_s=2.0,        # refuse to fire before this much valid signal
    dropout_hold_ms=50.0,  # hold-off after a dropout sample (ms)
)


# --------------------------------------------------------------------------
# Small streaming primitives
# --------------------------------------------------------------------------

class _Biquad:
    """Cascaded second-order sections, direct-form II transposed."""

    __slots__ = ("sos", "z")

    def __init__(self, sos):
        self.sos = [tuple(map(float, s)) for s in sos]
        self.z = [[0.0, 0.0] for _ in self.sos]

    def __call__(self, x):
        for i, (b0, b1, b2, a0, a1, a2) in enumerate(self.sos):
            zi = self.z[i]
            y = (b0 * x + zi[0]) / a0
            zi[0] = b1 * x - a1 * y + zi[1]
            zi[1] = b2 * x - a2 * y
            x = y
        return x

    def reset(self):
        for zi in self.z:
            zi[0] = zi[1] = 0.0


class _RunningMedian:
    """Exact running median over a trailing window.

    Insertion-sorted mirror list: O(w) worst case per sample but with a
    tiny constant, and w is small (101). Deterministic, no allocation.
    """

    __slots__ = ("n", "buf", "srt")

    def __init__(self, n):
        self.n = int(n)
        self.buf = deque()
        self.srt = []

    def __call__(self, x):
        srt = self.srt
        if len(self.buf) == self.n:
            old = self.buf.popleft()
            lo, hi = 0, len(srt)
            while lo < hi:
                mid = (lo + hi) // 2
                if srt[mid] < old:
                    lo = mid + 1
                else:
                    hi = mid
            srt.pop(lo)
        self.buf.append(x)
        lo, hi = 0, len(srt)
        while lo < hi:
            mid = (lo + hi) // 2
            if srt[mid] < x:
                lo = mid + 1
            else:
                hi = mid
        srt.insert(lo, x)
        m = len(srt)
        return srt[m // 2] if m & 1 else 0.5 * (srt[m // 2 - 1] + srt[m // 2])


class _RunningQuantile:
    """Exact rolling quantile over a trailing window (same structure)."""

    __slots__ = ("n", "q", "buf", "srt")

    def __init__(self, n, q):
        self.n = int(n)
        self.q = float(q)
        self.buf = deque()
        self.srt = []

    def __call__(self, x):
        srt = self.srt
        if len(self.buf) == self.n:
            old = self.buf.popleft()
            lo, hi = 0, len(srt)
            while lo < hi:
                mid = (lo + hi) // 2
                if srt[mid] < old:
                    lo = mid + 1
                else:
                    hi = mid
            srt.pop(lo)
        self.buf.append(x)
        lo, hi = 0, len(srt)
        while lo < hi:
            mid = (lo + hi) // 2
            if srt[mid] < x:
                lo = mid + 1
            else:
                hi = mid
        srt.insert(lo, x)
        m = len(srt)
        if m == 1:
            return srt[0]
        pos = self.q * (m - 1)
        i = int(pos)
        frac = pos - i
        if i + 1 < m:
            return srt[i] * (1.0 - frac) + srt[i + 1] * frac
        return srt[i]


class _MovMean:
    """Trailing moving average with a running sum."""

    __slots__ = ("n", "buf", "s")

    def __init__(self, n):
        self.n = max(int(n), 1)
        self.buf = deque()
        self.s = 0.0

    def __call__(self, x):
        self.buf.append(x)
        self.s += x
        if len(self.buf) > self.n:
            self.s -= self.buf.popleft()
        return self.s / len(self.buf)


def _butter_bandpass_sos(lo, hi, fs, order=4):
    """Butterworth bandpass as SOS via bilinear transform of analog
    prototype poles. Avoids a scipy dependency at runtime."""
    import cmath

    n = order  # lowpass prototype order; bandpass doubles it
    warp = lambda f: math.tan(math.pi * f / fs)
    wl, wh = warp(lo), warp(hi)
    bw = wh - wl
    w0sq = wl * wh

    # analog lowpass prototype poles (unit cutoff)
    poles = []
    for k in range(n):
        theta = math.pi * (2 * k + 1) / (2 * n) + math.pi / 2
        poles.append(cmath.exp(1j * theta))

    # lowpass -> bandpass:  s -> (s^2 + w0^2) / (bw*s)
    zs, ps = [], []
    for p in poles:
        alpha = bw * p / 2.0
        beta = cmath.sqrt(alpha * alpha - w0sq)
        ps.append(alpha + beta)
        ps.append(alpha - beta)
    zs = [0.0 + 0.0j] * n  # n zeros at origin, n at infinity

    # bilinear: s -> (1 - z^-1)/(1 + z^-1) with prewarped tan()
    def bilin(x):
        return (1.0 + x) / (1.0 - x)

    zd = [bilin(z) for z in zs]
    pd = [bilin(p) for p in ps]
    while len(zd) < len(pd):
        zd.append(-1.0 + 0.0j)  # zeros at Nyquist

    # gain normalisation at band centre
    wc = 2.0 * math.pi * math.sqrt(lo * hi) / fs
    ejw = cmath.exp(1j * wc)
    num = 1.0 + 0j
    den = 1.0 + 0j
    for z in zd:
        num *= (ejw - z)
    for p in pd:
        den *= (ejw - p)
    k = abs(den / num) if abs(num) > 0 else 1.0

    # pair conjugate poles/zeros into biquads
    def _pair(vals):
        vals = sorted(vals, key=lambda v: (-abs(v.imag), v.real))
        out, used = [], [False] * len(vals)
        for i, v in enumerate(vals):
            if used[i]:
                continue
            used[i] = True
            mate = None
            for j in range(i + 1, len(vals)):
                if used[j]:
                    continue
                if abs(vals[j] - v.conjugate()) < 1e-9:
                    mate = j
                    break
            if mate is None:
                for j in range(i + 1, len(vals)):
                    if not used[j]:
                        mate = j
                        break
            if mate is None:
                out.append((v, v.conjugate()))
            else:
                used[mate] = True
                out.append((v, vals[mate]))
        return out

    zp = _pair(zd)
    pp = _pair(pd)
    sos = []
    for i in range(len(pp)):
        z1, z2 = zp[i] if i < len(zp) else (-1.0 + 0j, -1.0 + 0j)
        p1, p2 = pp[i]
        b = [1.0, -(z1 + z2).real, (z1 * z2).real]
        a = [1.0, -(p1 + p2).real, (p1 * p2).real]
        sos.append([b[0], b[1], b[2], a[0], a[1], a[2]])
    if sos:
        sos[0][0] *= k
        sos[0][1] *= k
        sos[0][2] *= k
    return sos


# --------------------------------------------------------------------------
# Detector
# --------------------------------------------------------------------------

class EMGIntentDetector:
    """Streaming actuation-intent detector.

    Call push(chA, chB) once per sample. Returns True while an actuation
    is considered active.

    Extra state available after each push:
        .envelope   current smoothed TKEO envelope
        .baseline   current noise floor estimate
        .hi, .lo    current ON / OFF thresholds
        .dropout    True if the sensor is currently dropped out
    """

    def __init__(self, **kw):
        cfg = dict(DEFAULTS)
        unknown = set(kw) - set(cfg)
        if unknown:
            raise TypeError("unknown option(s): %s" % ", ".join(sorted(unknown)))
        cfg.update(kw)
        self.cfg = cfg
        fs = cfg["fs"]

        self._bp = _Biquad(_butter_bandpass_sos(cfg["bp_lo"], cfg["bp_hi"], fs))
        self._med = _RunningMedian(cfg["med_n"])
        self._env = _MovMean(round(cfg["env_ms"] * fs / 1000.0))
        self._floor = _RunningQuantile(round(cfg["floor_s"] * fs), 0.20)
        self._mad = _RunningQuantile(round(cfg["floor_s"] * fs), 0.50)
        self._amp = _RunningQuantile(round(cfg["amp_s"] * fs), 0.99)

        self._y1 = 0.0
        self._y2 = 0.0

        self._on_n = max(int(round(cfg["on_ms"] * fs / 1000.0)), 1)
        self._off_n = max(int(round(cfg["off_ms"] * fs / 1000.0)), 1)
        self._drop_n = max(int(round(cfg["dropout_hold_ms"] * fs / 1000.0)), 1)
        self._warm_n = int(round(cfg["warmup_s"] * fs))

        self._up = 0
        self._dn = 0
        self._state = False
        self._drop_c = 0
        self._n = 0

        self.envelope = 0.0
        self.baseline = 0.0
        self.sigma = 0.0
        self.amplitude = 0.0
        self.hi = float("inf")
        self.lo = float("inf")
        self.dropout = False

    # -- main entry point ---------------------------------------------------
    def push(self, chA, chB):
        cfg = self.cfg

        # 1. dropout gate: both channels exactly zero = sensor disconnected
        if chA == 0 and chB == 0:
            self._drop_c = self._drop_n
        elif self._drop_c > 0:
            self._drop_c -= 1
        self.dropout = self._drop_c > 0
        if self.dropout:
            self._state = False
            self._up = self._dn = 0
            return False

        self._n += 1
        x = float(chA)

        # 2. DC / drift removal
        x -= self._med(x)

        # 3. bandpass 20-200 Hz
        y = self._bp(x)

        # 4. TKEO (one-sample lag keeps it causal)
        tk = self._y1 * self._y1 - y * self._y2
        if tk < 0.0:
            tk = 0.0
        self._y2 = self._y1
        self._y1 = y

        # 5. envelope
        e = self._env(math.sqrt(tk))
        self.envelope = e

        # 6. adaptive statistics
        base = self._floor(e)
        sigma = 1.4826 * self._mad(abs(e - base))
        amp = self._amp(e) - base
        if amp < 1e-9:
            amp = 1e-9
        self.baseline, self.sigma, self.amplitude = base, sigma, amp

        # 7. hybrid threshold: sigma keeps weak signals detectable,
        #    amplitude fraction stops flooding when sigma collapses
        hi = base + cfg["k_hi"] * sigma
        hi2 = base + cfg["frac_hi"] * amp
        self.hi = hi if hi > hi2 else hi2
        lo = base + cfg["k_lo"] * sigma
        lo2 = base + cfg["frac_lo"] * amp
        self.lo = lo if lo > lo2 else lo2

        # 8. Schmitt trigger with confirm / release counts
        if self._n < self._warm_n:
            return False
        if not self._state:
            self._up = self._up + 1 if e > self.hi else 0
            if self._up >= self._on_n:
                self._state = True
                self._dn = 0
        else:
            self._dn = self._dn + 1 if e <= self.lo else 0
            if self._dn >= self._off_n:
                self._state = False
                self._up = 0
        return self._state

    # -- convenience --------------------------------------------------------
    def reset(self):
        self.__init__(**self.cfg)

    @property
    def warm(self):
        """True once enough signal has been seen to allow firing."""
        return self._n >= self._warm_n
