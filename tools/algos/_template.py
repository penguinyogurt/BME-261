"""Template for a causal intent detector. Copy me to tools/algos/<yourname>.py.

Files starting with "_" are NOT loaded by replay_intent.py.

THE ONE HARD RULE: update() sees one sample at a time, in arrival order, and
never sees the future. Anything a real ESP32 could not compute at that instant
is cheating — no file-wide percentiles, no sorting the whole signal, no second
pass, no "calibrate on the max of the trial".

What the data actually looks like (data/emg_2026-07-23_*.csv, chB only —
chA is dead on this hardware and rests near 0/1700 depending on the run):
  * chB rests around 1800-2100 counts and is ALREADY an analog peak-detector
    envelope, so it does not need a second envelope on top, just de-jitter.
  * strong contractions SATURATE at 4095. Several trials sit pinned there for
    tens of seconds — anything that scales off "peak seen so far" dies here.
  * the resting floor DRIFTS tens to hundreds of counts over a few minutes.
  * chB drops to literally 0 for long stretches when the front-end browns out.
    The shared FSM already forces OPEN below 300 counts, so you do not need to
    handle it — but do not let those zeros poison your baseline estimate.
  * trials run 7 s to 400 s; one (134728) is all zeros and must produce nothing.
"""
NAME = "template"
DOC = "one line describing the method — shows on the plot and in --list"


class Detector:
    def __init__(self, fs=500):
        self.fs = fs
        self.base = None        # running estimate of the resting floor
        self.act = 0.0          # de-jittered activity

    def update(self, chB, chA):
        """Return (activity, t_on, t_off, t_full) for this one sample.

        activity is whatever your method measures; the three thresholds are in
        the SAME units. t_off < t_on gives hysteresis; t_full is where grip
        force saturates at max. They may move over time — they just cannot be
        derived from samples that have not arrived yet.
        """
        if chB < 300:                       # brown-out: hold state, learn nothing
            return 0.0, 1.0, 0.5, 2.0
        if self.base is None:
            self.base = float(chB)
        # asymmetric tracking: follow the floor down fast, up very slowly, so a
        # sustained contraction does not get absorbed into the baseline
        self.base += (0.02 if chB < self.base else 0.0002) * (chB - self.base)
        a = max(0.0, chB - self.base)
        self.act += 0.04 * (a - self.act)   # ~50 ms
        return self.act, 750.0, 350.0, 1750.0
