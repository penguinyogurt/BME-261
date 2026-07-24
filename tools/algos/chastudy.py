"""Adapter for the channelAstudy detector, so it can be scored on OUR answer key.

That study is channel-A based (DC-removal -> 20-200 Hz bandpass -> TKEO ->
envelope -> hybrid sigma/amplitude threshold -> Schmitt trigger) and reports
leave-one-out F1 0.763. Our numbers are NOT comparable to that figure: theirs is
BURST-level (an event is one TP/FP/FN), ours is TICK-level (every 20 ms sample
counts). Tick-level F1 runs much higher because long bursts dominate the count
and a partially-detected burst still scores well.

So this wraps their shipping detector unmodified and runs it through the same
FSM and the same scorer as everything else. Only then do the numbers mean the
same thing.

Their push() returns a BOOLEAN (in-burst or not), not a graded activity, so the
activity here is 0/1 and the thresholds sit either side of 0.5. Grip force is
therefore ungraded — this measures DETECTION only, which is what we want to
compare.
"""
import os
import sys

_STUDY = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "channelAstudy")
if _STUDY not in sys.path:
    sys.path.insert(0, _STUDY)

from emg_detector import EMGIntentDetector       # noqa: E402

NAME = "chastudy"
DOC = "channelAstudy chA detector (bandpass+TKEO+hybrid threshold), as shipped"


class Detector:
    def __init__(self, fs=500):
        self.d = EMGIntentDetector()

    def update(self, chB, chA):
        # note the argument order: our harness passes (chB, chA), theirs takes
        # (chA, chB). Getting this backwards silently scores noise.
        on = self.d.push(chA, chB)
        return (1.0 if on else 0.0), 0.5, 0.5, 1.0


if __name__ == "__main__":
    d = Detector(500)
    for _ in range(3000):
        d.update(1800, 1750)
    assert d.update(1800, 1750)[0] == 0.0, "quiet input must not fire"
    print("ok")
