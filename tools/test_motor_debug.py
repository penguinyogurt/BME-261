"""Self-check for the motor jog server + the receiver's command parser.

    python tools/test_motor_debug.py

Runs the real HTTP server against a fake serial port, and a Python model of the
receiver's pollSerial()/deadman logic against the same bytes. Nothing here
touches hardware.
"""
import json
import sys
import threading
import time
import urllib.request
from http.server import ThreadingHTTPServer

sys.path.insert(0, __import__("os").path.dirname(__file__))


class FakeSerial:
    """Just enough pyserial to run the server."""

    def __init__(self):
        self.written = bytearray()
        self.is_open = True
        self.in_waiting = 0

    def write(self, b):
        self.written += b
        return len(b)

    def read(self, n):
        return b""

    def reset_input_buffer(self):
        pass

    def close(self):
        self.is_open = False


# ---- model of the receiver's parser + deadman (firmware/servo-receiver) ----
MANUAL_HOLD_MS = 400
DEADBAND, SPEED_SPAN, STOP_US = 20, 400, 1500


class Receiver:
    def __init__(self):
        self.buf = ""
        self.bad = False
        self.intent = 0
        self.until = 0.0

    def feed(self, data, now):
        for c in data.decode():
            if c in "\n\r":
                if not self.bad and len(self.buf) >= 2 and self.buf[0] == "m":
                    try:
                        self.intent = max(-1000, min(1000, int(self.buf[1:])))
                        self.until = now + MANUAL_HOLD_MS / 1000.0
                    except ValueError:
                        pass          # non-numeric payload: ignore, stay safe
                self.buf = ""
                self.bad = False
            elif self.bad:
                pass                  # poisoned line: discard up to the newline
            elif len(self.buf) < 15:
                self.buf += c
            else:
                self.bad = True
        return self

    def us(self, now):
        if now >= self.until:                       # deadman expired
            return STOP_US
        if abs(self.intent) < DEADBAND:
            return STOP_US
        return STOP_US + int(self.intent * SPEED_SPAN / 1000)


def main():
    import motor_debug as M

    M.ser = FakeSerial()
    srv = ThreadingHTTPServer(("127.0.0.1", 0), M.Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"

    get = lambda p: urllib.request.urlopen(base + p, timeout=3).read()

    assert b"servo jog" in get("/"), "page must render"

    r = json.loads(get("/cmd?v=600"))
    assert r["ok"] and r["v"] == 600
    assert M.ser.written.endswith(b"m600\n"), M.ser.written

    json.loads(get("/cmd?v=-600")); assert M.ser.written.endswith(b"m-600\n")
    json.loads(get("/cmd?v=0"));    assert M.ser.written.endswith(b"m0\n")

    # clamping and junk must never produce an out-of-range command
    assert json.loads(get("/cmd?v=99999"))["v"] == 1000
    assert json.loads(get("/cmd?v=-99999"))["v"] == -1000
    assert json.loads(get("/cmd?v=abc"))["v"] == 0
    assert json.loads(get("/cmd"))["v"] == 0

    t = json.loads(get("/telemetry"))
    assert t["open"] is True and isinstance(t["lines"], list)

    # ---- receiver-side: the deadman is what actually keeps this safe ----
    rx = Receiver()
    t0 = 1000.0
    rx.feed(b"m600\n", t0)
    assert rx.us(t0) > STOP_US, "command must drive"
    assert rx.us(t0 + 0.399) > STOP_US, "still driving inside the hold window"
    assert rx.us(t0 + 0.401) == STOP_US, "DEADMAN: must stop 400 ms after silence"

    # a held button: repeats at 120 ms keep it alive continuously
    now = t0
    for _ in range(20):
        rx.feed(b"m600\n", now)
        now += 0.120
        assert rx.us(now) > STOP_US, "held button must not stutter"
    assert rx.us(now + 0.5) == STOP_US, "release -> stops"

    # garbage from a serial monitor must never move the motor
    rx2 = Receiver()
    for junk in (b"hello\n", b"\n", b"m\n", b"xyz600\n", b"600\n", b"m\n\n"):
        rx2.feed(junk, t0)
    assert rx2.us(t0) == STOP_US, "junk must not drive the motor"

    # overlong garbage must not wrap into a valid command
    rx3 = Receiver()
    rx3.feed(b"m" + b"9" * 40 + b"\n", t0)
    assert rx3.us(t0) == STOP_US, "overlong payload must be dropped"

    # ...and must not RESYNC mid-line into one either
    rx4 = Receiver()
    rx4.feed(b"x" * 20 + b"m600\n", t0)
    assert rx4.us(t0) == STOP_US, "a poisoned line must be discarded entirely"
    rx4.feed(b"m600\n", t0)          # but the NEXT line still works
    assert rx4.us(t0) > STOP_US, "recovery after a poisoned line"

    srv.shutdown()
    print("ok")


if __name__ == "__main__":
    main()
