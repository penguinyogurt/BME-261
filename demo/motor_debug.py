#!/usr/bin/env python3
"""Browser jog controls for the servo receiver, over USB serial.

    pip install pyserial
    python tools/motor_debug.py --port COM7      # then open http://localhost:8080

Browser -> this server -> USB serial -> receiver's "m<intent>" command. The
ESP-NOW radio is not involved at all, so nothing here can destabilise the link
between the two boards.

SAFETY MODEL: hold-to-run with a firmware deadman.
Each "m<intent>" only buys MANUAL_HOLD_MS (400 ms) of authority on the receiver.
The page re-sends every 120 ms while a button is held, so a released button, a
closed tab, a crashed browser, a yanked USB cable or a killed server all stop
the motor within 400 ms without anything having to send a stop. Silence is the
safe state. The explicit stops the page sends are belt-and-braces on top.

The receiver's slew limiter still applies, so a jump from full open to full
close cannot slam the mechanism.
"""
import argparse
import json
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import serial

ser = None
ser_lock = threading.Lock()
telemetry = deque(maxlen=40)
last_cmd = {"v": 0, "t": 0.0}

PAGE = """<!doctype html>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>servo jog</title>
<style>
  :root { color-scheme: light dark; --bg:#fcfcfb; --ink:#0b0b0b; --mute:#6b6a66;
          --line:#dededa; --card:#fff; --close:#eb6834; --open:#2a78d6; }
  @media (prefers-color-scheme: dark) {
    :root { --bg:#16161a; --ink:#f2f2ef; --mute:#a0a09a; --line:#2c2c31; --card:#1e1e23; }
  }
  * { box-sizing: border-box; }
  body { margin:0; padding:24px; background:var(--bg); color:var(--ink);
         font:15px/1.5 ui-sans-serif,system-ui,-apple-system,sans-serif; }
  .wrap { max-width:560px; margin:0 auto; }
  h1 { font-size:17px; margin:0 0 2px; letter-spacing:-0.01em; }
  p.sub { margin:0 0 20px; color:var(--mute); font-size:13px; }
  .row { display:flex; gap:10px; margin-bottom:14px; }
  button { flex:1; padding:26px 12px; font-size:15px; font-weight:600;
           border:1px solid var(--line); border-radius:12px; background:var(--card);
           color:var(--ink); cursor:pointer; user-select:none;
           -webkit-user-select:none; touch-action:none; transition:transform .04s; }
  button:active { transform:scale(0.98); }
  #open:active  { background:var(--open);  color:#fff; border-color:var(--open); }
  #close:active { background:var(--close); color:#fff; border-color:var(--close); }
  #stop { padding:16px; font-weight:700; }
  label { display:flex; align-items:center; gap:12px; font-size:13px;
          color:var(--mute); margin-bottom:18px; }
  input[type=range] { flex:1; accent-color:var(--close); }
  #spd { font-variant-numeric:tabular-nums; color:var(--ink); min-width:3.5em;
         text-align:right; font-weight:600; }
  pre { background:var(--card); border:1px solid var(--line); border-radius:12px;
        padding:12px; font-size:11.5px; line-height:1.45; height:190px;
        overflow:auto; margin:0; white-space:pre-wrap; word-break:break-all; }
  .hint { color:var(--mute); font-size:12px; margin:14px 0 0; }
  .dot { display:inline-block; width:8px; height:8px; border-radius:50%;
         background:#bbb; margin-right:6px; vertical-align:middle; }
  .dot.on { background:#3aa757; }
</style>
<div class="wrap">
  <h1><span class="dot" id="dot"></span>servo jog</h1>
  <p class="sub">Hold a button to drive. Release — or close this tab — and the
     motor stops within 400&nbsp;ms.</p>
  <label>speed
    <input type="range" id="speed" min="100" max="1000" step="50" value="500">
    <span id="spd">500</span>
  </label>
  <div class="row">
    <button id="open">&#9664;&nbsp; OPEN</button>
    <button id="close">CLOSE &nbsp;&#9654;</button>
  </div>
  <div class="row"><button id="stop">STOP</button></div>
  <pre id="log">waiting for telemetry…</pre>
  <p class="hint">Keyboard: &larr; open, &rarr; close, space stop.</p>
</div>
<script>
const speed = document.getElementById('speed'), spd = document.getElementById('spd');
speed.oninput = () => spd.textContent = speed.value;
let timer = null, active = 0;

function send(v) { fetch('/cmd?v=' + v).catch(() => {}); }

function start(dir) {
  if (active === dir) return;
  active = dir;
  stopTimer();
  const tick = () => send(dir * parseInt(speed.value, 10));
  tick();                                  // fire immediately, no lag on press
  timer = setInterval(tick, 120);          // < the 400 ms firmware deadman
}
function stopTimer() { if (timer) { clearInterval(timer); timer = null; } }
function halt() { active = 0; stopTimer(); send(0); }

function bind(el, dir) {
  el.addEventListener('pointerdown', e => { e.preventDefault(); start(dir); });
  el.addEventListener('pointerup', halt);
  el.addEventListener('pointercancel', halt);
  el.addEventListener('pointerleave', halt);
}
bind(document.getElementById('open'), -1);
bind(document.getElementById('close'), +1);
document.getElementById('stop').addEventListener('pointerdown', halt);

// Any loss of focus or visibility stops the motor. The firmware deadman would
// catch it anyway; this just makes it instant.
addEventListener('blur', halt);
addEventListener('pagehide', halt);
document.addEventListener('visibilitychange', () => { if (document.hidden) halt(); });

addEventListener('keydown', e => {
  if (e.repeat) return;
  if (e.key === 'ArrowLeft') start(-1);
  else if (e.key === 'ArrowRight') start(+1);
  else if (e.key === ' ') { e.preventDefault(); halt(); }
});
addEventListener('keyup', e => {
  if (e.key === 'ArrowLeft' || e.key === 'ArrowRight') halt();
});

const log = document.getElementById('log'), dot = document.getElementById('dot');
setInterval(async () => {
  try {
    const r = await fetch('/telemetry');
    const j = await r.json();
    dot.className = 'dot' + (j.open ? ' on' : '');
    if (j.lines.length) log.textContent = j.lines.join('\\n');
  } catch (e) { dot.className = 'dot'; }
}, 500);
</script>
"""


def reader():
    """Drain the receiver's telemetry so the port never backs up."""
    buf = b""
    while True:
        try:
            with ser_lock:
                n = ser.in_waiting
                chunk = ser.read(n) if n else b""
            if chunk:
                buf += chunk
                *lines, buf = buf.split(b"\n")
                for ln in lines:
                    s = ln.decode("ascii", "ignore").strip()
                    if s:
                        telemetry.append(s)
            else:
                time.sleep(0.02)
        except Exception:
            time.sleep(0.2)


class Handler(BaseHTTPRequestHandler):
    def _send(self, body, ctype="text/html; charset=utf-8"):
        b = body.encode() if isinstance(body, str) else body
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(b)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/":
            return self._send(PAGE)
        if u.path == "/cmd":
            try:
                v = int(parse_qs(u.query).get("v", ["0"])[0])
            except ValueError:
                v = 0
            v = max(-1000, min(1000, v))
            try:
                with ser_lock:
                    ser.write(f"m{v}\n".encode())
                last_cmd.update(v=v, t=time.time())
            except Exception as e:
                return self._send(json.dumps({"ok": False, "err": str(e)}),
                                  "application/json")
            return self._send(json.dumps({"ok": True, "v": v}), "application/json")
        if u.path == "/telemetry":
            return self._send(json.dumps({
                "lines": list(telemetry)[-12:],
                "open": bool(ser and ser.is_open),
                "last": last_cmd["v"],
            }), "application/json")
        self.send_error(404)

    def log_message(self, *a):
        pass            # the jog loop is ~8 requests/second; don't spam the console


def main():
    global ser
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="receiver's serial port, e.g. COM7")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--http", type=int, default=8080)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    time.sleep(2.0)             # ESP32 auto-resets when the port opens
    ser.reset_input_buffer()
    threading.Thread(target=reader, daemon=True).start()

    srv = ThreadingHTTPServer(("127.0.0.1", args.http), Handler)
    print(f"serial {args.port} @ {args.baud}")
    print(f"open   http://localhost:{args.http}")
    print("ctrl-c to stop")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.write(b"m0\n")      # explicit stop; the deadman covers us anyway
            ser.close()
        except Exception:
            pass
        print("\nstopped")


if __name__ == "__main__":
    main()
