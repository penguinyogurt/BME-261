# Demo folder — run the EMG grip from scratch

Everything needed for a live demo, copied here so you don't have to go hunting
through the repo. These are **copies**; the originals still live in `../tools/`
and `../firmware/`. If you change firmware for real, change the original.

```
demo/
  emg_bringup_viewer.py    live plot of both EMG channels + CSV log
  motor_debug.py           manual override: browser jog buttons for the servo
  dual_serial_monitor.py   watch both boards' serial at once
  firmware/
    emg-collector/         EMG board (sender)      -> COM6
    servo-receiver-ino/    servo board (receiver)  -> COM7   Arduino IDE
    servo-receiver/        same code as a PlatformIO project
    espnow-link-test/      flash to BOTH if the radio link is suspect
```

Flash `servo-receiver-ino` if you're using the Arduino IDE — it's the same code
as `servo-receiver/src/main.cpp`, just packaged as a sketch. Pick one; flashing
both is pointless. **Edits go in `main.cpp`, then re-copy** — don't let the two
drift.

| Role | MAC | Port |
|---|---|---|
| EMG collector (sender) | `30:AE:A4:D3:36:38` | COM6 |
| Servo receiver | `7C:87:CE:30:FA:88` | COM7 |

Ports are what these boards enumerate as on this machine. Check Device Manager
if they moved, and substitute your own everywhere below.

---

## 0. One-time setup

```
pip install pyserial matplotlib
```

Firmware toolchain (only if you need to re-flash):

* Arduino IDE or `arduino-cli` with the **esp32** board package — for every
  sketch here. The receiver also needs the **ESP32Servo** library: Arduino IDE
  → Tools → Manage Libraries → search "ESP32Servo" (by Kevin Harrington /
  madhephaestus) → Install.
* PlatformIO — only if you prefer it over the Arduino IDE for the receiver.
  `platformio.ini` doesn't list ESP32Servo, so install it once with
  `pio pkg install -g -l "madhephaestus/ESP32Servo"`.

---

## 1. Wire it up (do this before power)

**EMG board**

| EMG circuit | ESP32 |
|---|---|
| raw bandpass out | GPIO34 |
| peak detector out | GPIO35 |
| circuit GND | ESP32 GND (**mandatory** — shared ground) |

⚠️ GPIO34/35 are input-only 0–3.3 V pins. **Meter both outputs before wiring
them** and confirm they never exceed 3.3 V.

**Servo board**

| HSR-2645CR | goes to |
|---|---|
| signal (yellow) | GPIO18 |
| VCC (red) | separate 4.8–6 V battery pack — **never** the ESP32 3V3 pin |
| GND (black) | pack negative, and pack negative → ESP32 GND |

---

## 2. Flash the boards (skip if they're already programmed)

```
arduino-cli compile --fqbn esp32:esp32:esp32 demo/firmware/emg-collector
arduino-cli upload -p COM6 --fqbn esp32:esp32:esp32 demo/firmware/emg-collector
```

Pass the **folder**, not the `.ino`.

Servo receiver — Arduino IDE (open `servo-receiver-ino/servo-receiver-ino.ino`,
board "ESP32 Dev Module", port COM7, Upload), or from the command line:

```
arduino-cli compile --fqbn esp32:esp32:esp32 demo/firmware/servo-receiver-ino
arduino-cli upload -p COM7 --fqbn esp32:esp32:esp32 demo/firmware/servo-receiver-ino
```

PlatformIO instead, if you'd rather:

```
cd demo/firmware/servo-receiver
pio run -t upload            # upload_port COM7 is already set in platformio.ini
```

---

## 3. Power on and check the self-test

Power both boards. The EMG board runs `selfTest()` on boot and prints a verdict
over serial at **115200 baud**. Watch it with:

```
python demo/dual_serial_monitor.py COM6 COM7
```

It flags three things — fix them before demoing:

1. resting mean far from mid-rail (~2048) → bias problem,
2. min or max pinned at 0 or 4095 → clipping,
3. peak-to-peak noise at rest > 800 counts → floating pin or missing ground.

`dual_serial_monitor.py` is read-only. To *send* a command (like `t` to re-run
the self-test) use the Arduino IDE's Serial Monitor at 115200, no line ending.

There is **no calibration and no tare** —
nothing to hold still for. Detection uses a fast/slow envelope *ratio*, so a
drifting resting floor moves the thresholds with it.

Close the monitor before step 4 — only one program can hold a COM port.

---

## 4. Live viewer — `emg_bringup_viewer.py`

```
python demo/emg_bringup_viewer.py --port COM6
```

Other platforms: `--port /dev/ttyUSB0` (Linux), `--port /dev/cu.usbserial-XXXX`
(macOS). `--baud` defaults to 115200.

What it does:

1. Opens the port, waits 2 s for the ESP32's auto-reset, sends `p` to put the
   firmware in plot mode.
2. Opens a window with three stacked plots:
   * **ch A** — raw bandpass. Rests mid-rail (~1721–1789 counts on this board),
     sharp onsets, no smoothing.
   * **ch B** — peak-detector envelope. Smooth, rails at 4095 on a hard
     contraction. This is what drives the shipped intent logic.
   * **intent** — what the board is radioing. `+` close, `−` open, `0` hold.
3. Logs **every** sample to `../data/emg_YYYY-MM-DD_HHMMSS.csv`, columns
   `host_time_s,chA_raw,chB_env,env,intent`. The plot only keeps the last 1500
   samples (~3 s at 500 Hz); the CSV keeps all of it, unbounded.

**Close the plot window to stop.** That flushes and saves the CSV — the path is
printed on exit. Ctrl-C in the terminal instead can leave the last ~0.1 s
unwritten.

Quick read of a good trial: chA sits mid-rail and gets visibly fuzzier on a
contraction; chB steps up within ~130–210 ms; intent goes positive shortly
after and returns to 0 on release.

Firmware keys, typed into the Arduino IDE Serial Monitor (the viewer sends `p`
for you and has no keyboard input of its own):
`p` plot stream · `i` intent stream only · `t` re-run self-test ·
`ca`/`cb` choose which channel drives intent.

---

## 5. Manual override frontend — `motor_debug.py`

Drives the servo directly over USB, for testing the mechanism with no EMG board
and no radio involved.

```
python demo/motor_debug.py --port COM7
```

Then open **http://localhost:8080**. (`--http 9000` for a different port.)

The page gives you:

* **OPEN** / **CLOSE** — *hold* to drive, release to stop. Not click-to-toggle.
* **speed** slider, 100–1000.
* **STOP**, and keyboard: ← open, → close, space stop.
* a live telemetry pane, and a green dot when the serial port is open.

Safety model — hold-to-run with a firmware deadman. Each command buys the
receiver only **400 ms** of authority, and the page re-sends every 120 ms while
you hold. A released button, a closed tab, a crashed browser, a yanked cable or
a killed server all stop the motor within 400 ms. Silence is the safe state.
The slew limiter still applies, so open→close can't slam the mechanism.

Ctrl-C in the terminal to stop the server.

Note: this holds COM7, so it can't run at the same time as
`dual_serial_monitor.py` or PlatformIO's monitor.

---

## 6. Full demo run

1. Power both boards, electrodes on.
2. `python demo/emg_bringup_viewer.py --port COM6` — confirm the self-test
   passed and chB responds to a contraction.
3. Contract → the hand closes and holds. Relax → it opens back to zero.
   Opening runs on *travel*, not a timer, so a long close takes a long open.
4. Close the plot window. The CSV in `../data/` is your record of the run.

---

## Troubleshooting

| Symptom | Do this |
|---|---|
| `ESP32Servo.h: No such file or directory` | The library isn't installed — see step 0. |
| "could not open port" / "access denied" | Something else has the port — Arduino IDE monitor, PlatformIO monitor, another copy of the viewer. Close it. |
| Viewer window opens but the lines are flat at 0 | No data arriving. Wrong COM port, or the board didn't see the `p` — close the viewer, reopen it (it re-sends `p` after the 2 s reset delay). |
| Self-test complains about noise > 800 counts p-p | Missing shared ground between the EMG circuit and the ESP32. Check that first, before electrodes. |
| chB pinned at 4095 | Gain too high or the contraction is maxing the front-end. Usable but the envelope is saturated — note it if you keep the log. |
| Servo doesn't move on intent, but the jog page works | The radio, not the mechanism. Flash `espnow-link-test` to **both** boards and confirm packets cross. |
| Nothing moves, jog page included | Servo power. It needs its own 4.8–6 V pack with its negative tied to ESP32 GND. |
| Hand latches shut and won't open | Power-cycle the EMG board. Then check the log: a resting-floor step is the usual cause. |
