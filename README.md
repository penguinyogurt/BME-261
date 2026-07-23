# BME-261

EMG-controlled servo grip. An ESP32 samples an analog EMG front-end, turns the
signal into a grip *intent*, and sends it over ESP-NOW to a second ESP32 that
drives an HSR-2645CR continuous-rotation servo.

## Layout

```
firmware/
  emg-collector/     EMG sampling + self-test + intent state machine (sender)
  servo-receiver/    ESP-NOW receiver + servo driver, PlatformIO (receiver)
  espnow-link-test/  flash to BOTH boards to verify the radio link
  hsr2645cr_drive/   standalone servo spin demo
tools/               Python: live viewer, serial monitor, offline simulators
data/                logged captures (emg_YYYY-MM-DD_*.csv) + synthetic sets
docs/                ESP32-WROOM-32 and HSR-2645CR datasheets
output/              generated plots (gitignored - regenerate from tools/)
```

CSV format is `host_time_s,chA_raw,chB_env`. **chA is a dead channel on the
current hardware - chB (the analog peak-detector envelope) carries the signal.**

## Boards

| Role | MAC | Port |
|---|---|---|
| EMG collector (sender) | `30:AE:A4:D3:36:38` | COM6 |
| Servo receiver | `7C:87:CE:30:FA:88` | COM7 |

## Common commands

```bash
# live plot + log to CSV
python tools/emg_bringup_viewer.py --port COM6

# watch both boards at once (Arduino IDE 1.x can only monitor one)
python tools/dual_serial_monitor.py COM6 COM7

# offline: replay logged data through the control logic (no hardware needed)
python tools/sim_intent.py --all        # firmware state machine, faithful port
python tools/sim_schemes.py             # compare 5 control schemes
python tools/sim_envelopes.py           # compare envelope front-ends

# regenerate synthetic test data and its plots
python tools/make_synthetic_emg.py && python tools/render_synthetic_png.py
```

Firmware builds with the Arduino IDE / `arduino-cli` (ESP32 board package);
`servo-receiver` is a PlatformIO project and also needs the `ESP32Servo` library.
