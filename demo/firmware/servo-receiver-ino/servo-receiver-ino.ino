// ARDUINO IDE COPY of firmware/servo-receiver/src/main.cpp - byte-identical
// below this header. Same code, packaged as a sketch so it can be flashed
// without PlatformIO. Needs the ESP32Servo library (Library Manager).
// If you edit the receiver, edit main.cpp and re-copy; do not fork this.
// Servo-receiver ESP32 — ESP-NOW receiver + HSR-2645CR driver.
//
// Receives {seq, intent, state, calibrated} intent packets at 50 Hz from the
// EMG collector (30:AE:A4:D3:36:38) and turns intent into a servo pulse:
//     intent  +1000 = close at full speed
//     intent      0 = stop / hold  (neutral pulse, gearbox holds the grip)
//     intent  -1000 = open at full speed
//
// This board owns SAFETY. Even in this first version:
//   - Link watchdog: if intent packets stop arriving (LINK_TIMEOUT_MS) the
//     servo is commanded to NEUTRAL, so a dead radio can never leave the hand
//     driving or clenched.
//   - Not-calibrated guard: packets with calibrated=0 hold neutral.
//   - Slew limit: pulse width can only ramp so fast, so a jump in intent
//     can't slam the mechanism.
// TODO (next): travel-range clamp from a homing trial + current-sense stall
//   stop / re-zero. Those are the position-tracking safety layer we discussed.
//
// This board's STA MAC: 7C:87:CE:30:FA:88
//
// Wiring (servo): signal(yellow)->GPIO18, VCC(red)->SEPARATE 4.8-6V pack,
//   GND(black)->pack negative, and pack negative->ESP32 GND (shared ground).
//   Do NOT power the servo from the ESP32 3V3 pin.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>

// ---- Intent packet (MUST byte-match the EMG sender's struct) ----
typedef struct __attribute__((packed)) {
  uint32_t seq;
  int16_t  intent;      // -1000 open .. 0 hold .. +1000 close
  uint8_t  state;       // sender's IntentState (for telemetry only)
  uint8_t  calibrated;  // 0 -> stay neutral
} IntentPacket;

const char *STATE_NAMES[] = {"OPEN", "CLOSING", "HOLDING", "OPENING", "?"};

// ---- Servo ----
Servo drive;
const int SERVO_PIN  = 18;
const int STOP_US    = 1500;
const int MIN_US     = 900;
const int MAX_US     = 2100;
// Map intent +/-1000 to +/-SPEED_SPAN around neutral. 400 caps the servo at
// 1100..1900 us - about 2/3 of its rated speed (~39 RPM at 6 V). The servo's
// own limit is 600 (900..2100 us).
const int SPEED_SPAN = 400;
// Closing drives 1.5x faster than opening: a grip should feel immediate, while
// opening stays gentle. 400 * 1.5 = 600 us is exactly the servo's rated limit
// (900..2100), so this is the most it can take without clipping.
// SPEED_SPAN remains the REFERENCE span the gripDrive integrator is expressed
// in, so a boosted close correctly advances gripDrive 1.5x faster — it really
// is covering travel 1.5x faster.
// MUST MATCH CLOSE_BOOST in firmware/emg-collector: the sender mirrors this
// math to decide when to stop OPENING, and if the two disagree the hand stops
// short of fully open.
const float CLOSE_BOOST = 1.5f;
const int CLOSE_SIGN = +1;    // +1: positive intent -> pulses above neutral.
                              // Flip to -1 if "close" turns out to open.
const int DEADBAND   = 20;    // |intent| below this = true stop
const int SLEW_US    = 12;    // max pulse-width change per control tick

// ---- Link watchdog ----
const uint32_t LINK_TIMEOUT_MS = 300;   // 15 missed 50 Hz packets
const uint32_t CONTROL_MS      = 20;    // servo update period (50 Hz)

// ---- Manual override (tools/motor_debug.py over USB serial) ----
// Command: "m<intent>\n", intent -1000..1000, e.g. "m600" / "m-600" / "m0".
// DEADMAN: every command only buys MANUAL_HOLD_MS of authority. The web page
// re-sends while a button is held, so a closed tab, a crashed browser, or an
// unplugged cable all stop the motor within 400 ms on their own. Manual never
// needs an explicit "release" to be safe — silence IS the safe state.
const uint32_t MANUAL_HOLD_MS = 400;
int16_t  manualIntent  = 0;
uint32_t manualUntilMs = 0;
char     cmdBuf[16];
uint8_t  cmdLen = 0;
bool     cmdBad = false;      // current line overflowed -> ignore it entirely

// ---- Travel limit ----
// gripDrive is measured in SECONDS OF FULL-SPEED-EQUIVALENT DRIVE, not as a
// normalised 0..1 fraction. Driving at half speed for 2 s counts the same as
// full speed for 1 s, and an open simply has to give back what a close put in.
// Normalising by a "full travel" constant used to sit in this integrator, but
// it is a constant divisor that cancels out of the unwind completely — it only
// ever scaled the grip% readout, while making the units hard to reason about.
//
// TRAVEL_CLAMP_S is the ONE physical constant that matters: how many seconds of
// full-speed drive the mechanism absorbs going fully open -> fully closed. It
// exists because of STALL — once the hand is against its end stop the servo is
// still commanded but no longer moves, and counting that phantom drive would
// make the matching open take absurdly long unwinding motion that never
// happened. Without the clamp the unwind is exact but unbounded; with it set
// too LOW the estimate stops counting travel the hand is still making, and a
// close can no longer be fully undone.
//
// MEASURED on the real mechanism: driving CLOSE at full manual speed takes 24 s
// to reach the closed stop. Closing runs at cmdFrac 1.5 (CLOSE_BOOST), so that
// is 24 * 1.5 = 36 drive-units of full travel.
//
// The old value here was 1.955 — a guess, and 18x too small. That is what made
// the hand creep shut: a 2 s grip adds 12% of travel, the estimate stopped
// counting long before, and the matching open gave back only a fraction.
//
// MUST MATCH the sender's TRAVEL_CLAMP_S. Re-measure if the mechanism, the
// gearing or the servo supply voltage changes.
const float TRAVEL_CLAMP_S   = 36.0f;   // drive-units for full open -> closed
const float TRAVEL_NOMINAL_S = 36.0f;   // grip% readout scaling (same value, so
                                        // 100% really means fully closed)

// ---- Shared state (written in ISR-like callback, read in loop) ----
volatile int16_t  rxIntent     = 0;
volatile uint8_t  rxState      = 0;
volatile bool     rxCalibrated = false;
volatile uint32_t rxSeq        = 0;
volatile uint32_t rxCount      = 0;
volatile uint32_t lastRxMs     = 0;

int   curUs        = STOP_US;   // last pulse width actually commanded
uint32_t nextCtrlMs = 0, nextLogMs = 0, lastLogCount = 0;

// Net closing drive commanded so far, in seconds of full-speed equivalent.
// 0 = fully open. Bounded at both ends: at 0 so an open cannot wind past full
// open, and at TRAVEL_CLAMP_S because the hand cannot close past its end stop
// (see the measured value above).
// ponytail: open-loop estimate, drifts with battery sag and load. Holding the
// mechanism against a hard stop stalls the servo AND inflates this estimate.
// Re-zero against a real reference (current-sense at the end stop, or a limit
// switch) when that exists; assumes the hand starts OPEN at power-on.
float gripDrive = 0.0f;

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != (int)sizeof(IntentPacket)) return;   // ignore foreign traffic
  IntentPacket p;
  memcpy(&p, data, sizeof p);
  rxIntent     = p.intent;
  rxState      = p.state;
  rxCalibrated = (p.calibrated != 0);
  rxSeq        = p.seq;
  rxCount++;
  lastRxMs     = millis();
}

int intentToUs(int intent) {
  if (abs(intent) < DEADBAND) return STOP_US;
  // intent > 0 is CLOSING (see the packet doc at the top), independent of
  // CLOSE_SIGN, which only decides which way that maps onto pulse width.
  long span = (intent > 0) ? (long)(SPEED_SPAN * CLOSE_BOOST) : SPEED_SPAN;
  int off = (int)((long)intent * span / 1000) * CLOSE_SIGN;
  return constrain(STOP_US + off, MIN_US, MAX_US);
}

// Parse "m<intent>\n" from USB serial. Anything else is ignored, so stray
// characters from a serial monitor cannot move the motor.
void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (!cmdBad && cmdLen >= 2 && cmdBuf[0] == 'm') {
        cmdBuf[cmdLen] = '\0';
        manualIntent  = constrain(atoi(cmdBuf + 1), -1000, 1000);
        manualUntilMs = millis() + MANUAL_HOLD_MS;
      }
      cmdLen = 0; cmdBad = false;
    } else if (cmdBad) {
      // discard the REST of a poisoned line. Resetting cmdLen instead would let
      // a long garbage line resync mid-stream: "xxxxxxxxxxxxxxxm600\n" would
      // overflow, restart the buffer at 'm', and drive the motor.
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    } else {
      cmdBad = true;          // overlong: ignore everything up to the newline
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  WiFi.mode(WIFI_STA);

  ESP32PWM::allocateTimer(0);
  drive.setPeriodHertz(50);
  drive.attach(SERVO_PIN, MIN_US, MAX_US);
  drive.writeMicroseconds(STOP_US);   // start stopped

  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  Serial.println();
  Serial.println("=== Servo receiver (ESP-NOW intent) ===");
  Serial.printf("This board's STA MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  if (esp_now_init() != ESP_OK) {
    Serial.println("FATAL: esp_now_init failed");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onRecv);
  Serial.println("Waiting for intent packets from the EMG board...");
}

void loop() {
  pollSerial();

  uint32_t now = millis();
  if ((int32_t)(now - nextCtrlMs) < 0) return;
  nextCtrlMs = now + CONTROL_MS;

  bool linkUp = (rxCount > 0) && (now - lastRxMs <= LINK_TIMEOUT_MS);
  bool manual = (int32_t)(now - manualUntilMs) < 0;

  // Decide target pulse width, safest-first.
  int targetUs;
  if (manual)                        targetUs = intentToUs(manualIntent);
  else if (!linkUp || !rxCalibrated) targetUs = STOP_US;   // failsafe / not ready
  else                               targetUs = intentToUs(rxIntent);

  // Open limit: opening can only undo the closing we actually did, so it can
  // never wind past full-open. Closing is deliberately unbounded - you control
  // how far to close.
  // MANUAL BYPASSES THIS ON PURPOSE. gripDrive is dead-reckoned and can drift; if
  // it reads 0 while the hand is actually still closed, the clamp would refuse
  // to open and there would be no way back. Manual is the recovery path, so it
  // must be able to drive open regardless. The slew limit still applies.
  if (!manual && gripDrive <= 0.0f && (targetUs - STOP_US) * CLOSE_SIGN < 0)
    targetUs = STOP_US;

  // Slew-limit toward the target so intent jumps can't slam the mechanism.
  if      (curUs < targetUs) curUs = min(curUs + SLEW_US, targetUs);
  else if (curUs > targetUs) curUs = max(curUs - SLEW_US, targetUs);
  drive.writeMicroseconds(curUs);

  // Integrate what we actually commanded (not the raw intent - slew limiting
  // means they differ) to keep the travel estimate honest.
  gripDrive += ((curUs - STOP_US) * CLOSE_SIGN / (float)SPEED_SPAN)
               * (CONTROL_MS / 1000.0f);
  if (gripDrive < 0.0f)            gripDrive = 0.0f;
  if (gripDrive > TRAVEL_CLAMP_S)  gripDrive = TRAVEL_CLAMP_S;  // at the end stop

  // Telemetry ~5 Hz so the serial monitor stays readable.
  if ((int32_t)(now - nextLogMs) >= 0) {
    uint32_t c = rxCount;
    uint32_t hz = (c - lastLogCount) * 1000 / 200;   // packets in last 200 ms
    lastLogCount = c; nextLogMs = now + 200;
    uint8_t st = rxState <= 3 ? rxState : 4;
    Serial.printf("RX #%lu seq=%lu %luHz | intent=%d state=%s cal=%d | us=%d grip=%d%% %s%s\n",
                  (unsigned long)c, (unsigned long)rxSeq, (unsigned long)hz,
                  manual ? (int)manualIntent : (int)rxIntent,
                  manual ? "MANUAL" : STATE_NAMES[st], rxCalibrated ? 1 : 0,
                  curUs, (int)(gripDrive / TRAVEL_NOMINAL_S * 100),
                  manual ? "[manual override] " : "",
                  (linkUp || manual) ? "" : "*** LINK LOST -> NEUTRAL ***");
  }
}
