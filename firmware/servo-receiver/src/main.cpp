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
const int CLOSE_SIGN = +1;    // +1: positive intent -> pulses above neutral.
                              // Flip to -1 if "close" turns out to open.
const int DEADBAND   = 20;    // |intent| below this = true stop
const int SLEW_US    = 12;    // max pulse-width change per control tick

// ---- Link watchdog ----
const uint32_t LINK_TIMEOUT_MS = 300;   // 15 missed 50 Hz packets
const uint32_t CONTROL_MS      = 20;    // servo update period (50 Hz)

// ---- Travel limit ----
// Seconds to go fully open -> fully closed at FULL command (SPEED_SPAN). Only
// scales the grip% readout - the open limit is symmetric, so it holds at any
// value. Measure it on the real mechanism to make grip% mean something.
const float FULL_TRAVEL_S = 1.7f;

// ---- Shared state (written in ISR-like callback, read in loop) ----
volatile int16_t  rxIntent     = 0;
volatile uint8_t  rxState      = 0;
volatile bool     rxCalibrated = false;
volatile uint32_t rxSeq        = 0;
volatile uint32_t rxCount      = 0;
volatile uint32_t lastRxMs     = 0;

int   curUs        = STOP_US;   // last pulse width actually commanded
uint32_t nextCtrlMs = 0, nextLogMs = 0, lastLogCount = 0;

// Dead-reckoned grip travel: 0.0 = fully open, 1.0 = nominal full close. Only
// the open end is enforced, so opening exactly undoes the closing we did. It may
// read >1.0 if you close further - that is intended, so the matching open still
// unwinds all of it.
// ponytail: open-loop estimate, drifts with battery sag and load. Holding the
// mechanism against a hard stop stalls the servo AND inflates this estimate.
// Re-zero against a real reference (current-sense at the end stop, or a limit
// switch) when that exists; assumes the hand starts OPEN at power-on.
float gripPos = 0.0f;

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
  int off = (int)((long)intent * SPEED_SPAN / 1000) * CLOSE_SIGN;
  return constrain(STOP_US + off, MIN_US, MAX_US);
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
  uint32_t now = millis();
  if ((int32_t)(now - nextCtrlMs) < 0) return;
  nextCtrlMs = now + CONTROL_MS;

  bool linkUp = (rxCount > 0) && (now - lastRxMs <= LINK_TIMEOUT_MS);

  // Decide target pulse width, safest-first.
  int targetUs;
  if (!linkUp || !rxCalibrated) targetUs = STOP_US;   // failsafe / not ready
  else                          targetUs = intentToUs(rxIntent);

  // Open limit: opening can only undo the closing we actually did, so it can
  // never wind past full-open. Closing is deliberately unbounded - you control
  // how far to close.
  if (gripPos <= 0.0f && (targetUs - STOP_US) * CLOSE_SIGN < 0)
    targetUs = STOP_US;

  // Slew-limit toward the target so intent jumps can't slam the mechanism.
  if      (curUs < targetUs) curUs = min(curUs + SLEW_US, targetUs);
  else if (curUs > targetUs) curUs = max(curUs - SLEW_US, targetUs);
  drive.writeMicroseconds(curUs);

  // Integrate what we actually commanded (not the raw intent - slew limiting
  // means they differ) to keep the travel estimate honest.
  gripPos += ((curUs - STOP_US) * CLOSE_SIGN / (float)SPEED_SPAN)
             * (CONTROL_MS / 1000.0f) / FULL_TRAVEL_S;
  if (gripPos < 0.0f) gripPos = 0.0f;   // only the open end is bounded

  // Telemetry ~5 Hz so the serial monitor stays readable.
  if ((int32_t)(now - nextLogMs) >= 0) {
    uint32_t c = rxCount;
    uint32_t hz = (c - lastLogCount) * 1000 / 200;   // packets in last 200 ms
    lastLogCount = c; nextLogMs = now + 200;
    uint8_t st = rxState <= 3 ? rxState : 4;
    Serial.printf("RX #%lu seq=%lu %luHz | intent=%d state=%s cal=%d | us=%d grip=%d%% %s\n",
                  (unsigned long)c, (unsigned long)rxSeq, (unsigned long)hz,
                  (int)rxIntent, STATE_NAMES[st], rxCalibrated ? 1 : 0,
                  curUs, (int)(gripPos * 100),
                  linkUp ? "" : "*** LINK LOST -> NEUTRAL ***");
  }
}
