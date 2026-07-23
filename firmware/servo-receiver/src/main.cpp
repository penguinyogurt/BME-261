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
// Map intent +/-1000 to +/-SPEED_SPAN around neutral. 450 keeps it gentle
// (1050..1950 us); raise toward 600 for the servo's full speed once tuned.
const int SPEED_SPAN = 450;
const int CLOSE_SIGN = +1;    // +1: positive intent -> pulses above neutral.
                              // Flip to -1 if "close" turns out to open.
const int DEADBAND   = 20;    // |intent| below this = true stop
const int SLEW_US    = 12;    // max pulse-width change per control tick

// ---- Link watchdog ----
const uint32_t LINK_TIMEOUT_MS = 300;   // 15 missed 50 Hz packets
const uint32_t CONTROL_MS      = 20;    // servo update period (50 Hz)

// ---- Shared state (written in ISR-like callback, read in loop) ----
volatile int16_t  rxIntent     = 0;
volatile uint8_t  rxState      = 0;
volatile bool     rxCalibrated = false;
volatile uint32_t rxSeq        = 0;
volatile uint32_t rxCount      = 0;
volatile uint32_t lastRxMs     = 0;

int   curUs        = STOP_US;   // last pulse width actually commanded
uint32_t nextCtrlMs = 0, nextLogMs = 0, lastLogCount = 0;

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

  // Slew-limit toward the target so intent jumps can't slam the mechanism.
  if      (curUs < targetUs) curUs = min(curUs + SLEW_US, targetUs);
  else if (curUs > targetUs) curUs = max(curUs - SLEW_US, targetUs);
  drive.writeMicroseconds(curUs);

  // Telemetry ~5 Hz so the serial monitor stays readable.
  if ((int32_t)(now - nextLogMs) >= 0) {
    uint32_t c = rxCount;
    uint32_t hz = (c - lastLogCount) * 1000 / 200;   // packets in last 200 ms
    lastLogCount = c; nextLogMs = now + 200;
    uint8_t st = rxState <= 3 ? rxState : 4;
    Serial.printf("RX #%lu seq=%lu %luHz | intent=%d state=%s cal=%d | us=%d %s\n",
                  (unsigned long)c, (unsigned long)rxSeq, (unsigned long)hz,
                  (int)rxIntent, STATE_NAMES[st], rxCalibrated ? 1 : 0,
                  curUs, linkUp ? "" : "*** LINK LOST -> NEUTRAL ***");
  }
}
