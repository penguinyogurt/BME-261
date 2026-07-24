/*
 * EMG collector (ESP-NOW sender)
 * ------------------------------------------------------------
 * Samples the analog EMG front-end, turns it into a grip "intent", and radios
 * that intent to the servo board (7C:87:CE:30:FA:88) at 50 Hz. The servo board
 * owns all safety (travel clamp, watchdog, stall stop) - we only send intent.
 *
 * THRESHOLDING
 * chB's resting floor was measured moving only 83 counts over 95 s, and chB is
 * already a peak-detector envelope. So there is no adaptive baseline and no
 * rest/MVC trial: tare the floor ONCE, then use two fixed thresholds above it
 * with hysteresis. Nothing tracks the signal during use, so a grip can be held
 * for any length of time without being chased.
 *
 *   floor  = 10th percentile of a 200-sample tare   (boot, or 't')
 *   T_ON   = floor + ON_OFF    -> start closing; speed scales up to FULL_OFF
 *   T_OFF  = floor + OFF_OFF   -> release to HOLDING (hysteresis gap)
 *
 * Serial commands (single character):
 *   p : plot stream  -> "chA,chB,activity,intent" per sample
 *   i : intent stream-> "activity,intent" at 50 Hz
 *   t : re-tare the floor + bias / clipping / ground checks
 *   ca / cb : pick which channel drives intent (A=raw bipolar, B=envelope)
 *
 * Wiring:
 *   raw bandpass out ---> GPIO34 (ADC1_CH6, input-only)
 *   peak detector out --> GPIO35 (ADC1_CH7, input-only)
 *   circuit GND -------> ESP32 GND   (shared ground is mandatory)
 * ADC1 only: ADC2 is unusable while WiFi is on, and WiFi stays on for ESP-NOW.
 *
 * WARNING: never feed more than 3.3 V into a GPIO. Confirm with a multimeter
 * that both outputs stay within 0 - 3.3 V before wiring.
 *
 * Thresholds validated over the captures in data/ by tools/sim_simple.py.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ---- Pins / sampling ----
const int      PIN_RAW    = 34;   // raw bandpass (biased to ~1.65 V)
const int      PIN_ENV    = 35;   // peak detector envelope
const uint32_t SAMPLE_HZ  = 500;
const uint32_t SAMPLE_US  = 1000000UL / SAMPLE_HZ;
const int      OVERSAMPLE = 4;    // average N reads to cut noise
const int      ADC_MAX    = 4095; // 12-bit

// ---- Detection: DUAL-RATE RATIO (tools/algos/dualrate.py) ----
// Activity is the RATIO of a fast envelope to a slow one, not an offset above a
// tared floor. The ratio is dimensionless, so the thresholds are pure numbers
// and the resting floor may drift underneath them without recalibration —
// numerator and denominator move together. There is no tare step at all.
//
// Chosen by scoring 7 causal methods against an offline answer key
// (tools/score_intent.py). dualrate: F1 88 on held-out trials, 5% missed
// bursts, 86% quiet, and the only method that IMPROVED from fit to test.
// Brute-forced absolute thresholds scored similarly on fit but collapsed to
// 51% quiet on held-out data — on trial 171239 the resting floor steps up
// ~450 counts mid-recording and a fixed threshold ends up BELOW it, latching
// the hand shut for the whole run. That failure is what the ratio prevents.
const float T_ON_R   = 1.16f;   // ratio above which we start closing
const float T_OFF_R  = 1.075f;  // ...and below which we release (hysteresis)
const float T_FULL_R = 1.90f;   // full grip force at this ratio
const float FAST_MS      = 40.0f;   // fast envelope; lag here is response latency
const float SLOW_DOWN_S  = 0.8f;    // floor falls fast (release / post-brown-out)
const float SLOW_UP_S    = 6.0f;    // ...and rises slowly, so a grip isn't absorbed
const float FREEZE_R     = 1.22f;   // above this ratio the floor stops learning
const float FREEZE_S     = 6.0f;    // ...but not forever: a real floor step must win
const float SETTLE_S     = 2.5f;    // blind window while the peak detector recharges
const float WARM_S       = 0.5f;    // floor = running mean this long after power-on
const float REST_LO = 1500.0f, REST_HI = 2600.0f;  // front end's physical idle band
const int  DEAD_ABS = 300;          // reading below this = front-end unpowered

// ---- Intent ----
const uint8_t  SERVO_MAC[6]  = {0x7C, 0x87, 0xCE, 0x30, 0xFA, 0x88};
const uint32_t TICK_DIV      = 10;     // 500 Hz / 10 = 50 Hz intent + radio rate
const uint32_t TICK_MS       = 1000 / (SAMPLE_HZ / TICK_DIV);
const uint32_t DEBOUNCE_MS   = 80;     // sustained flex needed to enter CLOSING
const uint32_t RELAX_MS      = 150;    // sustained relax in CLOSING -> latch
const uint32_t OPEN_DWELL_MS = 2000;   // full relax this long -> hand opens
const int      OPEN_SPEED    = 600;    // intent magnitude while opening
const int      CLOSE_MIN     = 250;    // weakest close intent once engaged

// Opening used to run for a FIXED OPEN_DRIVE_MS (2500 ms). Closing has no time
// limit - you close for as long as you flex - so the two never balanced: at
// intent 1000 for 2 s the hand reaches 0.99 of travel and a full open sequence
// left it at 0.56, i.e. still 56% closed. Longer or harder closes were worse.
//
// So the open is now driven by TRAVEL, not time. gripEst mirrors the receiver's
// dead-reckoned gripPos using the same formula, and OPENING continues until it
// unwinds to zero. FULL_TRAVEL_S MUST MATCH the receiver's FULL_TRAVEL_S.
// The mirror must also model the receiver's SLEW LIMIT. Reversing from full
// close to full open takes ~53 ticks to ramp through neutral, and the hand
// keeps closing for part of that. Integrating raw intent instead ran the
// estimate to zero ~1 s early and still left the hand 16-30% closed.
// ponytail: open-loop mirror of an open-loop estimate. Replace both with a real
// reference (limit switch / current sense) when one exists. Assumes the hand
// starts OPEN at power-on.
const float    FULL_TRAVEL_S   = 1.7f;  // open->closed at full command
const float    SERVO_SLEW_TICK = 0.03f; // = SLEW_US/SPEED_SPAN on the receiver
const int      SERVO_DEADBAND  = 20;    // MUST MATCH the receiver's DEADBAND
// The hand cannot close past its end stop, so the estimate must not either.
// Without this an 8 s hard flex integrates to 4.5x full travel and the unwind
// needs 25 s - the servo was stalled against the stop for most of it, moving
// nothing. Capped, a full open is always ~3.4 s. MUST MATCH the receiver.
const float    GRIP_MAX        = 1.15f; // slack over 1.0 for FULL_TRAVEL_S error
const uint32_t OPEN_MAX_MS     = 8000;  // safety cap if the estimate runs away
float          gripEst         = 0.0f;  // 0.0 = open, 1.0 = nominal full close
float          cmdFrac         = 0.0f;  // slew-limited speed the servo is at

enum IntentState : uint8_t { ST_OPEN = 0, ST_CLOSING, ST_HOLDING, ST_OPENING };
const char *STATE_NAMES[] = {"OPEN", "CLOSING", "HOLDING", "OPENING"};

typedef struct __attribute__((packed)) {
  uint32_t seq;
  int16_t  intent;      // -1000 open .. 0 hold .. +1000 close
  uint8_t  state;       // IntentState
  uint8_t  ready;       // 0 -> servo board must stay neutral
} IntentPacket;

char        mode = 'p';
uint32_t    nextSampleUs = 0;
int         srcPin  = PIN_RAW;          // channel driving intent
char        srcName = 'a';
float       sig = 1.0f;                 // activity = fast/slow ratio, 1.0 = rest
float       envFast = 0.0f, envSlow = 0.0f;   // the two envelopes
bool        envValid = false;           // envelopes seeded on a live sample
uint32_t    heldN = 0, warmN = 0;       // freeze / warm-up counters
int         settleN = 0;                // samples left in the post-brown-out blind
float       aFast = 0.0f, aSlowDn = 0.0f, aSlowUp = 0.0f;   // EMA coefficients
bool        ready = false;              // signal alive
IntentState ist = ST_OPEN;
int         lastIntent = 0;
uint32_t    tickCount = 0, txSeq = 0, txFails = 0, lastFailReportMs = 0;
uint32_t    aboveOnMs = 0, belowOffMs = 0, openStartMs = 0;

int readAvg(int pin) {
  long acc = 0;
  for (int i = 0; i < OVERSAMPLE; i++) acc += analogRead(pin);
  return (int)(acc / OVERSAMPLE);
}

static float clampRest(float v) {
  return v < REST_LO ? REST_LO : (v > REST_HI ? REST_HI : v);
}

// EMA coefficients are 1/(tau_in_samples), matching dualrate.py exactly.
void resetDetector() {
  aFast   = 1.0f / max(1.0f, FAST_MS / 1000.0f * SAMPLE_HZ);
  aSlowDn = 1.0f / max(1.0f, SLOW_DOWN_S * SAMPLE_HZ);
  aSlowUp = 1.0f / max(1.0f, SLOW_UP_S * SAMPLE_HZ);
  envValid = false; heldN = warmN = 0; settleN = 0; sig = 1.0f;
}

// Activity = fastEnvelope / slowEnvelope. Returns 1.0 (= "resting") whenever
// the reading carries no usable information, so the FSM simply sees no activity.
// Mirror of Detector.update() in tools/algos/dualrate.py — keep the two in step.
float activity(int x) {
  if (x < DEAD_ABS) {                 // unpowered: learn nothing, report nothing
    envValid = false;
    heldN = 0;
    settleN = (int)(SETTLE_S * SAMPLE_HZ);   // arm the blind window for recharge
    sig = 1.0f;
    return sig;
  }
  float v = (float)x;
  if (!envValid) {                    // cold start: trust the physical idle band,
    envFast = envSlow = clampRest(v);  // so a trial that opens railed still fires
    envValid = true;
    // NB: warmN is deliberately NOT reset here. The warm-up runs once per boot,
    // not once per brown-out — after a brown-out the SETTLE window already pins
    // the floor to the present value, which is the same protection. Resetting it
    // here diverged from the validated reference on all three heavy-dropout
    // trials (190832, 195519, 202105).
    sig = 1.0f;
    return sig;
  }
  envFast += aFast * (v - envFast);
  if (settleN > 0) {                  // peak detector still recharging: neither a
    settleN--;                        // contraction nor a usable floor, so pin the
    envSlow = envFast;                // floor to now and emerge already correct
    sig = 1.0f;
    return sig;
  }
  envSlow = clampRest(envSlow);
  sig = envFast / max(envSlow, 1.0f);

  // Warm-up: for WARM_S the floor is just a running mean. Seeding it from a
  // single sample let one low reading at power-on hold the ratio at 1.29 and
  // close the hand for two seconds (seen on trial 195519).
  warmN++;
  bool warm = warmN <= (uint32_t)(WARM_S * SAMPLE_HZ);

  // Freeze the floor while clearly contracting, so a long grip is not absorbed
  // into it — but only for FREEZE_S. Once that expires the floor tracks again
  // and KEEPS tracking until the ratio falls back, otherwise a genuine step in
  // the resting level (trial 171239) holds the freeze on forever.
  heldN = (sig > FREEZE_R && !warm) ? heldN + 1 : 0;
  if (heldN == 0 || heldN > (uint32_t)(FREEZE_S * SAMPLE_HZ)) {
    float a = (envFast < envSlow) ? aSlowDn : aSlowUp;
    if (warm) a = max(a, 1.0f / (float)warmN);
    envSlow += a * (envFast - envSlow);
  }
  return sig;
}

void selfTest() {
  Serial.println(F("\n== self-test / tare (hold muscle relaxed) =="));
  long sA = 0, sB = 0;
  int mnA = ADC_MAX, mxA = 0, mnB = ADC_MAX, mxB = 0;
  const int N = 200;
  for (int i = 0; i < N; i++) {
    int a = readAvg(PIN_RAW), b = readAvg(PIN_ENV);
    sA += a; sB += b;
    if (a < mnA) mnA = a;  if (a > mxA) mxA = a;
    if (b < mnB) mnB = b;  if (b > mxB) mxB = b;
    delay(2);
  }
  int meanA = sA / N, meanB = sB / N;
  Serial.printf("ch A (raw): mean=%d (%d mV)  range[%d..%d]  ripple=%d\n",
                meanA, analogReadMilliVolts(PIN_RAW), mnA, mxA, mxA - mnA);
  Serial.printf("ch B (env): mean=%d (%d mV)  range[%d..%d]  ripple=%d\n",
                meanB, analogReadMilliVolts(PIN_ENV), mnB, mxB, mxB - mnB);
  if (meanA < 1500 || meanA > 2600)
    Serial.println(F("  ! ch A resting level is far from mid-rail (~2048)."));
  if (mnA <= 2 || mxA >= ADC_MAX - 2 || mnB <= 2 || mxB >= ADC_MAX - 2)
    Serial.println(F("  ! clipping at a rail - signal may exceed 0-3.3 V."));
  if ((mxA - mnA) > 800)
    Serial.println(F("  ! ch A very noisy at rest - check ground / floating input."));

  // No tare: the dual-rate ratio learns its own floor and tracks it while
  // running, so there is nothing to calibrate and nothing to hold still for.
  resetDetector();
  ist = ST_OPEN; aboveOnMs = belowOffMs = 0;
  gripEst = 0.0f; cmdFrac = 0.0f;
  ready = (meanB > DEAD_ABS) || (srcPin == PIN_RAW);
  Serial.printf("  ch %c dual-rate ratio: T_on=%.3f T_off=%.3f full=%.3f%s\n\n",
                srcName - 32, T_ON_R, T_OFF_R, T_FULL_R,
                ready ? "" : "   (front-end looks unpowered)");
}

void setSource(char ch) {
  if (ch == srcName) { Serial.printf("# already ch %c\n", ch - 32); return; }
  srcName = ch;
  srcPin  = (ch == 'a') ? PIN_RAW : PIN_ENV;
  Serial.printf("# intent source -> ch %c (%s), re-taring\n", ch - 32,
                ch == 'a' ? "raw bandpass" : "analog envelope");
  selfTest();
}

void onEspNowSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) txFails++;
}

void initEspNow() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println(F("! esp_now_init failed - intent radio disabled"));
    return;
  }
  esp_now_register_send_cb(onEspNowSent);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, SERVO_MAC, 6);
  peer.channel = 0;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK)
    Serial.println(F("! esp_now_add_peer failed - intent radio disabled"));
}

// 50 Hz: two thresholds with hysteresis, latching, then radio the intent.
void intentTick(int raw) {
  int intent = 0;
  bool alive = ready && raw > DEAD_ABS;      // unplugged/unpowered -> stay safe

  if (alive) {
    aboveOnMs  = (sig > T_ON_R)  ? aboveOnMs + TICK_MS : 0;
    belowOffMs = (sig < T_OFF_R) ? belowOffMs + TICK_MS : 0;

    IntentState prev = ist;
    switch (ist) {
      case ST_OPEN:
        if (aboveOnMs >= DEBOUNCE_MS) ist = ST_CLOSING;
        break;
      case ST_CLOSING: {
        float frac = constrain((sig - T_ON_R) / (T_FULL_R - T_ON_R), 0.0f, 1.0f);
        intent = CLOSE_MIN + (int)(frac * (1000 - CLOSE_MIN));
        if (belowOffMs >= RELAX_MS) ist = ST_HOLDING;      // latch the grip
        break;
      }
      case ST_HOLDING:
        if (aboveOnMs >= DEBOUNCE_MS) ist = ST_CLOSING;    // re-grip
        else if (belowOffMs >= OPEN_DWELL_MS) {            // let go
          ist = ST_OPENING; openStartMs = millis();
        }
        break;
      case ST_OPENING:
        intent = -OPEN_SPEED;
        if (aboveOnMs >= DEBOUNCE_MS) ist = ST_CLOSING;
        // keep driving until the travel we actually commanded is unwound, not
        // for a fixed time, or a hard close never fully opens again
        else if (gripEst <= 0.0f ||
                 millis() - openStartMs >= OPEN_MAX_MS) ist = ST_OPEN;
        break;
    }
    if (ist != prev)   // no comma, so the Python viewer's parser skips it
      Serial.printf("# intent state -> %s\n", STATE_NAMES[ist]);
  } else {
    ist = ST_OPEN; aboveOnMs = belowOffMs = 0;
    gripEst = 0.0f; cmdFrac = 0.0f;   // front end dead: receiver holds neutral,
                                      // the hand is not moving, estimate stale
  }

  // Mirror the receiver: slew toward the commanded speed, then integrate the
  // speed actually reached. Only the open end is bounded - closing further is
  // deliberate, and the matching open must be able to unwind all of it.
  float want = (abs(intent) < SERVO_DEADBAND) ? 0.0f : intent / 1000.0f;
  if      (cmdFrac < want) cmdFrac = min(cmdFrac + SERVO_SLEW_TICK, want);
  else if (cmdFrac > want) cmdFrac = max(cmdFrac - SERVO_SLEW_TICK, want);
  if (gripEst <= 0.0f && cmdFrac < 0.0f) cmdFrac = 0.0f;   // receiver's open clamp
  gripEst += cmdFrac * (TICK_MS / 1000.0f) / FULL_TRAVEL_S;
  if (gripEst < 0.0f)     gripEst = 0.0f;
  if (gripEst > GRIP_MAX) gripEst = GRIP_MAX;   // cannot close past the end stop

  lastIntent = intent;
  IntentPacket p = {++txSeq, (int16_t)intent, (uint8_t)ist, (uint8_t)alive};
  esp_now_send(SERVO_MAC, (const uint8_t *)&p, sizeof p);

  if (txFails > 0 && millis() - lastFailReportMs > 5000) {
    Serial.printf("# espnow: %lu sends unacked (servo board off?)\n",
                  (unsigned long)txFails);
    txFails = 0; lastFailReportMs = millis();
  }

  if (mode == 'i') {
    Serial.print((int)sig); Serial.print(','); Serial.println(intent);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_RAW, ADC_11db);   // ~0 - 3.3 V input range
  analogSetPinAttenuation(PIN_ENV, ADC_11db);

  Serial.println(F("\nEMG collector ready."));
  Serial.println(F("cmds: p=plot  i=intent  t=self-test  ca/cb=source channel"));
  initEspNow();        // WiFi up BEFORE the self-test so noise is measured under
  resetDetector();     // real operating conditions
  selfTest();
  nextSampleUs = micros();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 't')                  selfTest();
    else if (c == 'p' || c == 'i') mode = c;
    else if (c == 'c') {           // 'ca' / 'cb' select the source channel
      uint32_t t0 = millis();
      while (!Serial.available() && millis() - t0 < 50) delay(1);
      char n = Serial.available() ? Serial.peek() : 0;
      if (n == 'a' || n == 'b') { Serial.read(); setSource(n); }
    }
  }

  uint32_t now = micros();
  if ((int32_t)(now - nextSampleUs) < 0) return;
  nextSampleUs += SAMPLE_US;

  int src = readAvg(srcPin);
  activity(src);
  if (++tickCount >= TICK_DIV) {
    tickCount = 0;
    intentTick(src);
  }

  if (mode == 'p') {   // "chA,chB,activity,intent"
    int a = (srcPin == PIN_RAW) ? src : readAvg(PIN_RAW);
    int b = (srcPin == PIN_ENV) ? src : readAvg(PIN_ENV);
    Serial.print(a);        Serial.print(',');
    Serial.print(b);        Serial.print(',');
    Serial.print((int)sig); Serial.print(',');
    Serial.println(lastIntent);
  }
}
