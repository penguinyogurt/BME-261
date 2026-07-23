/*
 * EMG collector (ESP-NOW sender)
 * ------------------------------------------------------------
 * Samples the analog EMG front-end, turns it into a grip "intent", and radios
 * that intent to the servo board (7C:87:CE:30:FA:88) at 50 Hz. The servo board
 * owns all safety (travel clamp, watchdog, stall stop) - we only send intent.
 *
 * Serial commands (single character):
 *   p : plot stream  -> "chA,chB" per sample (Serial Plotter / Python viewer)
 *   i : intent stream-> "env,intent" at 50 Hz
 *   t : self-test    -> bias / clipping / floating-ground check, sets baseline
 *   c : calibrate    -> guided rest + max-contraction trial, saved to flash
 *   ca / cb : pick which channel drives the intent envelope (A=raw, B=envelope)
 *   cr : erase this channel's stored trial and fall back to the built-in default
 *
 * Calibration is stored PER CHANNEL (rest/mvc are in that channel's units), so
 * switching sources reloads that channel's own trial rather than junking it.
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
 * The intent state machine is mirrored in tools/sim_intent.py - keep the two in
 * sync; that script is how this logic gets exercised without hardware.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>

// ---- Pins / sampling ----
const int      PIN_RAW    = 34;   // raw bandpass (biased to ~1.65 V)
const int      PIN_ENV    = 35;   // peak detector envelope
const uint32_t SAMPLE_HZ  = 500;
const uint32_t SAMPLE_US  = 1000000UL / SAMPLE_HZ;
const int      OVERSAMPLE = 4;    // average N reads to cut noise
const int      ADC_MAX    = 4095; // 12-bit

// ---- Intent ----
const uint8_t  SERVO_MAC[6]  = {0x7C, 0x87, 0xCE, 0x30, 0xFA, 0x88};
const uint32_t TICK_DIV      = 10;     // 500 Hz / 10 = 50 Hz intent + radio rate
const uint32_t TICK_MS       = 1000 / (SAMPLE_HZ / TICK_DIV);
const float    ENV_ALPHA     = 0.01f;  // ~200 ms EMA at 500 Hz
const float    BASE_DN       = 0.02f;   // baseline falls to the resting floor (~0.1 s)
const float    BASE_UP       = 0.0002f; // and rises slowly (~10 s), so a held grip
                                        // cannot drag the baseline up with it
const float    T_LOW_FRAC    = 0.12f;  // thresholds as fraction of (MVC - rest)
const float    T_HIGH_FRAC   = 0.28f;
const float    FULL_FRAC     = 0.80f;  // intent saturates at 80% MVC
const uint32_t DEBOUNCE_MS   = 80;     // sustained flex needed to enter CLOSING
const uint32_t RELAX_MS      = 150;    // sustained relax in CLOSING -> latch
const uint32_t OPEN_DWELL_MS = 2000;   // full relax this long -> hand opens
const uint32_t OPEN_DRIVE_MS = 2500;   // how long OPEN intent is commanded
const int      OPEN_SPEED    = 600;    // intent magnitude while opening
const int      CLOSE_MIN     = 250;    // weakest close intent once engaged
const float    CAL_MIN_SPAN  = 40.0f;  // below this, calibration is meaningless

// Fallback calibration used when a channel has no stored trial. These are
// envelope counts ABOVE baseSrc (not absolute ADC levels), so they stay valid
// while the live tare tracks the DC offset. Derived from
// data/emg_2026-07-23_171239.csv and verified in tools/sim_intent.py, which
// replays this exact state machine. 'c' overwrites them per channel.
const float DEF_REST_A = 47.0f,  DEF_MVC_A = 1488.0f;
const float DEF_REST_B = 104.0f, DEF_MVC_B = 1595.0f;

enum IntentState : uint8_t { ST_OPEN = 0, ST_CLOSING, ST_HOLDING, ST_OPENING };
const char *STATE_NAMES[] = {"OPEN", "CLOSING", "HOLDING", "OPENING"};

typedef struct __attribute__((packed)) {
  uint32_t seq;
  int16_t  intent;      // -1000 open .. 0 hold .. +1000 close
  uint8_t  state;       // IntentState
  uint8_t  calibrated;  // 0 -> servo board must stay neutral
} IntentPacket;

Preferences prefs;
char        mode = 'p';
uint32_t    nextSampleUs = 0;
int         srcPin  = PIN_RAW;          // channel driving the intent envelope
char        srcName = 'a';              // 'a' or 'b', selects NVS keys too
float       baseSrc = 0;                // resting level, tracked continuously
float       env = 0.0f;                 // digital envelope (counts)
float       restEnv = 0, mvcEnv = 0;    // calibration
float       tLow = 0, tHigh = 0, full = 0;   // derived thresholds
bool        calibrated = false;
IntentState ist = ST_OPEN;
int         lastIntent = 0;             // latest intent, so plot mode can show it
uint32_t    tickCount = 0, txSeq = 0, txFails = 0, lastFailReportMs = 0;
uint32_t    aboveHighMs = 0, belowHighMs = 0, belowLowMs = 0, openStartMs = 0;

int readAvg(int pin) {
  long acc = 0;
  for (int i = 0; i < OVERSAMPLE; i++) acc += analogRead(pin);
  return (int)(acc / OVERSAMPLE);
}

// Average n samples over ~2n ms. A baseline taken from one instant is
// meaningless on a noisy channel - a single transient locks in a wrong offset
// that muscle variation can never bring back under threshold.
int meanOf(int pin, int n) {
  long acc = 0;
  for (int i = 0; i < n; i++) { acc += readAvg(pin); delay(2); }
  return (int)(acc / n);
}

// The baseline TRACKS the signal instead of being a one-shot tare: chB's DC was
// measured drifting ~400 counts in 12 s, which is far wider than T_high, so a
// fixed tare turns drift into a permanent phantom contraction and the grip never
// releases. Verified in tools/sim_baseline.py: under an 800-count drift a fixed
// baseline spends 85% of the session CLOSING, this stays at 19%.
// The two channels are different signals and need different handling.
float envUpdate(int x) {
  float r;
  if (srcPin == PIN_ENV) {
    // chB is a unipolar envelope: keep the baseline on the resting floor (fall
    // fast, rise slowly so a held grip can't drag it up), and below the floor
    // means no activity - not activity in the other direction.
    baseSrc += ((x < baseSrc) ? BASE_DN : BASE_UP) * (x - baseSrc);
    r = max(0.0f, x - baseSrc);
  } else {
    // chA is bipolar around mid-rail: the baseline is its DC bias, so track it
    // slowly and symmetrically, and rectify - both halves carry signal.
    baseSrc += BASE_UP * (x - baseSrc);
    r = fabsf(x - baseSrc);
  }
  env += ENV_ALPHA * (r - env);
  return env;
}

// Thresholds only change when calibration does, so derive them once.
void setThresholds() {
  float span = mvcEnv - restEnv;
  calibrated = span > CAL_MIN_SPAN;
  tLow  = restEnv + T_LOW_FRAC  * span;
  tHigh = restEnv + T_HIGH_FRAC * span;
  full  = restEnv + FULL_FRAC   * span;
}

void selfTest() {
  Serial.println(F("\n== self-test (hold muscle relaxed) =="));
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

  // 1) raw channel should rest near mid-rail (~2048 counts / ~1.65 V)
  if (meanA < 1500 || meanA > 2600)
    Serial.println(F("  ! ch A resting level is far from mid-rail (~2048)."));
  else
    Serial.println(F("  ok: ch A biased near mid-rail."));
  // 2) clipping against a rail = signal outside 0-3.3 V
  if (mnA <= 2 || mxA >= ADC_MAX - 2 || mnB <= 2 || mxB >= ADC_MAX - 2)
    Serial.println(F("  ! clipping at a rail - signal may exceed 0-3.3 V."));
  // 3) huge noise at rest usually means no common ground / floating pin
  if ((mxA - mnA) > 800)
    Serial.println(F("  ! ch A very noisy at rest - check ground / floating input."));

  baseSrc = (srcPin == PIN_RAW) ? meanA : meanB;
  Serial.printf("  baseline captured (intent source = ch %c).\n\n",
                srcName - 32);   // 'a' -> 'A'
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
  peer.channel = 0;            // current channel (both boards default to 1)
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK)
    Serial.println(F("! esp_now_add_peer failed - intent radio disabled"));
}

void loadCalibration() {
  char kr[8], km[8];                        // per-channel keys: restA / mvcB ...
  snprintf(kr, sizeof kr, "rest%c", srcName);
  snprintf(km, sizeof km, "mvc%c", srcName);
  bool isA = (srcName == 'a');
  prefs.begin("emg", true);
  restEnv = prefs.getFloat(kr, isA ? DEF_REST_A : DEF_REST_B);
  mvcEnv  = prefs.getFloat(km, isA ? DEF_MVC_A  : DEF_MVC_B);
  bool stored = prefs.isKey(kr);
  prefs.end();
  setThresholds();
  Serial.printf("ch %c calibration (%s): rest=%.0f mvc=%.0f | "
                "T_low=%.0f T_high=%.0f full=%.0f\n",
                srcName - 32, stored ? "from trial" : "built-in default",
                restEnv, mvcEnv, tLow, tHigh, full);
}

// Sample for `ms` keeping the envelope EMA warm. Returns the mean envelope;
// reports the max via *maxOut when non-null.
float sampleEnvWindow(uint32_t ms, float *maxOut) {
  double sum = 0; uint32_t n = 0; float mx = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    float e = envUpdate(readAvg(srcPin));
    sum += e; n++; if (e > mx) mx = e;
    delay(2);
  }
  if (maxOut) *maxOut = mx;
  return n ? (float)(sum / n) : 0.0f;
}

void countdown(const char *prompt) {
  Serial.println(prompt);
  for (int i = 3; i > 0; i--) { Serial.printf("  %d...\n", i); delay(1000); }
}

void calibrate() {
  Serial.println(F("\n== calibration =="));
  countdown("step 1/2: RELAX the muscle completely...");

  baseSrc = meanOf(srcPin, 250);        // re-tare, then measure rest
  env = 0.0f;                           // restart the EMA from the new baseline
  sampleEnvWindow(500, nullptr);        // settle
  float rest = sampleEnvWindow(2000, nullptr);

  countdown("step 2/2: now SQUEEZE as hard as you can, hold it...");
  float mvc = 0.0f;
  sampleEnvWindow(3000, &mvc);
  Serial.println(F("  ...and relax."));

  if (mvc - rest <= CAL_MIN_SPAN) {
    Serial.printf("! contraction too weak (rest=%.0f mvc=%.0f) - check "
                  "electrodes. Calibration NOT saved.\n", rest, mvc);
    return;
  }
  restEnv = rest; mvcEnv = mvc;
  setThresholds();
  char kr[8], km[8];
  snprintf(kr, sizeof kr, "rest%c", srcName);
  snprintf(km, sizeof km, "mvc%c", srcName);
  prefs.begin("emg", false);
  prefs.putFloat(kr, restEnv);
  prefs.putFloat(km, mvcEnv);
  prefs.end();
  Serial.printf("saved ch %c: rest=%.0f mvc=%.0f | T_low=%.0f T_high=%.0f full=%.0f\n\n",
                srcName - 32, restEnv, mvcEnv, tLow, tHigh, full);
}

// Switch which channel feeds the intent envelope. Drops the latch and reloads
// that channel's own calibration - rest/mvc from the other channel would be
// meaningless here, and carrying a HOLDING grip across a source change is unsafe.
void setSource(char ch) {
  if (ch == srcName) {
    Serial.printf("# intent source already ch %c\n", ch - 32);
    return;
  }
  srcName = ch;
  srcPin  = (ch == 'a') ? PIN_RAW : PIN_ENV;
  baseSrc = meanOf(srcPin, 100);        // ~200 ms re-tare, not a single instant
  env = 0.0f;                           // don't carry the old channel's EMA over
  ist = ST_OPEN;                        // drop any latched grip
  aboveHighMs = belowHighMs = belowLowMs = 0;
  Serial.printf("# intent source -> ch %c (%s)\n", ch - 32,
                ch == 'a' ? "raw bandpass" : "analog envelope");
  loadCalibration();
  Serial.println(F("# run 't' to re-baseline, 'c' if this channel is uncalibrated"));
}

// 50 Hz: run the latching state machine and radio the intent to the servo.
void intentTick() {
  int intent = 0;

  if (calibrated) {
    aboveHighMs = (env > tHigh) ? aboveHighMs + TICK_MS : 0;
    belowHighMs = (env < tHigh) ? belowHighMs + TICK_MS : 0;
    belowLowMs  = (env < tLow)  ? belowLowMs  + TICK_MS : 0;  // dwell timer; any
                                  // activity >= T_low is a refresh squeeze

    IntentState prev = ist;
    switch (ist) {
      case ST_OPEN:
        if (aboveHighMs >= DEBOUNCE_MS) ist = ST_CLOSING;
        break;
      case ST_CLOSING: {
        float frac = constrain((env - tHigh) / (full - tHigh), 0.0f, 1.0f);
        intent = CLOSE_MIN + (int)(frac * (1000 - CLOSE_MIN));
        if (belowHighMs >= RELAX_MS) ist = ST_HOLDING;        // latch the grip
        break;
      }
      case ST_HOLDING:
        if (aboveHighMs >= DEBOUNCE_MS) ist = ST_CLOSING;     // re-grip
        else if (belowLowMs >= OPEN_DWELL_MS) {               // let go
          ist = ST_OPENING; openStartMs = millis();
        }
        break;
      case ST_OPENING:
        intent = -OPEN_SPEED;
        if (aboveHighMs >= DEBOUNCE_MS) ist = ST_CLOSING;     // grab mid-open
        else if (millis() - openStartMs >= OPEN_DRIVE_MS) ist = ST_OPEN;
        break;
    }
    if (ist != prev)   // no comma, so the Python viewer's parser skips it
      Serial.printf("# intent state -> %s\n", STATE_NAMES[ist]);
  }

  lastIntent = intent;
  IntentPacket p = {++txSeq, (int16_t)intent, (uint8_t)ist, calibrated};
  esp_now_send(SERVO_MAC, (const uint8_t *)&p, sizeof p);

  if (txFails > 0 && millis() - lastFailReportMs > 5000) {
    Serial.printf("# espnow: %lu sends unacked (servo board off?)\n",
                  (unsigned long)txFails);
    txFails = 0; lastFailReportMs = millis();
  }

  if (mode == 'i') {
    Serial.print((int)env); Serial.print(','); Serial.println(intent);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_RAW, ADC_11db);   // ~0 - 3.3 V input range
  analogSetPinAttenuation(PIN_ENV, ADC_11db);

  Serial.println(F("\nEMG collector ready."));
  Serial.println(F("cmds: p=plot  i=intent  t=self-test  c=calibrate  ca/cb=source channel"));
  initEspNow();        // WiFi up BEFORE self-test so noise is measured
  loadCalibration();   // under real operating conditions
  selfTest();
  nextSampleUs = micros();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 't')                        selfTest();
    else if (c == 'p' || c == 'i')       mode = c;
    else if (c == 'c') {
      // 'ca'/'cb' pick the source channel; a bare 'c' calibrates. Serial
      // monitors send the whole line at once, so a short wait is enough.
      uint32_t t0 = millis();
      while (!Serial.available() && millis() - t0 < 50) delay(1);
      char n = Serial.available() ? Serial.peek() : 0;
      if (n == 'a' || n == 'b') { Serial.read(); setSource(n); }
      else if (n == 'r') {       // 'cr' = forget this channel's trial
        Serial.read();
        char kr[8], km[8];
        snprintf(kr, sizeof kr, "rest%c", srcName);
        snprintf(km, sizeof km, "mvc%c", srcName);
        prefs.begin("emg", false);
        prefs.remove(kr); prefs.remove(km);
        prefs.end();
        Serial.printf("# ch %c trial erased, reverting to built-in default\n",
                      srcName - 32);
        loadCalibration();
      }
      else                        calibrate();
    }
  }

  uint32_t now = micros();
  if ((int32_t)(now - nextSampleUs) < 0) return;
  nextSampleUs += SAMPLE_US;

  int src = readAvg(srcPin);

  // intent pipeline runs in every mode so the servo is always fed
  envUpdate(src);
  if (++tickCount >= TICK_DIV) {
    tickCount = 0;
    intentTick();
  }

  if (mode == 'p') {   // "chA,chB,env,intent" - both channels plus what the
                       // state machine made of them, so the viewer can show all
    int a = (srcPin == PIN_RAW) ? src : readAvg(PIN_RAW);
    int b = (srcPin == PIN_ENV) ? src : readAvg(PIN_ENV);
    Serial.print(a);          Serial.print(',');
    Serial.print(b);          Serial.print(',');
    Serial.print((int)env);   Serial.print(',');
    Serial.println(lastIntent);
  }
}
