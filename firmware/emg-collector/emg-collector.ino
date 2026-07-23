/*
 * EMG front-end bring-up test for ESP32
 * ------------------------------------------------------------
 * Verifies both analog channels BEFORE you commit to perfboard:
 *   ch A -> raw bandpass output (must be DC-biased to mid-rail)
 *   ch B -> peak detector envelope output
 *
 * On boot it runs a self-test that checks the resting bias level,
 * looks for clipping, and warns about a floating / ungrounded input.
 *
 * Serial commands (send a single character):
 *   p : plot stream   ->  "chA,chB"  (Arduino Serial Plotter or Python viewer)
 *   s : stats         ->  periodic min/max/mean/peak-to-peak per channel
 *   t : self-test     ->  re-run the bias / clip / ground health check
 *   z : tare          ->  capture the current levels as resting baseline
 *   c : calibrate     ->  guided rest + max-contraction trial; sets the
 *                         intent thresholds and stores them in flash (NVS)
 *   i : intent stream ->  "env,intent" at 50 Hz (viewer-compatible)
 *
 * Motor intent (runs in EVERY mode once calibrated):
 *   A digital envelope (rectified chA, ~200 ms EMA) feeds a latching state
 *   machine ticked at 50 Hz:
 *     OPEN    --flex > T_high--> CLOSING  (intent +250..+1000, prop. to env)
 *     CLOSING --relax--------->  HOLDING  (latched: intent 0, hand keeps grip)
 *     HOLDING --full relax for OPEN_DWELL_MS--> OPENING (intent -OPEN_SPEED,
 *              timed; any activity above T_low resets the dwell timer, so an
 *              occasional light "refresh squeeze" keeps the grip latched)
 *     any state --strong flex--> CLOSING
 *   The intent is sent over ESP-NOW to the servo board (7C:87:CE:30:FA:88)
 *   as {seq, intent, state, calibrated} at 50 Hz. The servo board owns all
 *   safety (travel clamp, watchdog, stall stop) - we only send intent.
 *
 * NOTE: WiFi is kept on for ESP-NOW, so ADC2 pins are unusable - both
 * channels below are ADC1, which is why this works.
 *
 * Wiring:
 *   raw bandpass out ---> GPIO34 (ADC1_CH6, input-only)
 *   peak detector out --> GPIO35 (ADC1_CH7, input-only)
 *   circuit GND -------> ESP32 GND   (shared ground is mandatory)
 *
 * WARNING: never feed more than 3.3 V into a GPIO. Confirm with a
 * multimeter that both outputs stay within 0 - 3.3 V before wiring.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>

// ---- Pins (ADC1 only; ADC2 is unusable once WiFi is on) ----
const int PIN_RAW = 34;   // raw bandpass (biased to ~1.65 V)
const int PIN_ENV = 35;   // peak detector envelope

// ---- Sampling ----
const uint32_t SAMPLE_HZ  = 500;                    // plenty to verify reads
const uint32_t SAMPLE_US  = 1000000UL / SAMPLE_HZ;
const int      OVERSAMPLE = 4;                      // average N reads (noise)

// ---- ADC ----
const int ADC_MAX = 4095;                           // 12-bit

char     mode = 'p';
uint32_t nextSampleUs = 0;

// stats window
const uint32_t STAT_WIN_MS = 500;
uint32_t statWinStart = 0;
int  aMin, aMax, bMin, bMax;
long aSum, bSum;
uint32_t nStat;

int baseA = 0, baseB = 0;   // resting baseline

// ---- Intent / latching state machine ----
const uint8_t SERVO_MAC[6] = {0x7C, 0x87, 0xCE, 0x30, 0xFA, 0x88};

const uint32_t TICK_DIV     = 10;     // 500 Hz / 10 = 50 Hz intent + radio rate
const uint32_t TICK_MS      = 1000 / (SAMPLE_HZ / TICK_DIV);
const float    ENV_ALPHA    = 0.01f;  // ~200 ms EMA at 500 Hz
const float    T_LOW_FRAC   = 0.12f;  // thresholds as fraction of (MVC - rest)
const float    T_HIGH_FRAC  = 0.28f;
const float    FULL_FRAC    = 0.80f;  // intent saturates at 80% MVC
const uint32_t DEBOUNCE_MS  = 80;     // sustained flex needed to enter CLOSING
const uint32_t RELAX_MS     = 150;    // sustained relax in CLOSING -> latch
const uint32_t OPEN_DWELL_MS = 2000;  // full relax this long -> hand opens
const uint32_t OPEN_DRIVE_MS = 2500;  // how long OPEN intent is commanded
const int      OPEN_SPEED   = 600;    // intent magnitude while opening
const int      CLOSE_MIN    = 250;    // weakest close intent once engaged

enum IntentState : uint8_t { ST_OPEN = 0, ST_CLOSING, ST_HOLDING, ST_OPENING };
const char *STATE_NAMES[] = {"OPEN", "CLOSING", "HOLDING", "OPENING"};

typedef struct __attribute__((packed)) {
  uint32_t seq;
  int16_t  intent;      // -1000 open .. 0 hold .. +1000 close
  uint8_t  state;       // IntentState
  uint8_t  calibrated;  // 0 -> servo board must stay neutral
} IntentPacket;

Preferences prefs;
float    env = 0.0f;              // digital envelope of chA (counts)
float    restEnv = 0.0f, mvcEnv = 0.0f;
bool     calibrated = false;
IntentState ist = ST_OPEN;
uint32_t tickCount = 0, txSeq = 0, txFails = 0, lastFailReportMs = 0;
uint32_t aboveHighMs = 0, belowHighMs = 0, belowLowMs = 0, openStartMs = 0;

float envUpdate(int a) {
  float r = fabsf((float)(a - baseA));   // rectify around resting baseline
  env += ENV_ALPHA * (r - env);
  return env;
}

int readAvg(int pin) {
  long acc = 0;
  for (int i = 0; i < OVERSAMPLE; i++) acc += analogRead(pin);
  return (int)(acc / OVERSAMPLE);
}

void resetStats() {
  aMin = bMin = ADC_MAX; aMax = bMax = 0;
  aSum = bSum = 0; nStat = 0;
  statWinStart = millis();
}

void selfTest() {
  Serial.println();
  Serial.println(F("== self-test (hold muscle relaxed) =="));
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
  int mvA = analogReadMilliVolts(PIN_RAW);
  int mvB = analogReadMilliVolts(PIN_ENV);

  Serial.printf("ch A (raw): mean=%d (%d mV)  range[%d..%d]  ripple=%d\n",
                meanA, mvA, mnA, mxA, mxA - mnA);
  Serial.printf("ch B (env): mean=%d (%d mV)  range[%d..%d]  ripple=%d\n",
                meanB, mvB, mnB, mxB, mxB - mnB);

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
    Serial.println(F("  ! ch A very noisy at rest - check shared ground / floating input."));

  baseA = meanA; baseB = meanB;
  Serial.println(F("  baseline captured."));
  Serial.println(F("=====================================\n"));
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
  peer.channel = 0;   // current channel (both boards default to 1)
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK)
    Serial.println(F("! esp_now_add_peer failed - intent radio disabled"));
}

void loadCalibration() {
  prefs.begin("emg", true);
  restEnv = prefs.getFloat("restEnv", 0.0f);
  mvcEnv  = prefs.getFloat("mvcEnv", 0.0f);
  prefs.end();
  calibrated = (mvcEnv - restEnv) > 40.0f;
  if (calibrated)
    Serial.printf("calibration loaded: rest=%.0f mvc=%.0f\n", restEnv, mvcEnv);
  else
    Serial.println(F("not calibrated - intent stays 0 until you run 'c'."));
}

// Sample both channels at the normal rate for `ms`, keeping the envelope
// EMA warm. Returns the mean envelope; tracks the max via *maxOut.
float sampleEnvWindow(uint32_t ms, float *maxOut) {
  double sum = 0; uint32_t n = 0; float mx = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    float e = envUpdate(readAvg(PIN_RAW));
    (void)readAvg(PIN_ENV);
    sum += e; n++; if (e > mx) mx = e;
    delay(2);
  }
  if (maxOut) *maxOut = mx;
  return n ? (float)(sum / n) : 0.0f;
}

void calibrate() {
  Serial.println(F("\n== calibration =="));
  Serial.println(F("step 1/2: RELAX the muscle completely..."));
  for (int i = 3; i > 0; i--) { Serial.printf("  %d...\n", i); delay(1000); }

  // re-tare the raw baseline off the first half second, then measure rest
  long acc = 0; const int N = 250;
  for (int i = 0; i < N; i++) { acc += readAvg(PIN_RAW); delay(2); }
  baseA = acc / N;
  baseB = readAvg(PIN_ENV);
  env = 0.0f;                       // restart the EMA from the new baseline
  sampleEnvWindow(500, nullptr);    // settle
  float rest = sampleEnvWindow(2000, nullptr);

  Serial.println(F("step 2/2: now SQUEEZE as hard as you can, hold it..."));
  for (int i = 3; i > 0; i--) { Serial.printf("  %d...\n", i); delay(1000); }
  float mvc = 0.0f;
  sampleEnvWindow(3000, &mvc);
  Serial.println(F("  ...and relax."));

  if (mvc - rest <= 40.0f) {
    Serial.printf("! contraction too weak (rest=%.0f mvc=%.0f) - check "
                  "electrodes. Calibration NOT saved.\n", rest, mvc);
    return;
  }
  restEnv = rest; mvcEnv = mvc; calibrated = true;
  prefs.begin("emg", false);
  prefs.putFloat("restEnv", restEnv);
  prefs.putFloat("mvcEnv", mvcEnv);
  prefs.end();
  float span = mvcEnv - restEnv;
  Serial.printf("saved: rest=%.0f mvc=%.0f | T_low=%.0f T_high=%.0f full=%.0f\n",
                restEnv, mvcEnv, restEnv + T_LOW_FRAC * span,
                restEnv + T_HIGH_FRAC * span, restEnv + FULL_FRAC * span);
  Serial.println(F("=================\n"));
}

// 50 Hz: run the latching state machine and radio the intent to the servo.
void intentTick() {
  int intent = 0;

  if (calibrated) {
    float span  = mvcEnv - restEnv;
    float tLow  = restEnv + T_LOW_FRAC  * span;
    float tHigh = restEnv + T_HIGH_FRAC * span;
    float full  = restEnv + FULL_FRAC   * span;

    // debounce accumulators
    aboveHighMs = (env > tHigh) ? aboveHighMs + TICK_MS : 0;
    belowHighMs = (env < tHigh) ? belowHighMs + TICK_MS : 0;
    belowLowMs  = (env < tLow)  ? belowLowMs  + TICK_MS : 0;  // dwell timer;
                                  // any activity >= T_low is a refresh squeeze

    IntentState prev = ist;
    switch (ist) {
      case ST_OPEN:
        if (aboveHighMs >= DEBOUNCE_MS) ist = ST_CLOSING;
        break;
      case ST_CLOSING: {
        float frac = (env - tHigh) / (full - tHigh);
        frac = constrain(frac, 0.0f, 1.0f);
        intent = CLOSE_MIN + (int)(frac * (1000 - CLOSE_MIN));
        if (belowHighMs >= RELAX_MS) ist = ST_HOLDING;   // latch the grip
        break;
      }
      case ST_HOLDING:
        if (aboveHighMs >= DEBOUNCE_MS) ist = ST_CLOSING;          // re-grip
        else if (belowLowMs >= OPEN_DWELL_MS) {                    // let go
          ist = ST_OPENING; openStartMs = millis();
        }
        break;
      case ST_OPENING:
        intent = -OPEN_SPEED;
        if (aboveHighMs >= DEBOUNCE_MS) ist = ST_CLOSING;    // grab mid-open
        else if (millis() - openStartMs >= OPEN_DRIVE_MS) ist = ST_OPEN;
        break;
    }
    if (ist != prev)   // no comma, so the Python viewer's parser skips it
      Serial.printf("# intent state -> %s\n", STATE_NAMES[ist]);
  }

  IntentPacket p;
  p.seq = ++txSeq;
  p.intent = (int16_t)intent;
  p.state = (uint8_t)ist;
  p.calibrated = calibrated ? 1 : 0;
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

  Serial.println(F("\nEMG bring-up test ready."));
  Serial.println(F("cmds: p=plot  s=stats  t=self-test  z=tare  c=calibrate  i=intent"));
  initEspNow();        // WiFi up BEFORE self-test so noise is measured
  loadCalibration();   // under real operating conditions
  selfTest();
  resetStats();
  nextSampleUs = micros();
}

void loop() {
  // commands
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 't') {
      selfTest();
    } else if (c == 'z') {
      baseA = readAvg(PIN_RAW); baseB = readAvg(PIN_ENV);
      Serial.printf("tared: baseA=%d baseB=%d\n", baseA, baseB);
    } else if (c == 'c') {
      calibrate();
    } else if (c == 'p' || c == 's' || c == 'i') {
      mode = c; resetStats();
    }
  }

  // fixed-rate sampling
  uint32_t now = micros();
  if ((int32_t)(now - nextSampleUs) < 0) return;
  nextSampleUs += SAMPLE_US;

  int a = readAvg(PIN_RAW);
  int b = readAvg(PIN_ENV);

  // intent pipeline runs in every mode so the servo is always fed
  envUpdate(a);
  if (++tickCount >= TICK_DIV) {
    tickCount = 0;
    intentTick();
  }

  if (mode == 'p') {
    Serial.print(a); Serial.print(','); Serial.println(b);
  } else if (mode == 's') {  // stats
    if (a < aMin) aMin = a;  if (a > aMax) aMax = a;  aSum += a;
    if (b < bMin) bMin = b;  if (b > bMax) bMax = b;  bSum += b;
    nStat++;
    if (millis() - statWinStart >= STAT_WIN_MS) {
      Serial.printf("A mean=%ld pp=%d [%d..%d] | B mean=%ld pp=%d [%d..%d]\n",
                    aSum / (long)nStat, aMax - aMin, aMin, aMax,
                    bSum / (long)nStat, bMax - bMin, bMin, bMax);
      resetStats();
    }
  }
}
