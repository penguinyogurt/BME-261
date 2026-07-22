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

void setup() {
  Serial.begin(115200);
  delay(300);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_RAW, ADC_11db);   // ~0 - 3.3 V input range
  analogSetPinAttenuation(PIN_ENV, ADC_11db);

  Serial.println(F("\nEMG bring-up test ready."));
  Serial.println(F("cmds: p=plot  s=stats  t=self-test  z=tare"));
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
    } else if (c == 'p' || c == 's') {
      mode = c; resetStats();
    }
  }

  // fixed-rate sampling
  uint32_t now = micros();
  if ((int32_t)(now - nextSampleUs) < 0) return;
  nextSampleUs += SAMPLE_US;

  int a = readAvg(PIN_RAW);
  int b = readAvg(PIN_ENV);

  if (mode == 'p') {
    Serial.print(a); Serial.print(','); Serial.println(b);
  } else {  // stats
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
