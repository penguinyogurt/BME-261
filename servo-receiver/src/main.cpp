// Servo-receiver ESP32 — step 1: report this board's MAC address.
//
// ESP-NOW pairs boards by WiFi station MAC. The EMG collection ESP32 (sender)
// must be given THIS board's MAC as its peer address.
//
// This board's MAC (read via esptool, confirmed by this sketch):
//   7C:87:CE:30:FA:88
//
// Next step: register an ESP-NOW receive callback here and drive the servo
// from the envelope value the EMG collector sends.

#include <Arduino.h>
#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(500);

  // ESP-NOW runs on the station interface; the STA MAC is the pairing address.
  WiFi.mode(WIFI_STA);

  Serial.println();
  Serial.println("=== Servo receiver bring-up ===");
  Serial.print("STA MAC (peer address for the EMG sender): ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  // Re-print every 2 s so the MAC is visible whenever the monitor attaches.
  Serial.println(WiFi.macAddress());
  delay(2000);
}
