// Drive a Hitec HSR-2645CR continuous-rotation servo from an ESP32.
//
// The HSR-2645CR is a servo, not a bare DC motor: it takes a standard
// 50 Hz servo signal and spins continuously. Pulse width sets speed AND
// direction:
//     1500 us  = stopped (neutral)
//     > 1500   = rotate one direction (faster toward 2100 us)
//     < 1500   = rotate the other direction (faster toward 900 us)
//
// Wiring:
//   servo signal (yellow) -> GPIO18
//   servo VCC (red)       -> +4.8-6 V from a SEPARATE battery pack
//   servo GND (black)     -> battery negative
//   ESP32 GND             -> battery negative   (shared ground is required)
//
// Library: install "ESP32Servo" via the Arduino Library Manager, and the
// "esp32" boards package (Boards Manager) so you can select your dev board.

#include <ESP32Servo.h>

Servo drive;                    // represents the HSR-2645CR

const int SERVO_PIN = 18;       // signal wire -> GPIO18
const int STOP_US   = 1500;     // neutral: motor stopped
const int MIN_US    = 900;      // full speed one direction
const int MAX_US    = 2100;     // full speed the other direction

void setup() {
  drive.setPeriodHertz(50);                 // standard servo frame (20 ms)
  drive.attach(SERVO_PIN, MIN_US, MAX_US);  // claim the pin
  drive.writeMicroseconds(STOP_US);         // make sure it starts stopped
  delay(1000);
}

void loop() {
  // gentle forward (well below full speed)
  drive.writeMicroseconds(1900);
  delay(2000);

  // stop
  drive.writeMicroseconds(STOP_US);
  delay(1000);

  // gentle reverse
  drive.writeMicroseconds(1000);
  delay(2000);

  // stop
  drive.writeMicroseconds(STOP_US);
  delay(1000);
}
