/*
  SignalDiag - isolates WHY a servo on LID_PIN doesn't move.

  It runs LID_PIN through 6 phases (~5s each), printing the expected meter
  reading (measure LID_PIN header pin -> GND, meter on DC volts) and telling
  you what the servo should do. Watch the servo AND the meter.

  P1 digital LOW      -> ~0.0 V
  P2 digital HIGH     -> ~5.0 V
  P3 Servo lib  90deg -> ~0.38 V   (servo centers if powered)
  P4 Servo lib 180deg -> ~0.50 V   (servo moves if powered)
  P5 raw 2.0ms pulses -> ~0.50 V   (bypasses the Servo library)
  P6 raw 1.0ms pulses -> ~0.25 V   (bypasses the Servo library)
*/

#include <Servo.h>

const uint8_t LID_PIN = 9;
Servo lid;

void waitSec(int n, const __FlashStringHelper* tag) {
  for (int s = 0; s < n; s++) {
    Serial.print(tag); Serial.print(' '); Serial.print(n - s); Serial.println('s');
    delay(1000);
  }
}

void rawPulses(int usec, unsigned long ms) {
  unsigned long t = millis();
  while (millis() - t < ms) {
    digitalWrite(LID_PIN, HIGH); delayMicroseconds(usec);
    digitalWrite(LID_PIN, LOW);  delayMicroseconds(20000 - usec);
  }
}

void setup() {
  Serial.begin(115200);
  delay(600);
  pinMode(LID_PIN, OUTPUT);
  Serial.println(F("=== SIGNAL DIAG: lid servo signal on LID_PIN ==="));
  Serial.println(F("Meter: black on GND, red on the LID_PIN pin (orange wire row)."));
}

void loop() {
  lid.detach();
  pinMode(LID_PIN, OUTPUT);

  Serial.println();
  digitalWrite(LID_PIN, LOW);
  Serial.println(F(">>> P1: LID_PIN = LOW        expect meter ~0.0 V"));
  waitSec(5, F("  P1"));

  digitalWrite(LID_PIN, HIGH);
  Serial.println(F(">>> P2: LID_PIN = HIGH       expect meter ~5.0 V"));
  waitSec(5, F("  P2"));

  lid.attach(LID_PIN);
  lid.write(90);
  Serial.println(F(">>> P3: Servo lib 90    expect ~0.38 V  (servo centers?)"));
  waitSec(5, F("  P3"));

  lid.write(180);
  Serial.println(F(">>> P4: Servo lib 180   expect ~0.50 V  (servo moves?)"));
  waitSec(5, F("  P4"));

  lid.detach();
  pinMode(LID_PIN, OUTPUT);
  Serial.println(F(">>> P5: RAW 2.0ms pulses  expect ~0.50 V  (bypass Servo lib)"));
  rawPulses(2000, 5000);

  Serial.println(F(">>> P6: RAW 1.0ms pulses  expect ~0.25 V  (bypass Servo lib)"));
  rawPulses(1000, 5000);

  Serial.println(F("--- cycle repeats ---"));
  delay(1000);
}
