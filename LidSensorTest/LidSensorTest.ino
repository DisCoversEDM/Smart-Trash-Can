/*
  Under-lid sensor test (HC-SR04 "justPi")
  TRIG -> D7   ECHO -> D6
  Prints the measured distance every 500 ms at 115200 baud.
*/

const uint8_t LID_TRIG_PIN = 7;
const uint8_t LID_ECHO_PIN = 6;
const unsigned long ECHO_TIMEOUT_US = 25000;

float readDistanceCm() {
  digitalWrite(LID_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(LID_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(LID_TRIG_PIN, LOW);
  unsigned long duration = pulseIn(LID_ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return -1.0f;
  return duration * 0.0343f / 2.0f;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(LID_TRIG_PIN, OUTPUT);
  pinMode(LID_ECHO_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LID_TRIG_PIN, LOW);
  Serial.println("=== LID SENSOR TEST (TRIG D7 / ECHO D6) ===");
}

void loop() {
  static unsigned long last = 0;
  digitalWrite(LED_BUILTIN, (millis() / 500) % 2);   // heartbeat

  if (millis() - last >= 500) {
    last = millis();
    float d = readDistanceCm();
    if (d > 0.0f) {
      Serial.print("lid: ");
      Serial.print(d, 1);
      Serial.println(" cm");
    } else {
      Serial.println("lid: no echo (check wiring / 5V / GND)");
    }
  }
}
