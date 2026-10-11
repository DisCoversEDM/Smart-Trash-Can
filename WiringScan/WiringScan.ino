/*
  Wiring scan for ultrasonic (HC-SR04) sensors.

  It fires each candidate TRIG pin and listens on each candidate ECHO pin,
  reporting any combination that actually receives an echo. This finds
  swapped TRIG/ECHO wires or a sensor connected to different pins than assumed.

  Hold a flat target ~5-20 cm in front of a sensor while scanning.
*/

const uint8_t trigCandidates[] = {4, 5, 6, 7};
const uint8_t echoCandidates[] = {2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13};
const unsigned long ECHO_TIMEOUT_US = 30000;
const int REPEAT = 3;

unsigned long ping(uint8_t trig, uint8_t echo) {
  pinMode(echo, INPUT);
  pinMode(trig, OUTPUT);
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  return pulseIn(echo, HIGH, ECHO_TIMEOUT_US);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== WIRING SCAN (target ~5-20cm in front of a sensor) ===");
}

void loop() {
  delay(1500);
  Serial.println("scan start");
  for (unsigned int t = 0; t < sizeof(trigCandidates); t++) {
    uint8_t trig = trigCandidates[t];
    for (unsigned int e = 0; e < sizeof(echoCandidates); e++) {
      uint8_t echo = echoCandidates[e];
      if (echo == trig) continue;

      int hits = 0;
      unsigned long best = 0;
      for (int r = 0; r < REPEAT; r++) {
        unsigned long d = ping(trig, echo);
        if (d > 0) { hits++; best = d; }
        delay(30);
      }
      if (hits > 0) {
        Serial.print("trig D"); Serial.print(trig);
        Serial.print(" + echo D"); Serial.print(echo);
        Serial.print(" -> ECHO  hits="); Serial.print(hits);
        Serial.print("/"); Serial.print(REPEAT);
        Serial.print("  (best ~");
        Serial.print(best * 0.0343f / 2.0f, 1);
        Serial.println(" cm)");
      }
    }
  }
  Serial.println("scan done");
}
