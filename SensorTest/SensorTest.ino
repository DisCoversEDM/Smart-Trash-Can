/*
  Raw pulse diagnostic
  Front : TRIG D5   ECHO D4
  Lid   : TRIG D7   ECHO D6
  Prints the raw pulseIn() duration (microseconds) per sample.
  0 us  = no echo (timeout). A valid echo is ~ (2 * distance_cm * 29) us.
*/

const uint8_t F_TRIG = 5, F_ECHO = 4;
const uint8_t L_TRIG = 7, L_ECHO = 6;
const unsigned long ECHO_TIMEOUT_US = 30000;
const int SAMPLES = 5;

unsigned long pulseRaw(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
}

void sweep(const char *label, uint8_t trigPin, uint8_t echoPin) {
  Serial.print(label);
  Serial.print(" [us]:");
  for (int i = 0; i < SAMPLES; i++) {
    Serial.print(' ');
    Serial.print(pulseRaw(trigPin, echoPin));
    delay(60);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(F_TRIG, OUTPUT); pinMode(F_ECHO, INPUT);
  pinMode(L_TRIG, OUTPUT); pinMode(L_ECHO, INPUT);
  digitalWrite(F_TRIG, LOW); digitalWrite(L_TRIG, LOW);
  Serial.println("=== RAW PULSE DIAGNOSTIC ===");
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last >= 1200) {
    last = millis();
    sweep("front(5/4)", F_TRIG, F_ECHO);
    sweep("lid  (7/6)", L_TRIG, L_ECHO);
    Serial.println("-");
  }
}
