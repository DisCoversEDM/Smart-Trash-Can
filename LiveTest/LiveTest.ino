/*
  LiveTest - wave your hand in front of the FRONT sensor.
    front distance < 20 cm  -> lid opens (0->180) AND both bag motors run
    front distance >= 20 cm -> lid closes (->0)    AND both motors stop

  Front HC-SR04 : TRIG D5, ECHO D4
  Lid SG-90     : signal D9
  Bag motor #1  : signal D10  (MG996R, 90=stop, 180=fwd, 0=rev)
  Bag motor #2  : signal D11

  Noise handling: median-of-5 filtering + a 6cm hysteresis band
  (open<20, close>26) + debounce (needs several consistent reads
  in a row before switching). This kills all spurious open/close chatter.
*/

#include <Servo.h>

const uint8_t FRONT_TRIG = 5;
const uint8_t FRONT_ECHO = 4;
const uint8_t LID_PIN    = 9;
const uint8_t MOTOR1_PIN = 10;
const uint8_t MOTOR2_PIN = 11;

const float OPEN_CM        = 20.0;  // nearer than this  -> open + run
const float CLOSE_CM       = 26.0;  // farther than this -> close + stop
const int   OPEN_CONFIRM   = 3;     // consecutive near reads needed to open
const int   CLOSE_CONFIRM  = 10;    // consecutive far reads needed to close

const int LID_CLOSED = 0;
const int LID_OPEN   = 180;
const int MOTOR_STOP = 90;
const int MOTOR_RUN  = 180;

Servo lidServo;
Servo motor1;
Servo motor2;

int  nearCnt = 0, farCnt = 0;
bool open = false;

float readCmRaw(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long d = pulseIn(echo, HIGH, 30000UL);
  if (d == 0) return -1.0;
  return d * 0.0343 / 2.0;
}

float readCmMedian(uint8_t trig, uint8_t echo) {
  const int N = 5;
  float v[N];
  int valid = 0;
  for (int i = 0; i < N; i++) {
    float c = readCmRaw(trig, echo);
    if (c > 0) v[valid++] = c;
  }
  if (valid == 0) return -1.0;
  for (int i = 1; i < valid; i++) {          // insertion sort
    float k = v[i]; int j = i - 1;
    while (j >= 0 && v[j] > k) { v[j + 1] = v[j]; j--; }
    v[j + 1] = k;
  }
  return v[valid / 2];
}

void setOpen(bool o) {
  if (o == open) return;
  open = o;
  lidServo.write(o ? LID_OPEN : LID_CLOSED);
  int m = o ? MOTOR_RUN : MOTOR_STOP;
  motor1.write(m);
  motor2.write(m);
  Serial.print("[state] ");
  Serial.println(o ? "OPEN + motors RUN" : "CLOSED + motors STOP");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);
  lidServo.attach(LID_PIN);
  motor1.attach(MOTOR1_PIN);
  motor2.attach(MOTOR2_PIN);
  lidServo.write(LID_CLOSED);
  motor1.write(MOTOR_STOP);
  motor2.write(MOTOR_STOP);
  Serial.println("=== LIVE TEST: front < 20cm -> lid open + motors ===");
}

unsigned long lastPrint = 0;

void loop() {
  float cm = readCmMedian(FRONT_TRIG, FRONT_ECHO);

  if (cm > 0) {
    if (cm < OPEN_CM)       { nearCnt++; farCnt = 0; }
    else if (cm > CLOSE_CM) { farCnt++;  nearCnt = 0; }
    // in the 20..26 deadband: hold both counters (keep current state)

    if (!open && nearCnt >= OPEN_CONFIRM)  { nearCnt = 0; setOpen(true);  }
    if (open  && farCnt  >= CLOSE_CONFIRM) { farCnt  = 0; setOpen(false); }
  }

  if (millis() - lastPrint >= 250) {
    lastPrint = millis();
    Serial.print("[front] ");
    if (cm < 0) Serial.print("-- (no echo)");
    else { Serial.print(cm, 1); Serial.print(" cm"); }
    Serial.print(open ? "  OPEN" : "  closed");
    Serial.print("   near="); Serial.print(nearCnt);
    Serial.print(" far=");    Serial.println(farCnt);
  }
  delay(20);
}
