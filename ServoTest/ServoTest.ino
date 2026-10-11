/*
  Servo test
    Lid SG-90        : signal D9   (sweeps 0 -> 180 -> 0)
    Bag motor #1     : signal D10  (continuous-rotation MG996R)
    Bag motor #2     : signal D11  (continuous-rotation MG996R)

  MG996R 360: 90 = stop, 180 = one way, 0 = the other way.
  Watch the rig: the lid should sweep fully, and both motors should spin
  one way, stop, then reverse.
*/

#include <Servo.h>

const uint8_t LID_SERVO_PIN = 9;
const uint8_t MOTOR1_PIN    = 10;
const uint8_t MOTOR2_PIN    = 11;

const int MOTOR_STOP = 90;
const int MOTOR_FWD  = 180;
const int MOTOR_REV  = 0;

Servo lidServo;
Servo motor1;
Servo motor2;

void motorWrite(int angle) {
  motor1.write(angle);
  motor2.write(angle);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  lidServo.attach(LID_SERVO_PIN);
  motor1.attach(MOTOR1_PIN);
  motor2.attach(MOTOR2_PIN);
  lidServo.write(0);
  motorWrite(MOTOR_STOP);
  Serial.println("=== SERVO TEST: D9 lid, D10 + D11 motors ===");
  delay(1000);
}

void sweepLid() {
  Serial.println("lid: sweeping 0 -> 180");
  for (int a = 0; a <= 180; a += 5) { lidServo.write(a); delay(40); }
  delay(500);
  Serial.println("lid: sweeping 180 -> 0");
  for (int a = 180; a >= 0; a -= 5) { lidServo.write(a); delay(40); }
  delay(500);
}

void spinMotors() {
  Serial.println("motors: FORWARD");
  motorWrite(MOTOR_FWD);
  delay(2500);
  Serial.println("motors: STOP");
  motorWrite(MOTOR_STOP);
  delay(1000);
  Serial.println("motors: REVERSE");
  motorWrite(MOTOR_REV);
  delay(2500);
  Serial.println("motors: STOP");
  motorWrite(MOTOR_STOP);
  delay(1000);
}

void loop() {
  sweepLid();
  spinMotors();
  Serial.println("--- cycle done ---");
  delay(1500);
}
