/*
  Smart Trash Can - Arduino UNO R4 WiFi
  -------------------------------------

  WIRING
    Front HC-SR04 "justPi" (proximity) : TRIG -> D5    ECHO -> D4
    Lid   HC-SR04 "justPi" (fill)      : TRIG -> D6    ECHO -> D7   (points down)
    (NOTE: the lid sensor's TRIG/ECHO were found SWAPPED vs the original plan.
     If you ever rewire it to TRIG->D7 / ECHO->D6, swap the two constants below.)
    SG-90 micro servo (lid)            : signal -> D9
    MG996R 360 #1 (bag eject, left)    : signal -> D10
    MG996R 360 #2 (bag eject, right)   : signal -> D11
    B10k potentiometer (threshold)     : wiper -> A0,  ends -> 5V / GND
    (A1 / A2 are reserved / unused)

  POWER WARNING
    Two MG996R can draw up to ~2.5 A EACH at stall, and the SG-90 adds more on
    top. USB from a laptop only safely supplies ~0.5-0.9 A, so pushing the bag
    under load can brown out the board or trip the USB port. For anything past
    a light prototype, power the servos from a separate 5 V (2-3 A) supply and
    tie its GND to the Arduino GND.

  SERVO LIBRARY
    On the UNO R4 WiFi, use the Arduino "Servo" library >= 1.2.2. Older
    versions jitter / step coarsely on the Renesas (R4) boards.

  LED MATRIX DISPLAY
    The on-board 12x8 LED matrix shows the fill percentage as two digits
    (e.g. " 5" or "60"); it shows "--" when the can is not calibrated yet.
    This needs the standalone "ArduinoGraphics" library (Library Manager).
    The matrix uses high-rate FSP timers; if the SG-90 ever jitters, that is
    the usual culprit.

  SERIAL COMMANDS (115200 baud, Serial Monitor, line ending = Newline)
    c  calibrate empty-can depth (lid MUST be closed) and save to EEPROM
       (optional: by default 0% = 20 cm and 100% = 2 cm)
    x  reset calibration back to the default 20 cm .. 2 cm scale
    r  measure and print the fill percentage right now
    i  print calibration + live distance readings
    l  toggle LIVE MODE: streams both sensors as distance + % (~3 times per second)
*/

// ArduinoGraphics must be included BEFORE Arduino_LED_Matrix.
#include "ArduinoGraphics.h"
#include "Arduino_LED_Matrix.h"
#include <Servo.h>
#include <EEPROM.h>

// ---------------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------------
const uint8_t FRONT_TRIG_PIN = 5;
const uint8_t FRONT_ECHO_PIN = 4;
const uint8_t LID_TRIG_PIN   = 6;
const uint8_t LID_ECHO_PIN   = 7;
const uint8_t LID_SERVO_PIN  = 9;
const uint8_t MOTOR1_PIN     = 10;
const uint8_t MOTOR2_PIN     = 11;
const uint8_t POT_PIN        = A0;

// ---------------------------------------------------------------------------
// Servo angles (adjust to your mechanics)
//   SG-90 is a positional servo: degrees.
//   MG996R 360 is a continuous-rotation servo: 90 = stop, 0 / 180 = full speed.
// ---------------------------------------------------------------------------
const int LID_CLOSED_ANGLE = 0;    // lid fully shut
const int LID_OPEN_ANGLE   = 120;  // lid fully open (0 -> 120 sweep)

const int MOTOR_STOP = 90;         // no rotation
const int MOTOR_FWD  = 180;        // one direction (pushes bag up)
const int MOTOR_REV  = 0;          // opposite direction (brings bag down)

// ---------------------------------------------------------------------------
// Timing (milliseconds)
// ---------------------------------------------------------------------------
const unsigned long HAND_CLEAR_MS      = 500;   // hand must be away this long before a new wave can close the lid
const unsigned long UP_MS              = 3000;  // how long the bag is pushed up
                                                 // (keep well under EJECT_TOTAL_MS)
const unsigned long EJECT_TOTAL_MS     = 10000; // total time before retracting (user removes bag)
const unsigned long FILL_READ_DELAY_MS = 3000;  // wait after lid closes before reading fill
const unsigned long REOPEN_COOLDOWN_MS = 1000;  // min gap between lid closes and next open
const unsigned long ECHO_TIMEOUT_US    = 25000; // ~4.3 m max echo wait
const unsigned long SENSOR_GAP_MS      = 60;    // gap between ultrasonic readings (anti crosstalk)
const unsigned long LIVE_INTERVAL_MS   = 300;   // live mode print interval

// ---------------------------------------------------------------------------
// Accepted distance ranges (readings outside the range count as "no echo")
//   Front sensor: only 2..5 cm counts.
//   Lid sensor  : 2 cm .. 11 inches (27.94 cm) counts.
// ---------------------------------------------------------------------------
const float FRONT_MIN_CM = 2.0f;
const float FRONT_MAX_CM = 5.0f;
const float LID_MIN_CM   = 2.0f;
const float LID_MAX_CM   = 11.0f * 2.54f;   // 27.94 cm
const float LID_EMPTY_CM = 20.0f;           // default 0 % (empty can reads 20-21 cm)

// ---------------------------------------------------------------------------
// Fill / eject thresholds
// ---------------------------------------------------------------------------
const float EJECT_FILL_PCT = 60.0;  // eject when fill >= this (test value; raise to 90 later)

// Proximity trigger distance. By default it is a FIXED 20 cm; set
// USE_POT_FOR_THRESHOLD to true to let the A0 potentiometer set it live.
const bool  USE_POT_FOR_THRESHOLD = false;
const float FIXED_PROXIMITY_CM    = 5.0;    // front sensor opens the lid at this range
const int   POT_MIN_CM     = 5;     // pot fully left  -> this proximity distance
const int   POT_MAX_CM     = 100;   // pot fully right -> this proximity distance

// ---------------------------------------------------------------------------
// EEPROM layout
// ---------------------------------------------------------------------------
const int      EEPROM_MAGIC_ADDR  = 0;      // 1 byte marker
const uint8_t  CAL_MAGIC          = 0xA5;   // "calibrated" marker
const int      EEPROM_DEPTH_ADDR  = 4;      // float, needs 4-byte alignment

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
Servo lidServo;
Servo motor1;
Servo motor2;

ArduinoLEDMatrix matrix;

enum EjectState { EJ_NONE, EJ_UP, EJ_HOLD, EJ_DOWN };

bool          lidIsOpen          = false;
unsigned long lidOpenedAt        = 0;
unsigned long lidClosedAt        = 0;
bool          handCleared        = false; // true once the hand left after the lid opened
unsigned long handAbsentSince    = 0;     // when the front sensor last stopped seeing a hand

EjectState    ejectState         = EJ_NONE;
bool          ejectActive        = false;
unsigned long ejectStartTime     = 0;   // when the whole eject sequence began
unsigned long ejectPhaseTime     = 0;   // when the current phase began

bool          pendingFillRead    = false;
unsigned long fillReadAt         = 0;

float         proximityThresholdCm = 20.0;
float         emptyDepthCm         = LID_EMPTY_CM; // sensor->bottom distance when can is empty (0 %)
bool          fillCalValid         = true;   // default range scale is always usable
bool          customCal            = false;  // true once saved via "c"
float         lastFillPct          = 0.0;
bool          lastFillValid        = false;

bool          liveMode             = false; // streams sensor readings over Serial

// ---------------------------------------------------------------------------
// Distance helpers
// ---------------------------------------------------------------------------
float readDistanceCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return -1.0f;               // no echo
  float cm = duration * 0.0343f / 2.0f;          // cm

  // Reject readings outside the accepted range for this sensor.
  float minCm = (trigPin == FRONT_TRIG_PIN) ? FRONT_MIN_CM : LID_MIN_CM;
  float maxCm = (trigPin == FRONT_TRIG_PIN) ? FRONT_MAX_CM : LID_MAX_CM;
  if (cm < minCm || cm > maxCm) return -1.0f;
  return cm;
}

// Median-ish average of a few valid reads; returns -1 if none valid.
float readDistanceAvg(uint8_t trigPin, uint8_t echoPin) {
  float sum = 0.0f;
  int n = 0;
  for (int i = 0; i < 3; i++) {
    float d = readDistanceCm(trigPin, echoPin);
    if (d > 0.0f) { sum += d; n++; }
    delay(15);
  }
  return (n > 0) ? (sum / n) : -1.0f;
}

float computeFillPct(float distanceCm) {
  if (!fillCalValid || emptyDepthCm <= LID_MIN_CM) return -1.0f;
  float span = emptyDepthCm - LID_MIN_CM;          // distance travelled from 0 % to 100 %
  if (span <= 0.0f) return -1.0f;
  float pct = (emptyDepthCm - distanceCm) / span * 100.0f;
  if (pct < 0.0f)   pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return pct;
}

// ---------------------------------------------------------------------------
// Servo helpers
// ---------------------------------------------------------------------------
void motorWrite(int angle) {
  motor1.write(angle);
  motor2.write(angle);
}

// ---------------------------------------------------------------------------
// LED matrix display (12x8)
// ---------------------------------------------------------------------------
void showFillOnMatrix(float pct, bool valid) {
  char buf[3];
  if (!valid || pct < 0.0f) {
    buf[0] = '-'; buf[1] = '-'; buf[2] = '\0';
  } else {
    int p = (int)(pct + 0.5f);
    if (p < 0)  p = 0;
    if (p > 99) p = 99;                 // two-digit display (100% shows as 99)
    buf[0] = '0' + (p / 10);
    buf[1] = '0' + (p % 10);
    buf[2] = '\0';
  }

  matrix.beginDraw();
  matrix.stroke(0xFFFFFFFF);
  matrix.textFont(Font_5x7);
  matrix.beginText(0, 1, 0xFFFFFF);
  matrix.print(buf);
  matrix.endText();                     // no scroll -> static
  matrix.endDraw();
}

// ---------------------------------------------------------------------------
// Calibration / reporting
// ---------------------------------------------------------------------------
void loadCalibration() {
  uint8_t magic = EEPROM.read(EEPROM_MAGIC_ADDR);
  if (magic == CAL_MAGIC) {
    EEPROM.get(EEPROM_DEPTH_ADDR, emptyDepthCm);
    if (emptyDepthCm > LID_MIN_CM + 1.0f && emptyDepthCm <= LID_MAX_CM) {
      fillCalValid = true;
      customCal    = true;
    } else {
      emptyDepthCm = LID_EMPTY_CM;             // saved value unusable -> default scale
    }
  } else {
    emptyDepthCm = LID_EMPTY_CM;
  }
}

void resetCalibration() {
  emptyDepthCm = LID_EMPTY_CM;
  fillCalValid = true;
  customCal    = false;
  EEPROM.write(EEPROM_MAGIC_ADDR, 0);
  Serial.println(F("[cal] reset: 0% = 20 cm, 100% = 2 cm"));
}

void saveCalibration(float depthCm) {
  emptyDepthCm = depthCm;
  fillCalValid = true;
  customCal    = true;
  EEPROM.put(EEPROM_DEPTH_ADDR, emptyDepthCm);
  EEPROM.write(EEPROM_MAGIC_ADDR, CAL_MAGIC);
}

void calibrateEmptyDepth() {
  if (lidIsOpen) {
    Serial.println(F("[cal] close the lid first, then run 'c' again"));
    return;
  }
  float d = readDistanceAvg(LID_TRIG_PIN, LID_ECHO_PIN);
  if (d <= 0.0f || d > 400.0f) {
    Serial.println(F("[cal] no valid reading - check the lid sensor"));
    return;
  }
  saveCalibration(d);
  Serial.print(F("[cal] empty-can depth saved: "));
  Serial.print(emptyDepthCm, 1);
  Serial.println(F(" cm (stored in EEPROM)"));
}

void printFill(float distanceCm, float pct) {
  Serial.print(F("[fill] distance="));
  Serial.print(distanceCm, 1);
  Serial.print(F(" cm  ->  "));
  if (pct < 0.0f) {
    Serial.println(F("not calibrated (run 'c' with an empty can)"));
  } else {
    Serial.print(pct, 0);
    Serial.println(F(" % full"));
  }
}

void printCurrentFill() {
  if (lidIsOpen) {
    Serial.println(F("[fill] lid is open - close it for an accurate reading"));
  }
  float d = readDistanceAvg(LID_TRIG_PIN, LID_ECHO_PIN);
  if (d <= 0.0f) {
    Serial.println(F("[fill] no valid reading"));
    return;
  }
  float pct = computeFillPct(d);
  lastFillPct = pct;
  lastFillValid = (pct >= 0.0f);
  showFillOnMatrix(lastFillPct, lastFillValid);
  printFill(d, pct);
}

void printInfo() {
  Serial.println(F("----- info -----"));
  if (fillCalValid) {
    Serial.print(F("empty depth : "));
    Serial.print(emptyDepthCm, 1);
    Serial.println(customCal ? F(" cm (calibrated)") : F(" cm (default)"));
  } else {
    Serial.println(F("empty depth : NOT calibrated (run 'c')"));
  }
  Serial.print(F("threshold   : "));
  Serial.print(proximityThresholdCm, 0);
  Serial.println(USE_POT_FOR_THRESHOLD ? F(" cm (from pot)") : F(" cm (fixed)"));

  float front = readDistanceCm(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
  Serial.print(F("front dist  : "));
  if (front > 0.0f) {
    Serial.print(front, 1); Serial.print(F(" cm ("));
    Serial.print(rangePct(front, FRONT_MAX_CM, FRONT_MIN_CM), 0); Serial.println(F("%)"));
  }
  else              { Serial.println(F("no echo")); }
  delay(SENSOR_GAP_MS);

  float lid = readDistanceCm(LID_TRIG_PIN, LID_ECHO_PIN);
  Serial.print(F("lid dist    : "));
  if (lid > 0.0f) {
    Serial.print(lid, 1); Serial.print(F(" cm ("));
    Serial.print(computeFillPct(lid), 0); Serial.println(F("% full)"));
  }
  else            { Serial.println(F("no echo")); }

  Serial.print(F("last fill   : "));
  if (lastFillValid) { Serial.print(lastFillPct, 0); Serial.println(F(" %")); }
  else               { Serial.println(F("unknown")); }
  Serial.println(F("----------------"));
}

// Position inside a range as a percentage: farCm -> 0 %, nearCm -> 100 %.
// Returns -1 if there is no valid distance.
float rangePct(float distanceCm, float farCm, float nearCm) {
  if (distanceCm <= 0.0f) return -1.0f;
  float p = (farCm - distanceCm) / (farCm - nearCm) * 100.0f;
  if (p < 0.0f)   p = 0.0f;
  if (p > 100.0f) p = 100.0f;
  return p;
}

// Streams both sensors as distance + percentage while live mode is on ('l').
//   front % : 5 cm = 0 %,  2 cm = 100 %  (how deep into the trigger range)
//   lid  %  : empty depth = 0 %, 2 cm = 100 % (how full the can is)
// The LED matrix and last fill value follow the lid percentage live
// (only while the lid is closed, so an open lid does not give a false reading).
void printLive() {
  static unsigned long lastPrint = 0;
  unsigned long now = millis();
  if (!liveMode || now - lastPrint < LIVE_INTERVAL_MS) return;
  lastPrint = now;

  float front = readDistanceCm(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
  delay(SENSOR_GAP_MS);
  float lid = readDistanceCm(LID_TRIG_PIN, LID_ECHO_PIN);

  float frontPct = rangePct(front, FRONT_MAX_CM, FRONT_MIN_CM);
  float fillPct  = (lid > 0.0f) ? computeFillPct(lid) : -1.0f;

  Serial.print(F("front: "));
  if (front > 0.0f) {
    Serial.print(front, 1); Serial.print(F(" cm ("));
    Serial.print(frontPct, 0); Serial.print(F("%)"));
  } else {
    Serial.print(F("no echo"));
  }

  Serial.print(F("   |   lid: "));
  if (lid > 0.0f) {
    Serial.print(lid, 1); Serial.print(F(" cm ("));
    Serial.print(fillPct, 0); Serial.println(F("% full)"));
  } else {
    Serial.println(F("no echo"));
  }

  if (!lidIsOpen && fillPct >= 0.0f) {
    lastFillPct   = fillPct;
    lastFillValid = true;
    showFillOnMatrix(lastFillPct, lastFillValid);
  }
}

// ---------------------------------------------------------------------------
// Sequencer
// ---------------------------------------------------------------------------
void startEject(unsigned long now) {
  motorWrite(MOTOR_FWD);
  ejectState     = EJ_UP;
  ejectActive    = true;
  ejectStartTime = now;
  ejectPhaseTime = now;
  Serial.print(F("[eject] bag up for "));
  Serial.print(UP_MS);
  Serial.println(F(" ms"));
}

void openLid(unsigned long now) {
  lidServo.write(LID_OPEN_ANGLE);
  lidIsOpen   = true;
  lidOpenedAt = now;
  handCleared     = false;   // hand is still in front: must leave first
  handAbsentSince = 0;
  Serial.println(F("[lid] opening"));

  // Eject only when BOTH the lid just opened AND the can is full enough.
  if (lastFillValid && fillCalValid && lastFillPct >= EJECT_FILL_PCT) {
    startEject(now);
  }
}

void updateEject(unsigned long now) {
  switch (ejectState) {
    case EJ_UP:
      if (now - ejectPhaseTime >= UP_MS) {
        motorWrite(MOTOR_STOP);
        ejectState     = EJ_HOLD;
        ejectPhaseTime = now;
        Serial.println(F("[eject] holding - remove the bag"));
      }
      break;

    case EJ_HOLD:
      if (now - ejectStartTime >= EJECT_TOTAL_MS) {
        motorWrite(MOTOR_REV);
        ejectState     = EJ_DOWN;
        ejectPhaseTime = now;
        Serial.println(F("[eject] bag down"));
      }
      break;

    case EJ_DOWN:
      if (now - ejectPhaseTime >= UP_MS) {
        motorWrite(MOTOR_STOP);
        ejectState  = EJ_NONE;
        ejectActive = false;
        Serial.println(F("[eject] done"));
      }
      break;

    default:
      break;
  }
}

void closeLid(unsigned long now) {
  lidServo.write(LID_CLOSED_ANGLE);
  lidIsOpen       = false;
  lidClosedAt     = now;
  pendingFillRead = true;
  fillReadAt      = now + FILL_READ_DELAY_MS;
  Serial.println(F("[lid] closing"));
}

// The lid stays open until a hand is placed in front of the sensor AGAIN:
//   1. the hand that opened the lid must first leave (absent for HAND_CLEAR_MS),
//   2. the next time a hand is detected, the lid closes.
// Closing is still deferred while the eject sequence is running.
void updateLidClose(unsigned long now) {
  if (!lidIsOpen || ejectActive) return;

  float d = readDistanceCm(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
  bool handPresent = (d > 0.0f && d <= proximityThresholdCm);

  if (!handCleared) {
    if (handPresent) {
      handAbsentSince = 0;                       // still the original hand
    } else {
      if (handAbsentSince == 0) handAbsentSince = now;
      if (now - handAbsentSince >= HAND_CLEAR_MS) {
        handCleared = true;
        Serial.println(F("[lid] open - wave your hand again to close"));
      }
    }
  } else if (handPresent) {
    closeLid(now);
  }
}

void updatePendingFillRead(unsigned long now) {
  if (pendingFillRead && now >= fillReadAt) {
    pendingFillRead = false;
    float d = readDistanceAvg(LID_TRIG_PIN, LID_ECHO_PIN);
    if (d <= 0.0f) {
      Serial.println(F("[fill] no valid reading"));
      return;
    }
    float pct = computeFillPct(d);
    lastFillPct   = pct;
    lastFillValid = (pct >= 0.0f);
    showFillOnMatrix(lastFillPct, lastFillValid);
    printFill(d, pct);
  }
}

void updateProximityThreshold() {
  if (USE_POT_FOR_THRESHOLD) {
    int raw = analogRead(POT_PIN);               // 0..1023
    proximityThresholdCm = map(raw, 0, 1023, POT_MIN_CM, POT_MAX_CM);
  } else {
    proximityThresholdCm = FIXED_PROXIMITY_CM;   // fixed 20 cm
  }
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') continue;
    switch (c) {
      case 'c': case 'C': calibrateEmptyDepth(); break;
      case 'r': case 'R': printCurrentFill();    break;
      case 'i': case 'I': printInfo();           break;
      case 'x': case 'X': resetCalibration();    break;
      case 'l': case 'L':
        liveMode = !liveMode;
        Serial.println(liveMode ? F("live mode ON (send l to stop)") : F("live mode OFF"));
        break;
      default:
        Serial.println(F("commands: c=calibrate  x=reset cal  r=read fill  i=info  l=live"));
        break;
    }
  }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);

  matrix.begin();
  showFillOnMatrix(-1.0f, false);   // show "--" until the first reading

  pinMode(FRONT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ECHO_PIN, INPUT);
  pinMode(LID_TRIG_PIN, OUTPUT);
  pinMode(LID_ECHO_PIN, INPUT);
  pinMode(POT_PIN, INPUT);

  digitalWrite(FRONT_TRIG_PIN, LOW);
  digitalWrite(LID_TRIG_PIN, LOW);

  lidServo.attach(LID_SERVO_PIN);
  motor1.attach(MOTOR1_PIN);
  motor2.attach(MOTOR2_PIN);

  lidServo.write(LID_CLOSED_ANGLE);
  motorWrite(MOTOR_STOP);

  loadCalibration();

  Serial.println(F("Smart Trash Can ready."));
  if (fillCalValid) {
    Serial.print(F("empty depth: "));
    Serial.print(emptyDepthCm, 1);
    Serial.println(F(" cm"));
  } else {
    Serial.println(F("not calibrated - put an EMPTY can, close lid, send 'c'"));
  }
  Serial.println(F("commands: c=calibrate  x=reset cal  r=read fill  i=info  l=live"));

  delay(500);          // let the lid settle at the closed position
  printCurrentFill();  // seed lastFillPct so eject works on the first opening
}

void loop() {
  unsigned long now = millis();

  updateProximityThreshold();

  // Front proximity -> open the lid (only when idle and past the reopen cooldown).
  if (!lidIsOpen && !ejectActive && (now - lidClosedAt) >= REOPEN_COOLDOWN_MS) {
    float d = readDistanceCm(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
    if (d > 0.0f && d <= proximityThresholdCm) {
      openLid(now);
    }
  }

  updateEject(now);
  updateLidClose(now);
  updatePendingFillRead(now);
  handleSerial();
  printLive();

  delay(20);
}
