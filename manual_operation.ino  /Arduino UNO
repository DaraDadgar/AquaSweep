// =============================================================================
// AquaSweep — manual_operation.ino
// =============================================================================
// Runs on the Arduino Uno. Responsible for:
//   - Reading RC input from the FlySky receiver via iBUS protocol
//   - Driving the left and right thrusters (ESCs) with differential mixing
//   - Controlling the debris conveyor DC motor (speed + direction)
//   - Detecting bin overflow via two JSN-SR04T ultrasonic sensors
//   - Outputting a JSON telemetry packet over Serial at 10 Hz to the Pi
//
// Two compile-time modes (set DEBUG_MODE below):
//   0 = RUN   — iBUS on D0 (hardware Serial), JSON telemetry output, motors live
//   1 = DEBUG — iBUS on D8 (SoftwareSerial), Serial Monitor prints, motors neutral
// =============================================================================

#define DEBUG_MODE 0
// DEBUG_MODE = 0  -> RUN: iBUS on D0 (hardware Serial), no Serial prints
// DEBUG_MODE = 1  -> DEBUG: iBUS on D8 (SoftwareSerial), Serial Monitor prints, motors held neutral

#include <Servo.h>

#if DEBUG_MODE
  #include <SoftwareSerial.h>
#endif

// ===================== Thrusters (ESCs) =====================
Servo leftMotor;
Servo rightMotor;

const int LEFT_ESC_PIN  = 9;  // D9
const int RIGHT_ESC_PIN = 3;  // D3

// Measured Neutral positions for the motors
const int LEFT_NEUTRAL_US  = 1500;
const int RIGHT_NEUTRAL_US = 1500;

// ===================== Conveyor DC Motor Driver =====================
#define CONV_RPWM 5 //D5
#define CONV_LPWM 11 //D11
#define CONV_REN  2 //D2
#define CONV_LEN  13 //D13

//constant speed set to the DC motor
// this can be increased dynamically during execution
const int CONV_SPEED_MIN = 40;

static int currentConvDir = 0;      // 0=FORWARD, 1=BACKWARD
static int requestedConvDir = 0;
static bool directionChanging = false;
static unsigned long dirChangeStartMs = 0;

// ===================== Ultrasonic (JSN-SR04T) =====================
#define US1_TRIG 6 //D6
#define US1_ECHO 7 //D7
#define US2_TRIG 4 //D4
#define US2_ECHO A1   //used as a digital pin 

//Threshold values
float FULL_ON_CM  = 45.0;
float FULL_OFF_CM = 55.0;
float US_MIN_CM = 2.0;
float US_MAX_CM = 200.0;

float us1_cm = -1;
float us2_cm = -1;
bool  full1  = false;
bool  full2  = false;
bool  overflow = false;

uint8_t full1NearCount = 0;
uint8_t full1FarCount  = 0;
uint8_t full2NearCount = 0;
uint8_t full2FarCount  = 0;

//onfidence count for US measurements
const uint8_t FULL_CONF_COUNT = 10; 

// ===================== FlySky iBUS =====================
static const uint8_t IBUS_FRAME_SIZE   = 32;
static const uint8_t IBUS_MAX_CHANNELS = 10;

uint8_t  ibusBuf[IBUS_FRAME_SIZE];
uint8_t  ibusIdx = 0;
uint16_t ch[IBUS_MAX_CHANNELS];

// 0-based indices
static const uint8_t CH_THROTTLE    = 2; // CH3
static const uint8_t CH_STEER       = 3; // CH4
static const uint8_t CH_ESTOP       = 4; // CH5
static const uint8_t CH_CONV_SPEED = 5; // CH6
static const uint8_t CH_CONVEYOR    = 7; // CH8
static const uint8_t CH_CONV_DIR_SW = 6; // CH7

#if DEBUG_MODE
  // DEBUG wiring: iBUS signal -> D8
  static const uint8_t IBUS_RX_PIN_DEBUG = 8;
  static const uint8_t IBUS_TX_PIN_DEBUG = 12; // unused
  SoftwareSerial ibusSS(IBUS_RX_PIN_DEBUG, IBUS_TX_PIN_DEBUG);
#endif

// We'll read iBUS from a Stream* so it works with Serial OR SoftwareSerial
Stream* ibusPort = nullptr;

// ===================== iBUS Parser =====================
// Reads bytes from the iBUS port one at a time, assembles a full 32-byte frame,
// validates the checksum, and unpacks all 10 channel values into ch[].
// Returns true only when a complete valid frame has been received.
bool readIbusFrame() {
  while (ibusPort && ibusPort->available()) {
    uint8_t b = (uint8_t)ibusPort->read();

    // iBUS header 0x20 0x40
    if (ibusIdx == 0 && b != 0x20) continue;
    if (ibusIdx == 1 && b != 0x40) { ibusIdx = 0; continue; }

    ibusBuf[ibusIdx++] = b;

    if (ibusIdx >= IBUS_FRAME_SIZE) {
      ibusIdx = 0;

      uint16_t checksum = 0xFFFF;
      for (uint8_t i = 0; i < IBUS_FRAME_SIZE - 2; i++) checksum -= ibusBuf[i];

      uint16_t rxChecksum = (uint16_t)ibusBuf[IBUS_FRAME_SIZE - 2] |
                            ((uint16_t)ibusBuf[IBUS_FRAME_SIZE - 1] << 8);

      if (checksum != rxChecksum) return false;

      for (uint8_t i = 0; i < IBUS_MAX_CHANNELS; i++) {
        uint8_t lo = ibusBuf[2 + i * 2];
        uint8_t hi = ibusBuf[3 + i * 2];
        ch[i] = (uint16_t)lo | ((uint16_t)hi << 8);
      }
      return true;
    }
  }
  return false;
}

// ===================== Helpers =====================
// Converts a raw iBUS value (1000–2000 µs) to a 0–100% integer
int toPercent0to100(uint16_t v) {
  if (v < 1000) v = 1000;
  if (v > 2000) v = 2000;
  return (int)((v - 1000) / 10); // 1000->0, 1500->50, 2000->100
}

// Converts a raw iBUS value to a binary 0 or 1 (threshold at 1500)
int toBool01(uint16_t v) {
  return (v > 1500) ? 1 : 0;
}

// Snaps the value to 50 (neutral) if it falls within the deadzone band
int applyDeadzoneCentered50(int pct, int dz) {
  if (pct >= (50 - dz) && pct <= (50 + dz)) return 50;
  return pct;
}

bool approxEqualInt(int a, int b, int tol) {
  return (a >= b - tol) && (a <= b + tol);
}

int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

//converts the pct values back to ESC based values (1000-2000)
int pctToEscUs(int pct) {
  pct = constrain(pct, 0, 100);
  return map(pct, 0, 100, 1100, 1900);
}

//Slew step acceleration function
int slewStepUs(int currentValue, int targetValue, int stepUp, int stepDown) {
  if (targetValue > currentValue) {
    currentValue += stepUp;
    if (currentValue > targetValue) currentValue = targetValue;
  } else if (targetValue < currentValue) {
    currentValue -= stepDown;
    if (currentValue < targetValue) currentValue = targetValue;
  }
  return currentValue;
}

// Ultrasonic
// Fires a trigger pulse and measures the echo return time.
// Returns distance in cm, or -1.0 if the reading is out of valid range or timed out.
float readUltrasonicCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);
  if (duration == 0) return -1.0;

  float cm = (duration * 0.0343f) / 2.0f;
  if (cm < US_MIN_CM || cm > US_MAX_CM) return -1.0;
  return cm;
}

//Confidence based hysteresis overflow detection
// Requires FULL_CONF_COUNT consecutive "close" readings to declare full,
// and FULL_CONF_COUNT consecutive "far" readings to clear it.
// This prevents false positives from water splashing near the sensor.
bool updateFullStateConfidence(
  bool currentFull,
  float cm,
  uint8_t &nearCount,
  uint8_t &farCount
) {
  // Ignore invalid readings
  if (cm < 0) return currentFull;

  if (!currentFull) {
    // Need 10 consecutive "close" readings to trigger full
    if (cm <= FULL_ON_CM) {
      nearCount++;
      farCount = 0;

      //if the nearcount reaches the thershold set by the confidence count, set it back to zero and return true for overflow
      if (nearCount >= FULL_CONF_COUNT) {
        nearCount = 0;
        return true;
      }
    } else {
      nearCount = 0;
    }
  } else {
    // Need 10 consecutive "far" readings to clear full
    if (cm >= FULL_OFF_CM) {
      farCount++;
      nearCount = 0;

      if (farCount >= FULL_CONF_COUNT) {
        farCount = 0;
        return false;
      }
    } else {
      farCount = 0;
    }
  }

  return currentFull;
}

// Conveyor helpers
void conveyorStop() {
  analogWrite(CONV_RPWM, 0);
  analogWrite(CONV_LPWM, 0);
}

void conveyorForward(int speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(CONV_RPWM, speed);
  analogWrite(CONV_LPWM, 0);
}

void conveyorBackward(int speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(CONV_RPWM, 0);
  analogWrite(CONV_LPWM, speed);
}

// Maps the raw iBUS channel value (1000–2000) to a PWM speed (0–255).
// Values below CONV_SPEED_MIN are floored to 0 to avoid stalling the motor.
int channelToConvSpeed(uint16_t raw) {
  raw = constrain(raw, 1000, 2000);
  int speed = (int)map(raw, 1000, 2000, 0, 255);
  speed = constrain(speed, 0, 255);
  if (speed < CONV_SPEED_MIN) speed = 0;
  return speed;
}


void setup() {
#if DEBUG_MODE
  // DEBUG: keep USB Serial for prints
  Serial.begin(115200);

  // iBUS on SoftwareSerial (D8)
  ibusSS.begin(115200);
  ibusPort = &ibusSS;

  Serial.println("DEBUG MODE: iBUS on D8 (SoftwareSerial). Motors held NEUTRAL. Conveyor STOP.");
#else
  // RUN: hardware Serial used for iBUS input on D0
  Serial.begin(115200);
  ibusPort = &Serial;
  // No Serial prints in RUN MODE
#endif

  // Thrusters
  leftMotor.attach(LEFT_ESC_PIN);
  rightMotor.attach(RIGHT_ESC_PIN);

  // Send neutral signal for 7 seconds to allow ESCs to complete their arming sequence
  leftMotor.writeMicroseconds(LEFT_NEUTRAL_US);
  rightMotor.writeMicroseconds(RIGHT_NEUTRAL_US);
  delay(7000);

  // Ultrasonics
  pinMode(US1_TRIG, OUTPUT);
  pinMode(US1_ECHO, INPUT);
  pinMode(US2_TRIG, OUTPUT);
  pinMode(US2_ECHO, INPUT);

  // Conveyor
  pinMode(CONV_RPWM, OUTPUT);
  pinMode(CONV_LPWM, OUTPUT);
  pinMode(CONV_REN, OUTPUT);
  pinMode(CONV_LEN, OUTPUT);
  //enabling the right/left pwm drivers
  digitalWrite(CONV_REN, HIGH);
  digitalWrite(CONV_LEN, HIGH);
  conveyorStop();
}

void loop() {
  static unsigned long lastFrameMs  = millis();
  static unsigned long lastChangeMs = millis();

  static int lastThr = 50;
  static int lastStr = 50;
  static int lastConv = 0;
  static int lastEstop = 0;
  static int lastConvDirSw = 0;
  static int lastConvSpeed = 0;
  static int slewedThr = 50;


  // Ultrasonic schedule
  // Sensors are read every 200 ms with a 30 ms gap between the two sensors
  // to prevent acoustic crosstalk between them.
  static unsigned long usTimer = 0;
  const unsigned long US_PERIOD_MS = 200;

  if (millis() - usTimer >= US_PERIOD_MS) {
    usTimer = millis();
    us1_cm = readUltrasonicCm(US1_TRIG, US1_ECHO);
    delay(30);
    us2_cm = readUltrasonicCm(US2_TRIG, US2_ECHO);

    full1 = updateFullStateConfidence(full1, us1_cm, full1NearCount, full1FarCount);
    full2 = updateFullStateConfidence(full2, us2_cm, full2NearCount, full2FarCount);
    overflow = full1 && full2;
  }

  // iBUS read
  // Parse any available bytes. If a complete valid frame is received,
  // update all channel values and record the timestamp for RC-lost detection.
  bool gotFrame = readIbusFrame();
  if (gotFrame) {
    lastFrameMs = millis();

    int thr       = toPercent0to100(ch[CH_THROTTLE]);
    int str       = toPercent0to100(ch[CH_STEER]);
    int estop     = toBool01(ch[CH_ESTOP]);
    int conv      = toBool01(ch[CH_CONVEYOR]);
    int convDirSw = toBool01(ch[CH_CONV_DIR_SW]);
    int convSpeed = channelToConvSpeed(ch[CH_CONV_SPEED]);

    //applying deadzone around the neutral position to avoid motor jitter problems
    thr = applyDeadzoneCentered50(thr, 1);
    str = applyDeadzoneCentered50(str, 2);

    bool changed =
      !approxEqualInt(thr, lastThr, 1) ||
      !approxEqualInt(str, lastStr, 1) ||
      (conv != lastConv) ||
      (estop != lastEstop) ||
      (convDirSw != lastConvDirSw) ||
      !approxEqualInt(convSpeed, lastConvSpeed, 2); 

    if (changed) {
      lastChangeMs = millis();
      lastThr = thr;
      lastStr = str;
      lastConv = conv;
      lastEstop = estop;
      lastConvDirSw = convDirSw;
      lastConvSpeed = convSpeed;
    }
  }

  // If no valid iBUS frame has been received for more than 300 ms, declare RC lost
  bool frameLost = (millis() - lastFrameMs) > 300;

  // ---------- Safety + final commands ----------
  // E-stop or RC loss both force all outputs to neutral/off
  int thrFinal = lastThr;
  int strFinal = lastStr;
  int convFinal = lastConv;
  bool safetyStop = false;

  if (lastEstop) {
    thrFinal = 50;
    strFinal = 50;
    convFinal = 0;
    safetyStop = true;
  }

  if (frameLost) {
    thrFinal = 50;
    strFinal = 50;
    convFinal = 0;
    safetyStop = true;
  }

 // ── Throttle slew ────────────────────────────────────────────
  static unsigned long slewTimer = 0;
  const unsigned long SLEW_PERIOD_MS = 40;  

  if (millis() - slewTimer >= SLEW_PERIOD_MS) {
    slewTimer = millis();
    slewedThr = slewStepUs(slewedThr, thrFinal, 1, 2);
  }

  if (safetyStop) slewedThr = 50;

  // ===================== 2-MOTOR MIXING =====================
  int throttle = (slewedThr - 50) * 2; // ← change thrFinal to slewedThr
  int steer    = (strFinal  - 50) * 2;

  //Differential mixing
  int leftCmd  = throttle + steer;
  int rightCmd = throttle - steer;

  //saturation
  int maxAbs = max(abs(leftCmd), abs(rightCmd));
  if (maxAbs > 100) {
    leftCmd  = (leftCmd  * 100) / maxAbs;
    rightCmd = (rightCmd * 100) / maxAbs;
  }

  int leftPct  = clampInt((leftCmd  / 2) + 50, 0, 100);
  int rightPct = clampInt((rightCmd / 2) + 50, 0, 100);

  int leftUs  = pctToEscUs(leftPct);
  int rightUs = pctToEscUs(rightPct);

  if (slewedThr == 50 && strFinal == 50) {
    leftUs  = LEFT_NEUTRAL_US;
    rightUs = RIGHT_NEUTRAL_US;
  }

#if DEBUG_MODE
  // DEBUG: keep everything SAFE
  leftMotor.writeMicroseconds(LEFT_NEUTRAL_US);
  rightMotor.writeMicroseconds(RIGHT_NEUTRAL_US);
  conveyorStop();
  directionChanging = false;

  // Print debug at 5 Hz
  static unsigned long printTimer = 0;
  if (millis() - printTimer >= 200) {
    printTimer = millis();
    Serial.print("THR="); Serial.print(thrFinal);
    Serial.print(" STR="); Serial.print(strFinal);
    Serial.print(" | L(us)="); Serial.print(leftUs);
    Serial.print(" R(us)="); Serial.print(rightUs);
    Serial.print(" | CONV_SW="); Serial.print(convFinal);
    Serial.print(" DIR_SW="); Serial.print(lastConvDirSw);
    Serial.print(" | US1="); Serial.print(us1_cm, 1);
    Serial.print(" US2="); Serial.print(us2_cm, 1);
    Serial.print(" OVERFLOW="); Serial.print(overflow ? "YES" : "NO");
    Serial.print(" | N1="); Serial.print(full1NearCount);
    Serial.print(" F1="); Serial.print(full1FarCount);
    Serial.print(" | N2="); Serial.print(full2NearCount);
    Serial.print(" F2="); Serial.print(full2FarCount);
    Serial.print(" RC_LOST="); Serial.print(noFramesLost ? "YES" : "NO");
    Serial.print(" ESTOP="); Serial.println(lastEstop ? "1" : "0");
  }
#else
  // RUN: drive thrusters normally

  leftMotor.writeMicroseconds(leftUs);
  rightMotor.writeMicroseconds(rightUs);

  // Conveyor safe switching
  requestedConvDir = lastConvDirSw;

  if (safetyStop || convFinal == 0) {
    conveyorStop(); //stop the conveyor if there is an emergency stop and input set to 0
    directionChanging = false;
  } else {
    if (requestedConvDir != currentConvDir && !directionChanging) {
      conveyorStop(); //stop the conveyor and start the timer
      directionChanging = true;
      dirChangeStartMs = millis();
    }

    if (directionChanging) {
      if (millis() - dirChangeStartMs >= 1000) {
        currentConvDir = requestedConvDir; //once the timer hits the end point, change the direction and reset the variable for future use
        directionChanging = false;
      } else {
        conveyorStop(); //keep the conveyor stopped until the 1 second timer is reached
      }
    }

    //run the conveyor motor in the correct direction based on the user input via the controller
    if (!directionChanging) {
     if (currentConvDir == 1) conveyorBackward(lastConvSpeed);
     else  conveyorForward(lastConvSpeed);
    }
  }

  // ---------- TELEMETRY OUTPUT (JSON @ 10 Hz) ----------
  static unsigned long telemetryTimer = 0;
  if (millis() - telemetryTimer >= 100) {  // 10 Hz
    telemetryTimer = millis();

    Serial.print("{\"thr\":");
    Serial.print(thrFinal);

    Serial.print(",\"str\":");
    Serial.print(strFinal);

    Serial.print(",\"conv\":");
    Serial.print(convFinal);

    Serial.print(",\"conv_dir\":");
    Serial.print(lastConvDirSw);

    Serial.print(",\"conv_speed\":");
    Serial.print(lastConvSpeed);

    Serial.print(",\"estop\":");
    Serial.print(lastEstop);

    Serial.print(",\"overflow\":");
    Serial.print(overflow ? 1 : 0);

    Serial.print(",\"rc_lost\":");
    Serial.print(frameLost ? 1 : 0);

    Serial.print(",\"ts\":");
    Serial.print(millis());

    Serial.println("}");
  }

#endif
}
