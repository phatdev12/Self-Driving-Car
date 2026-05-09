#include "robot_common.h"

const int SENSOR_PINS[SENSOR_COUNT] = {32, 33, 34, 35, 27, 4, 25, 26};
static const int SENSOR_WEIGHTS[SENSOR_COUNT] = {-3500, -2500, -1500, -500, 500, 1500, 2500, 3500};

int sensorValues[SENSOR_COUNT] = {0};
int normalizedValues[SENSOR_COUNT] = {0};
int sensorMin[SENSOR_COUNT] = {4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095};
int sensorMax[SENSOR_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};

// PID tuning values (easy to tune at top of file)
float Kp = 0.12f;
float Ki = 0.0f;
float Kd = 0.35f;
int baseSpeed = 140;

// Bonus: low-pass filter + anti-windup + adaptive speed
static float filteredSensors[SENSOR_COUNT] = {0.0f};
static float integralTerm = 0.0f;
static int lastError = 0;
static unsigned long lastPidMs = 0;
static unsigned long lastLineSeenMs = 0;
static unsigned long lastDebugMs = 0;
static int lastCorrection = 0;
static int lastLeftMotor = 0;
static int lastRightMotor = 0;
static bool filterInitialized = false;
static LineType lastLineType = LINE_NORMAL;
static bool sensorSaturatedAll = false;

static const int SENSOR_ACTIVE_THRESHOLD = 250;
static const int SENSOR_JUNCTION_THRESHOLD = 650;
static const unsigned long CALIBRATION_TIME_MS = 3000;
static const unsigned long SAFETY_LOST_TIMEOUT_MS = 2000;
static const unsigned long DEBUG_INTERVAL_MS = 100;
static const float LOW_PASS_ALPHA = 0.35f;
static const float INTEGRAL_LIMIT = 3500.0f;
static const int TURN_SLOW_SPEED = 95;
static const int STRAIGHT_FAST_SPEED = 175;
static const int CENTER_MAX_SPEED = 255;
static const int CENTER_SENSOR_THRESHOLD = 600;
static const int SHARP_TURN_ERROR = 1800;
static const int STRAIGHT_ERROR = 350;
static const int SEARCH_TURN_SPEED = 120;
static const int ADC_SAMPLES = 4;
static const int MIN_CALIBRATION_SPAN = 120;
static const bool MOTOR_SELF_TEST_ON_BOOT = true;
static const int SATURATED_RAW_THRESHOLD = 4080;
// Set true if left/right branch logs are reversed on your wiring layout.
static const bool SWAP_BRANCH_LOGIC = true;

static const uint32_t PWM_FREQ = 20000;
static const uint8_t PWM_RESOLUTION = 8;
static const uint8_t CH_AIN1 = 0;
static const uint8_t CH_AIN2 = 1;
static const uint8_t CH_BIN1 = 2;
static const uint8_t CH_BIN2 = 3;

static int readSensorAveraged(int pin) {
  long sum = 0;
  for (int i = 0; i < ADC_SAMPLES; ++i) sum += analogRead(pin);
  return (int)(sum / ADC_SAMPLES);
}

static int clampMotor(int speed) {
  if (speed > 255) return 255;
  if (speed < -255) return -255;
  return speed;
}

static int normalizeSensorValue(int raw, int minVal, int maxVal) {
  int span = maxVal - minVal;
  if (span < 10) return 0;
  long scaled = (long)(raw - minVal) * 1000L / span;
  if (scaled < 0) return 0;
  if (scaled > 1000) return 1000;
  return (int)scaled;
}

static const char* lineTypeToString(LineType t) {
  switch (t) {
    case LINE_NORMAL: return "NORMAL";
    case LINE_LEFT_BRANCH: return "LEFT_BRANCH";
    case LINE_RIGHT_BRANCH: return "RIGHT_BRANCH";
    case LINE_CROSS: return "CROSS";
    case LINE_LOST: return "LOST";
    default: return "UNKNOWN";
  }
}

static int adaptiveBaseSpeed(int error) {
  (void)error;
  return CENTER_MAX_SPEED;
}

static void printArray(const int* values) {
  Serial.print("[");
  for (int i = 0; i < SENSOR_COUNT; ++i) {
    Serial.print(values[i]);
    if (i < SENSOR_COUNT - 1) Serial.print(", ");
  }
  Serial.print("]");
}

static void runMotorSelfTest() {
  if (!MOTOR_SELF_TEST_ON_BOOT) return;
  Serial.println("[MOTOR] Self-test: forward/reverse");
  setMotor(110, 110);
  delay(250);
  setMotor(-110, -110);
  delay(250);
  setMotor(0, 0);
}

void setMotor(int leftSpeed, int rightSpeed) {
  leftSpeed = clampMotor(leftSpeed);
  rightSpeed = clampMotor(rightSpeed);

  if (leftSpeed >= 0) {
    ledcWrite(CH_AIN1, leftSpeed);
    ledcWrite(CH_AIN2, 0);
  } else {
    ledcWrite(CH_AIN1, 0);
    ledcWrite(CH_AIN2, -leftSpeed);
  }

  if (rightSpeed >= 0) {
    ledcWrite(CH_BIN1, rightSpeed);
    ledcWrite(CH_BIN2, 0);
  } else {
    ledcWrite(CH_BIN1, 0);
    ledcWrite(CH_BIN2, -rightSpeed);
  }

  lastLeftMotor = leftSpeed;
  lastRightMotor = rightSpeed;
}

void calibrateSensors() {
  Serial.println("\n[CAL] Sensor calibration started (3s)...");
  unsigned long startMs = millis();
  unsigned long lastProgressMs = 0;

  while (millis() - startMs < CALIBRATION_TIME_MS) {
    for (int i = 0; i < SENSOR_COUNT; ++i) {
      int v = readSensorAveraged(SENSOR_PINS[i]);
      if (v < sensorMin[i]) sensorMin[i] = v;
      if (v > sensorMax[i]) sensorMax[i] = v;
    }

    unsigned long now = millis();
    if (now - lastProgressMs >= 250) {
      lastProgressMs = now;
      Serial.print("[CAL] ");
      Serial.print((now - startMs) * 100 / CALIBRATION_TIME_MS);
      Serial.println("%");
    }
  }

  Serial.println("[CAL] Done. Min/Max:");
  for (int i = 0; i < SENSOR_COUNT; ++i) {
    if (sensorMax[i] - sensorMin[i] < MIN_CALIBRATION_SPAN) {
      int center = (sensorMax[i] + sensorMin[i]) / 2;
      sensorMin[i] = max(0, center - MIN_CALIBRATION_SPAN);
      sensorMax[i] = min(4095, center + MIN_CALIBRATION_SPAN);
      Serial.print("[CAL][WARN] S");
      Serial.print(i);
      Serial.println(" span low, expanded.");
    }
    Serial.print("S");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(sensorMin[i]);
    Serial.print(" / ");
    Serial.println(sensorMax[i]);
  }
}

void readSensors() {
  int saturatedCount = 0;
  for (int i = 0; i < SENSOR_COUNT; ++i) {
    int raw = readSensorAveraged(SENSOR_PINS[i]);
    sensorValues[i] = raw;
    if (raw >= SATURATED_RAW_THRESHOLD) saturatedCount++;

    if (!filterInitialized) {
      filteredSensors[i] = (float)raw;
    } else {
      filteredSensors[i] += LOW_PASS_ALPHA * ((float)raw - filteredSensors[i]);
    }

    normalizedValues[i] = normalizeSensorValue((int)filteredSensors[i], sensorMin[i], sensorMax[i]);
  }
  sensorSaturatedAll = (saturatedCount == SENSOR_COUNT);
  filterInitialized = true;
}

LineType detectLineType() {
  int activeCount = 0;
  int leftCount = 0;
  int rightCount = 0;
  int centerCount = 0;

  for (int i = 0; i < SENSOR_COUNT; ++i) {
    if (normalizedValues[i] >= SENSOR_JUNCTION_THRESHOLD) {
      activeCount++;
      if (i <= 2) leftCount++;
      if (i >= 5) rightCount++;
      if (i == 3 || i == 4) centerCount++;
    }
  }

  if (activeCount == 0) return LINE_LOST;
  if (activeCount >= 6) return LINE_CROSS;
  if (leftCount >= 2 && rightCount == 0 && centerCount >= 1) {
    return SWAP_BRANCH_LOGIC ? LINE_RIGHT_BRANCH : LINE_LEFT_BRANCH;
  }
  if (rightCount >= 2 && leftCount == 0 && centerCount >= 1) {
    return SWAP_BRANCH_LOGIC ? LINE_LEFT_BRANCH : LINE_RIGHT_BRANCH;
  }
  if (leftCount >= 2 && rightCount >= 2) return LINE_CROSS;
  return LINE_NORMAL;
}

int calculateLinePosition() {
  if (sensorSaturatedAll) {
    lastLineType = LINE_LOST;
    return 0;
  }

  long weightedSum = 0;
  long activeWeight = 0;

  for (int i = 0; i < SENSOR_COUNT; ++i) {
    if (normalizedValues[i] >= SENSOR_ACTIVE_THRESHOLD) {
      weightedSum += (long)SENSOR_WEIGHTS[i] * normalizedValues[i];
      activeWeight += normalizedValues[i];
    }
  }

  if (activeWeight == 0) {
    lastLineType = LINE_LOST;
    return (lastError >= 0) ? 3500 : -3500;
  }

  int position = (int)(weightedSum / activeWeight);
  lastLineType = detectLineType();
  lastLineSeenMs = millis();
  return position;
}

int computePID(int error) {
  unsigned long now = millis();
  float dt = (now - lastPidMs) * 0.001f;
  if (dt <= 0.0f) dt = 0.001f;
  lastPidMs = now;

  integralTerm += error * dt;
  if (integralTerm > INTEGRAL_LIMIT) integralTerm = INTEGRAL_LIMIT;
  if (integralTerm < -INTEGRAL_LIMIT) integralTerm = -INTEGRAL_LIMIT;

  float derivative = (error - lastError) / dt;
  float correction = Kp * error + Ki * integralTerm + Kd * derivative;
  lastError = error;

  lastCorrection = (int)correction;
  return lastCorrection;
}

void setupLineFollower() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(IR_LED_PIN, OUTPUT);
  digitalWrite(IR_LED_PIN, HIGH);

  for (int i = 0; i < SENSOR_COUNT; ++i) {
    pinMode(SENSOR_PINS[i], INPUT);
    analogSetPinAttenuation(SENSOR_PINS[i], ADC_11db);
  }

  ledcSetup(CH_AIN1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_AIN2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_BIN1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_BIN2, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(AIN1, CH_AIN1);
  ledcAttachPin(AIN2, CH_AIN2);
  ledcAttachPin(BIN1, CH_BIN1);
  ledcAttachPin(BIN2, CH_BIN2);

  setMotor(0, 0);
  runMotorSelfTest();
  calibrateSensors();

  lastPidMs = millis();
  lastLineSeenMs = millis();
  Serial.println("[SYS] Line follower ready.");
}

void loopLineFollower() {
  readSensors();

  int position = calculateLinePosition();
  int error = position;
  int correction = computePID(error);
  int dynamicBase = adaptiveBaseSpeed(error);

  int leftSpeed = dynamicBase + correction;
  int rightSpeed = dynamicBase - correction;

  if (sensorSaturatedAll || lastLineType == LINE_LOST) {
    setMotor(0, 0);
  } else {
    setMotor(leftSpeed, rightSpeed);
  }

  unsigned long now = millis();
  if (now - lastDebugMs >= DEBUG_INTERVAL_MS) {
    lastDebugMs = now;
    Serial.print("RAW=");
    printArray(sensorValues);
    Serial.print(" | NORM=");
    printArray(normalizedValues);
    Serial.print(" | pos=");
    Serial.print(position);
    Serial.print(" | pid=");
    Serial.print(lastCorrection);
    Serial.print(" | motor(L,R)=(");
    Serial.print(lastLeftMotor);
    Serial.print(", ");
    Serial.print(lastRightMotor);
    Serial.print(") | line=");
    Serial.print(lineTypeToString(lastLineType));
    if (sensorSaturatedAll) Serial.print(" | SAFE=SATURATED");
    Serial.println();
  }
}
