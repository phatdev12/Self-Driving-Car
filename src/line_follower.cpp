#include "robot_common.h"

// ========== Global Data ==========
const int SENSOR_PINS[SENSOR_COUNT] = {32, 33, 34, 35, 27, 4, 25, 26};
int sensorValues[SENSOR_COUNT];
int normalizedValues[SENSOR_COUNT];
int sensorMin[SENSOR_COUNT];
int sensorMax[SENSOR_COUNT];

float Kp = 0.12f;
float Ki = 0.0f;
float Kd = 0.35f;
int baseSpeed = 500;

// Constants
static const int SENSOR_WEIGHTS[SENSOR_COUNT] = {0, 1000, 2000, 3000, 4000, 5000, 6000, 7000};
static const int BLACK_RAW_THRESHOLD = 4000;
static const int SENSOR_ACTIVE_THRESHOLD = 100;
static const int ADC_SAMPLES = 4;
static const unsigned long CALIBRATION_TIME_MS = 3000;

static const uint32_t PWM_FREQ = 20000;
static const uint8_t PWM_RESOLUTION = 8;

static const uint8_t CH_AIN1 = 0;
static const uint8_t CH_AIN2 = 1;
static const uint8_t CH_BIN1 = 2;
static const uint8_t CH_BIN2 = 3;

// PID state
static float integralTerm = 0.0f;
static int lastError = 0;
static unsigned long lastPidMs = 0;
static int lastPosition = 3500;

// ========== Helpers ==========
static int readSensorAveraged(int pin) {
  long sum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    sum += analogRead(pin);
  }
  return (int)(sum / ADC_SAMPLES);
}

static int clampMotor(int speed) {
  if (speed > 255) return 255;
  if (speed < -255) return -255;
  return speed;
}

static int normalizeSensorValue(int raw) {
  raw = constrain(raw, 0, 4095);
  if (raw >= BLACK_RAW_THRESHOLD) return 1000;
  return 0;
}

// ========== Core Functions ==========
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
}

void calibrateSensors() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }

  unsigned long startMs = millis();
  while (millis() - startMs < CALIBRATION_TIME_MS) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
      int v = readSensorAveraged(SENSOR_PINS[i]);
      if (v < sensorMin[i]) sensorMin[i] = v;
      if (v > sensorMax[i]) sensorMax[i] = v;
    }
  }
}

void readSensors() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorValues[i] = readSensorAveraged(SENSOR_PINS[i]);
    normalizedValues[i] = normalizeSensorValue(sensorValues[i]);
  }
}

int calculateLinePosition() {
  long weightedSum = 0;
  long activeWeight = 0;
  
  for (int i = 0; i < SENSOR_COUNT; i++) {
    int v = normalizedValues[i];
    if (v >= SENSOR_ACTIVE_THRESHOLD) {
      weightedSum += (long)SENSOR_WEIGHTS[i] * v;
      activeWeight += v;
    }
  }
  
  if (activeWeight == 0) {
    return lastPosition;
  }
  
  lastPosition = (int)(weightedSum / activeWeight);
  return lastPosition;
}

int computePID(int error) {
  unsigned long now = millis();
  float dt = (now - lastPidMs) * 0.001f;
  if (dt <= 0.0f) dt = 0.001f;
  lastPidMs = now;

  integralTerm += error * dt;
  if (integralTerm > 5000.0f) integralTerm = 5000.0f;
  if (integralTerm < -5000.0f) integralTerm = -5000.0f;

  float derivative = (error - lastError) / dt;
  float corr = Kp * error + Ki * integralTerm + Kd * derivative;
  lastError = error;
  
  return (int)corr;
}

void setupLineFollower() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(IR_LED_PIN, OUTPUT);
  digitalWrite(IR_LED_PIN, HIGH);

  for (int i = 0; i < SENSOR_COUNT; i++) {
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
  delay(100);
  
  // Motor test
  Serial.println("[TEST] LEFT motor 3s at 200...");
  setMotor(200, 0);
  delay(3000);
  
  Serial.println("[TEST] RIGHT motor 3s at 200...");
  setMotor(0, 200);
  delay(3000);
  
  Serial.println("[TEST] BOTH motors 3s at 200...");
  setMotor(200, 200);
  delay(3000);
  
  setMotor(0, 0);
  delay(500);
  Serial.println("[TEST] Done. Starting line follow...");
  
  calibrateSensors();
  
  lastPidMs = millis();
}

void loopLineFollower() {
  readSensors();
  
  int position = calculateLinePosition();
  int error = position - 3500;
  int correction = computePID(error);

  int left = baseSpeed + correction;
  int right = baseSpeed - correction;

  setMotor(left, right);
  
  // Debug output
  static unsigned long lastDbg = 0;
  if (millis() - lastDbg > 100) {
    lastDbg = millis();
    
    Serial.print("RAW=[");
    for (int i = 0; i < SENSOR_COUNT; i++) {
      Serial.print(sensorValues[i]);
      if (i < SENSOR_COUNT - 1) Serial.print(",");
    }
    Serial.print("] NORM=[");
    for (int i = 0; i < SENSOR_COUNT; i++) {
      Serial.print(normalizedValues[i]);
      if (i < SENSOR_COUNT - 1) Serial.print(",");
    }
    Serial.print("] pos=");
    Serial.print(position);
    Serial.print(" err=");
    Serial.print(error);
    Serial.print(" pid=");
    Serial.print(correction);
    Serial.print(" L=");
    Serial.print(left);
    Serial.print(" R=");
    Serial.println(right);
  }
}
