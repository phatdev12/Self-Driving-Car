#ifndef ROBOT_COMMON_H
#define ROBOT_COMMON_H

#include <Arduino.h>

constexpr uint8_t SENSOR_COUNT = 8;

// QTR-11A pins (analog)
extern const int SENSOR_PINS[SENSOR_COUNT];
#define IR_LED_PIN 13

// DRV8833 motor pins
#define AIN1 12
#define AIN2 14
#define BIN1 18
#define BIN2 19

enum LineType {
  LINE_NORMAL,
  LINE_LEFT_BRANCH,
  LINE_RIGHT_BRANCH,
  LINE_CROSS,
  LINE_LOST
};

// Shared runtime data for quick tuning/debug.
extern int sensorValues[SENSOR_COUNT];
extern int normalizedValues[SENSOR_COUNT];
extern int sensorMin[SENSOR_COUNT];
extern int sensorMax[SENSOR_COUNT];

extern float Kp;
extern float Ki;
extern float Kd;
extern int baseSpeed;

void setupLineFollower();
void loopLineFollower();

void readSensors();
void calibrateSensors();
int calculateLinePosition();
int computePID(int error);
LineType detectLineType();
void setMotor(int leftSpeed, int rightSpeed);

#endif
 