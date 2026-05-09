#ifndef ROBOT_COMMON_H
#define ROBOT_COMMON_H

#include <Arduino.h>

const uint8_t SENSOR_COUNT = 8;

// QTR sensor pins (analog)
extern const int SENSOR_PINS[SENSOR_COUNT];
#define IR_LED_PIN 13

// Motor driver pins
#define AIN1 12
#define AIN2 14
#define BIN1 18
#define BIN2 19

// Global sensor data
extern int sensorValues[SENSOR_COUNT];
extern int normalizedValues[SENSOR_COUNT];
extern int sensorMin[SENSOR_COUNT];
extern int sensorMax[SENSOR_COUNT];

// PID gains
extern float Kp;
extern float Ki;
extern float Kd;
extern int baseSpeed;

// Function prototypes
void setupLineFollower();
void loopLineFollower();
void readSensors();
void calibrateSensors();
int calculateLinePosition();
int computePID(int error);
void setMotor(int leftSpeed, int rightSpeed);

#endif  // ROBOT_COMMON_H
 