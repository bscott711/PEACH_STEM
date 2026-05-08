#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <stdint.h>

// --- Configuration & Magic Numbers ---
#define MOTOR_SPEED_SCALE_FACTOR 333
#define AUTO_SEQUENCE_SPEED 120000
#define AUTO_SEQUENCE_DURATION_MS 15000
#define SERVO_MIN_PERCENT 0
#define SERVO_CENTER_PERCENT 50
#define SERVO_MAX_PERCENT 100
#define ACTUATOR_STEP_PERCENT 10

// --- Event Group Bits ---
#define BIT_HOMING_REQUEST (1 << 0)
#define BIT_AUTO_RUNNING (1 << 1)
#define BIT_AUTO_RESUME (1 << 2)

enum DeviceMode { IDLE, PICKUP_CELL, DROPOFF_CELL };
enum ActuatorDirection { ACT_STOP = 0, ACT_FORWARD, ACT_REVERSE };

struct SystemState {
  DeviceMode mode;

  // Manual Servo Control
  bool servoAdjustMode;
  int servoPercent;
  int servoTargetPercent;

  // Linear Actuator
  ActuatorDirection actuatorDir;
  int actuatorTargetPercent;

  // Stepper Motor
  int actualSpeed;
  int targetSpeed;

  volatile bool isHoming;
  volatile bool sgDiagMode;

  // StallGuard Threshold for live tuning
  int sgThreshold;

  // Motor Position Tracking
  float currentPosition;
  bool isHomed;
  int motorEncoderLimit;

  // Collision Detection
  volatile bool collisionDetected;
  volatile uint32_t collisionTimestamp;
};

extern SystemState systemState;
extern SemaphoreHandle_t systemStateMutex;
extern SemaphoreHandle_t encoderStateMutex;
extern EventGroupHandle_t controlEvents;

void initSystemState();
void saveMotorState();

// FreeRTOS task entries
void controller_task(void *pvParameters);
void autonomous_task(void *pvParameters);

// Utility functions
float motorDistanceCalculator(float speed, int timeInMS);
float motorSpeedCalculator(float position, int timeInMS);