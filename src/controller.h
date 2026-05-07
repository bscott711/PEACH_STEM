#pragma once
#include <stdint.h>

enum DeviceMode { IDLE, PICKUP_CELL, DROPOFF_CELL };

enum ActuatorDirection { ACT_STOP = 0, ACT_FORWARD, ACT_REVERSE };

struct SystemState {
  DeviceMode mode;
  bool busy;

  // Manual Servo Control
  bool servoAdjustMode;
  int servoPercent;
  int servoTargetPercent;

  // Linear Actuator
  ActuatorDirection actuatorDir;

  // Servo Motor
  int actualSpeed;
  int targetSpeed;

  // Actuator
  int actuatorTargetPercent;

  volatile bool triggerHoming;
  volatile bool isHoming;
  volatile bool sgDiagMode;

  // StallGuard Threshold for live tuning
  int sgThreshold;

  // Motor Position Tracking
  double currentPosition;
  bool isHomed;
  int motorEncoderLimit;

  // Collision Detection
  volatile bool collisionDetected;
  volatile uint32_t collisionTimestamp;
};

extern SystemState systemState;

// FreeRTOS task entry
void controller_task(void *pvParameters);