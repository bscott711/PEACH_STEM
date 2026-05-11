#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <stdint.h>

// --- Configuration & Magic Numbers ---
#define MOTOR_SPEED_SCALE_FACTOR 333
#define AUTO_SEQUENCE_SPEED 4995
#define AUTO_SEQUENCE_DURATION_MS 15000
#define SERVO_MIN_PERCENT 0
#define SERVO_CENTER_PERCENT 50
#define SERVO_MAX_PERCENT 100
#define ACTUATOR_STEP_PERCENT 10

// Z-axis position targets (in currentPosition units)
// Derived from speed=120000, time=15s, factor=1.372e-6 ≈ 2.47
#define Z_CLEARANCE_POS 2.5f
#define Z_TUBE_POS 0.0f

// --- Event Group Bits ---
#define BIT_HOMING_REQUEST (1 << 0)
#define BIT_AUTO_RUNNING (1 << 1)
#define BIT_AUTO_RESUME (1 << 2)
#define BIT_ESTOP_REQUEST (1 << 3)

enum DeviceMode { IDLE, PICKUP_CELL, DROPOFF_CELL };
enum ActuatorDirection { ACT_STOP = 0, ACT_FORWARD, ACT_REVERSE };
enum ServoCalibrationStep { CAL_OFF, CAL_SET_START, CAL_SET_CENTER };
enum Enc1Menu { MENU_ACT_AUTO, MENU_ACT_GOTO_TOP, MENU_ACT_GOTO_MID, MENU_ACT_GOTO_BOT };
enum Enc3Menu { MENU_AUTO, MENU_GOTO_TOP, MENU_GOTO_MID, MENU_GOTO_BOT };


// --- Sequence Engine Types ---
enum SequenceAction {
  SEQ_MOVE_Z,        // Move Z-axis to target position (deterministic)
  SEQ_MOVE_SERVO,    // Set servo to target percent
  SEQ_MOVE_ACTUATOR, // Set actuator to target percent
  SEQ_WAIT_MS,       // Interruptible delay (target = milliseconds)
  SEQ_WAIT_USER      // Wait for user button press to continue
};

struct SequenceStep {
  SequenceAction action;
  int target;          // Position/percent/ms depending on action
  float zTarget;       // Z-position target (only for SEQ_MOVE_Z)
  const char *message; // LCD message (NULL = no update)
};

struct SystemState {
  DeviceMode mode;

  // Manual Servo Control
  bool servoAdjustMode;
  int servoPercent;
  int servoTargetPercent;

  // Servo Calibration (NVS-persisted)
  int servoCalStart;  // Raw percent for "start" position
  int servoCalCenter; // Raw percent for "center" position
  ServoCalibrationStep servoCalStep;

  // Linear Actuator
  ActuatorDirection actuatorDir;
  int actuatorTargetPercent;
  int actuatorPercent; // Actual physical position (updated by actuator_task)

  // Actuator Limits (0=Bot, 1=Mid, 2=Top)
  int actuatorLimits[3];
  bool actuatorLimitSet[3];
  Enc1Menu enc1MenuSelection;

  // Stepper Motor
  int actualSpeed;
  int targetSpeed;

  bool isHoming;
  bool sgDiagMode;

  // StallGuard Threshold for live tuning
  int sgThreshold;

  // Motor Position Tracking
  float currentPosition;
  bool isHomed;
  int motorEncoderLimit;

  // Motor Limits (0=Bot, 1=Mid, 2=Top)
  float motorLimits[3];
  bool motorLimitSet[3];

  // Encoder 3 Menu State
  Enc3Menu enc3MenuSelection;

  // Collision Detection
  bool collisionDetected;
  uint32_t collisionTimestamp;
};

extern SystemState systemState;
extern SemaphoreHandle_t systemStateMutex;
extern SemaphoreHandle_t encoderStateMutex;
extern EventGroupHandle_t controlEvents;

void initSystemState();
void saveMotorState();
void saveMotorLimits();
void saveActuatorLimits();
void saveServoCalibration();

// FreeRTOS task entries
void controller_task(void *pvParameters);
void autonomous_task(void *pvParameters);
void motor_goto_task(void *pvParameters);

// Utility functions
float motorDistanceCalculator(float speed, int timeInMS);
float motorSpeedCalculator(float position, int timeInMS);