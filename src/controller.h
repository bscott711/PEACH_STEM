#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <stdint.h>

// --- Configuration & Magic Numbers ---
#define MOTOR_SPEED_SCALE_FACTOR 333
#define AUTO_SEQUENCE_SPEED 4995
#define AUTO_SEQUENCE_DURATION_MS 15000
#define SERVO_MIN_PERCENT 0
#define SERVO_CENTER_PERCENT 50
#define SERVO_MAX_PERCENT 100
#define ACTUATOR_STEP_PERCENT 5

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
enum Enc1Menu { MENU_ACT_MAN, MENU_ACT_GOTO_TOP, MENU_ACT_GOTO_MID, MENU_ACT_GOTO_BOT };
enum Enc3Menu { MENU_AUTO, MENU_GOTO_TOP, MENU_GOTO_MID, MENU_GOTO_BOT };


// --- Sequence Engine Types ---
enum SequenceAction {
  SEQ_MOVE_Z,        // Move Z-axis to target position (deterministic)
  SEQ_MOVE_ARM,      // Move Arm
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

// Minimal SystemState - only controller-level state, no subsystem tracking
struct SystemState {
  DeviceMode mode;
  
  // Controller state only (subsystem state moved to Active Nodes)
  Enc1Menu enc1MenuSelection;  // MENU_ACT_MAN, GOTO_TOP, GOTO_MID, GOTO_BOT
  Enc3Menu enc3MenuSelection;
  
  // Collision Detection (shared flag)
  bool collisionDetected;
  uint32_t collisionTimestamp;
};

extern SystemState systemState;
extern SemaphoreHandle_t systemStateMutex;
extern SemaphoreHandle_t encoderStateMutex;
extern EventGroupHandle_t controlEvents;

// Queue handles declared in controller.cpp, extern here for access
extern QueueHandle_t armCmdQueue;
extern QueueHandle_t armTelQueue;
extern QueueHandle_t actuatorCmdQueue;
extern QueueHandle_t actuatorTelQueue;
extern QueueHandle_t motorCmdQueue;
extern QueueHandle_t motorTelQueue;

// --- OTA & WiFi Global States ---
extern volatile bool g_otaActive;
extern volatile int g_otaProgress;
extern const char* g_otaStatus;

void draw_wifiStatus(const char* status, const char* ssid, int attempt, bool failed);
void draw_otaScreen();

void initSystemState();
void saveMotorState();
void saveMotorLimits();
void saveActuatorLimits();

// FreeRTOS task entries
void controller_task(void *pvParameters);
void autonomous_task(void *pvParameters);
void motor_goto_task(void *pvParameters);

// Utility functions
float motorDistanceCalculator(float speed, int timeInMS);
float motorSpeedCalculator(float position, int timeInMS);