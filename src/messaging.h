#pragma once
#include "core/UIData.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <stdint.h>

// ============================================================================
// ARM MESSAGING
// ============================================================================

enum class ScraperArmCmdAction {
  SET_TARGET,      // Move to target
  SET_SPEED,       // Jog velocity
  STOP,            // Immediate halt
  SET_POS_CLEAR,   // Save current physical position as "Clear"
  SET_POS_SCRAPE,  // Save current physical position as "Scrape"
  CLEAR_CAL,       // Clear both calibration points
  JOG              // Jog target by relative steps
};

struct ScraperArmCommand {
  ScraperArmCmdAction action;
  float value;     // Target position or speed
  int targetSpeed; // Speed for GOTO tracking
};

struct ScraperArmTelemetry {
  float currentPosition; // Actual physical step position
  float targetPosition;  // Current target
  int posClear;          // Calibrated "Clear" position (-1 if not set)
  int posScrape;         // Calibrated "Scrape" position (-1 if not set)
  bool isMoving;         // True if motor is actively running
};

extern QueueHandle_t scraperArmCmdQueue;
extern QueueHandle_t scraperArmTelQueue;

// ============================================================================
// ROTATION MESSAGING
// ============================================================================

enum class DishRotationCmdAction {
  SET_TARGET,      // Set target absolute steps
  SET_SPEED,       // Jog velocity
  STOP,            // Immediate halt
  JOG,             // Jog relative steps
  ZERO_POS         // Set current position as 0
};

struct DishRotationCommand {
  DishRotationCmdAction action;
  float value;     // Target steps or speed
  int targetSpeed; // Speed for GOTO tracking
};

struct DishRotationTelemetry {
  float currentPosition; // Actual physical step position
  float targetPosition;  // Current target
  bool isMoving;         // True if motor is actively running
};

extern QueueHandle_t dishRotationCmdQueue;
extern QueueHandle_t dishRotationTelQueue;

// ============================================================================
// LIFT MESSAGING
// ============================================================================

enum class DishLiftCmdAction {
  SET_SPEED,       // Set velocity
  SET_TARGET,      // Move to absolute position
  SET_POS_HOME,    // Save current position as Home (Lower)
  SET_POS_TILT,    // Save current position as Tilt (Upper)
  CLEAR_CAL,       // Clear limits
  START_HOMING,    // Initiate homing sequence
  GET_STATE        // Request state
};

struct DishLiftCommand {
  DishLiftCmdAction action;
  float value;     // Speed or position value
  int targetSpeed; // Optional speed for GOTO
};

struct DishLiftTelemetry {
  float currentPosition; // Current Z-axis position
  int targetSpeed;       // Current target speed
  bool isHomed;          // Homing complete flag
  bool isHoming;         // Currently homing flag
  float posHome;         // Home position
  float posTilt;         // Tilt position
  bool posHomeSet;       // Is Home set?
  bool posTiltSet;       // Is Tilt set?
};

extern QueueHandle_t dishLiftCmdQueue;
extern QueueHandle_t dishLiftTelQueue;

extern QueueHandle_t lcdDataQueue;
