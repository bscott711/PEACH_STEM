#pragma once
#include "core/UIData.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <stdint.h>

#pragma once
#include "core/UIData.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <stdint.h>

// ============================================================================
// GENERIC AXIS MESSAGING
// ============================================================================

enum class AxisCmdAction {
  SET_SPEED,       // Jog velocity
  SET_TARGET,      // Move to target
  SET_POS_A,       // Save current physical position as "Limit A" (Clear/Home)
  SET_POS_B,       // Save current physical position as "Limit B" (Scrape/Tilt)
  CLEAR_CAL,       // Clear calibration limits
  JOG,             // Jog target by relative steps
  ZERO_POS,        // Set current position as 0 (used for continuous)
  START_HOMING,    // Start homing sequence
  SET_SG_THRESHOLD,// Set StallGuard threshold
  SET_CURRENT      // Set motor run current percentage
};

struct AxisCommand {
  AxisCmdAction action;
  float value;     // Target position or speed
  int targetSpeed; // Speed for GOTO tracking
  uint8_t currentPercent; // Motor run current percentage
};

struct AxisTelemetry {
  float currentPosition; // Actual physical step position
  float targetPosition;  // Current target
  int targetSpeed;       // Target speed
  float posA;            // Calibrated "Limit A" position (-1 if not set)
  float posB;            // Calibrated "Limit B" position (-1 if not set)
  bool posASet;          // Is Limit A set?
  bool posBSet;          // Is Limit B set?
  bool isMoving;         // True if motor is actively running
  bool isHoming;
  bool isHomed;
  uint16_t sgResult;
  uint32_t timestamp;    // Telemetry timestamp (FreeRTOS ticks * portTICK_PERIOD_MS)
};

// Queue definitions
extern QueueHandle_t scraperArmCmdQueue;
extern QueueHandle_t scraperArmTelQueue;

extern QueueHandle_t dishRotationCmdQueue;
extern QueueHandle_t dishRotationTelQueue;

extern QueueHandle_t dishLiftCmdQueue;
extern QueueHandle_t dishLiftTelQueue;

extern QueueHandle_t lcdDataQueue;

