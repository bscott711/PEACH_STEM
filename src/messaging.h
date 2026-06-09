#pragma once
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "core/UIData.h"

// ============================================================================
// ARM MESSAGING
// ============================================================================

enum class ScraperArmCmdAction {
    SET_TARGET,      // Move to calibrated target (0.0=Out, 100.0=In)
    SET_SPEED,       // Jog velocity (encoder turning)
    STOP,            // Immediate halt (encoder stopped turning)
    SET_POS_OUT,     // Save current physical position as "Out"
    SET_POS_BUFFER,  // Save current physical position as "Buffer"
    SET_POS_IN,      // Save current physical position as "In"
    CLEAR_CAL,       // Clear both calibration points
    JOG              // Jog target by relative steps
};

struct ScraperArmCommand {
    ScraperArmCmdAction action;
    float value;     // Target percent or speed
    int targetSpeed; // Speed for GOTO tracking
};

struct ScraperArmTelemetry {
    float currentPosition;   // Actual physical step position
    float targetPosition;    // Current target
    int posOut;              // Calibrated "Out" position (-1 if not set)
    int posBuffer;           // Calibrated "Buffer" position (-1 if not set)
    int posIn;               // Calibrated "In" position (-1 if not set)
    bool isMoving;           // True if motor is actively running
};

extern QueueHandle_t scraperArmCmdQueue;
extern QueueHandle_t scraperArmTelQueue;

// ============================================================================
// ACTUATOR MESSAGING
// ============================================================================

enum class DishRotationCmdAction {
    SET_TARGET,      // Set target percentage (int 0-100)
    SET_LIMIT_BOT,   // Save current position as bottom limit
    SET_LIMIT_MID,   // Save current position as middle limit
    SET_LIMIT_TOP,   // Save current position as top limit
    CLEAR_LIMIT_BOT, // Clear bottom limit
    CLEAR_LIMIT_MID, // Clear middle limit
    CLEAR_LIMIT_TOP, // Clear top limit
    GET_LIMITS       // Request limit data (for telemetry response)
};

struct DishRotationCommand {
    DishRotationCmdAction action;
    int value;       // Target percent or limit index
    int pwmSpeed;    // Target speed (0-255) for SET_TARGET
};

struct DishRotationTelemetry {
    float currentPercent;    // Actual physical position (float for smooth tracking)
    int targetPercent;       // Current target
    int limits[3];           // [0]=Bot, [1]=Mid, [2]=Top
    bool limitSet[3];        // Whether each limit is configured
};

extern QueueHandle_t dishRotationCmdQueue;
extern QueueHandle_t dishRotationTelQueue;

// ============================================================================
// MOTOR MESSAGING
// ============================================================================

enum class DishLiftCmdAction {
    SET_SPEED,       // Set velocity (int steps/s)
    SET_TARGET,      // Move to absolute position
    SET_LIMIT_BOT,   // Save current position as bottom limit
    SET_LIMIT_MID,   // Save current position as middle limit
    SET_LIMIT_TOP,   // Save current position as top limit
    CLEAR_LIMIT_BOT, // Clear bottom limit
    CLEAR_LIMIT_MID, // Clear middle limit
    CLEAR_LIMIT_TOP, // Clear top limit
    START_HOMING,    // Initiate homing sequence
    GET_STATE        // Request state (for telemetry response)
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
    float limits[3];       // [0]=Bot, [1]=Mid, [2]=Top
    bool limitSet[3];      // Whether each limit is configured
    bool topEndstopTriggered;
    bool botEndstopTriggered;
};

extern QueueHandle_t dishLiftCmdQueue;
extern QueueHandle_t dishLiftTelQueue;

extern QueueHandle_t lcdDataQueue;
