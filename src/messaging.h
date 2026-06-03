#pragma once
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "core/UIData.h"

// ============================================================================
// ARM MESSAGING
// ============================================================================

enum class ArmCmdAction {
    SET_TARGET,      // Move to calibrated target (0.0=Out, 100.0=In)
    SET_SPEED,       // Jog velocity (encoder turning)
    STOP,            // Immediate halt (encoder stopped turning)
    SET_POS_OUT,     // Save current physical position as "Out"
    SET_POS_IN,      // Save current physical position as "In"
    CLEAR_CAL,       // Clear both calibration points
    JOG              // Jog target by relative steps
};

struct ArmCommand {
    ArmCmdAction action;
    float value;     // Target percent or speed
};

struct ArmTelemetry {
    float currentPosition;   // Actual physical step position
    float targetPosition;    // Current target
    int posOut;              // Calibrated "Out" position (-1 if not set)
    int posIn;               // Calibrated "In" position (-1 if not set)
    bool isMoving;           // True if motor is actively running
};

extern QueueHandle_t armCmdQueue;
extern QueueHandle_t armTelQueue;

// ============================================================================
// ACTUATOR MESSAGING
// ============================================================================

enum class ActuatorCmdAction {
    SET_TARGET,      // Set target percentage (int 0-100)
    SET_LIMIT_BOT,   // Save current position as bottom limit
    SET_LIMIT_MID,   // Save current position as middle limit
    SET_LIMIT_TOP,   // Save current position as top limit
    CLEAR_LIMIT_BOT, // Clear bottom limit
    CLEAR_LIMIT_MID, // Clear middle limit
    CLEAR_LIMIT_TOP, // Clear top limit
    GET_LIMITS       // Request limit data (for telemetry response)
};

struct ActuatorCommand {
    ActuatorCmdAction action;
    int value;       // Target percent or limit index
    int pwmSpeed;    // Target speed (0-255) for SET_TARGET
};

struct ActuatorTelemetry {
    float currentPercent;    // Actual physical position (float for smooth tracking)
    int targetPercent;       // Current target
    int limits[3];           // [0]=Bot, [1]=Mid, [2]=Top
    bool limitSet[3];        // Whether each limit is configured
};

extern QueueHandle_t actuatorCmdQueue;
extern QueueHandle_t actuatorTelQueue;

// ============================================================================
// MOTOR MESSAGING
// ============================================================================

enum class MotorCmdAction {
    SET_SPEED,       // Set velocity (int steps/s)
    SET_LIMIT_BOT,   // Save current position as bottom limit
    SET_LIMIT_MID,   // Save current position as middle limit
    SET_LIMIT_TOP,   // Save current position as top limit
    CLEAR_LIMIT_BOT, // Clear bottom limit
    CLEAR_LIMIT_MID, // Clear middle limit
    CLEAR_LIMIT_TOP, // Clear top limit
    START_HOMING,    // Initiate homing sequence
    GET_STATE        // Request state (for telemetry response)
};

struct MotorCommand {
    MotorCmdAction action;
    float value;     // Speed or position value
};

struct MotorTelemetry {
    float currentPosition; // Current Z-axis position
    int targetSpeed;       // Current target speed
    bool isHomed;          // Homing complete flag
    bool isHoming;         // Currently homing flag
    float limits[3];       // [0]=Bot, [1]=Mid, [2]=Top
    bool limitSet[3];      // Whether each limit is configured
    bool topEndstopTriggered;
    bool botEndstopTriggered;
};

extern QueueHandle_t motorCmdQueue;
extern QueueHandle_t motorTelQueue;

extern QueueHandle_t lcdDataQueue;
