#pragma once
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// ============================================================================
// SERVO MESSAGING
// ============================================================================

enum class ServoCmdAction {
    SET_TARGET,      // Set target position (float percent)
    ACTIVATE,        // Enable servo output
    DEACTIVATE,      // Disable servo output (limp)
    SET_CAL_START,   // Save current position as calibration start
    SET_CAL_CENTER,  // Save current position as calibration center
    GET_CAL_DATA     // Request calibration data (for telemetry response)
};

struct ServoCommand {
    ServoCmdAction action;
    float value;     // Target percent or calibration value
};

struct ServoTelemetry {
    float currentPercent;    // Actual physical position
    float targetPercent;     // Current target
    bool isActive;           // Whether servo is enabled
    int calStart;            // Calibration start (-1 if not set)
    int calCenter;           // Calibration center (-1 if not set)
};

extern QueueHandle_t servoCmdQueue;
extern QueueHandle_t servoTelQueue;

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
    SET_SG_THRESHOLD,// Set StallGuard threshold
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
    int sgThreshold;       // Current StallGuard threshold
    bool collisionDetected;// Collision flag
};

extern QueueHandle_t motorCmdQueue;
extern QueueHandle_t motorTelQueue;
