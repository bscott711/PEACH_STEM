#pragma once

enum DeviceMode
{
    IDLE,
    PICKUP_CELL,
    DROPOFF_CELL
};

enum ActuatorDirection
{
    ACT_STOP = 0,
    ACT_FORWARD,
    ACT_REVERSE
};

struct SystemState
{
    DeviceMode mode;
    bool busy;

    // Manual Servo Control
    bool    servoAdjustMode;
    int     servoPercent;
    int     servoTargetPercent;

    // Linear Actuator
    ActuatorDirection actuatorDir;

    // Servo Motor 
    int actualSpeed;
    int targetSpeed;

    // Actuator
    int actuatorTargetPercent;
};

extern SystemState systemState;

// FreeRTOS task entry
void controller_task(void *pvParameters);