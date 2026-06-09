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
#define BIT_POS_REACHED_Z (1 << 4)
#define BIT_POS_REACHED_ARM (1 << 5)
#define BIT_POS_REACHED_ACT (1 << 6)

#include "core/SystemState.h"

extern SystemState systemState;
extern SemaphoreHandle_t systemStateMutex;
extern SemaphoreHandle_t encoderStateMutex;
extern SemaphoreHandle_t tmcUartMutex;
extern EventGroupHandle_t controlEvents;

// Queue handles declared in controller.cpp, extern here for access
extern QueueHandle_t scraperArmCmdQueue;
extern QueueHandle_t scraperArmTelQueue;
extern QueueHandle_t dishRotationCmdQueue;
extern QueueHandle_t dishRotationTelQueue;
extern QueueHandle_t dishLiftCmdQueue;
extern QueueHandle_t dishLiftTelQueue;

// Removed OTA globals from here

void initSystemState();

// FreeRTOS task entries
void controller_task(void *pvParameters);