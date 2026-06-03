#include "controller.h"
#include "messaging.h"
#include "tasks/ActuatorNode.h"
#include "tasks/MotorNode.h"
#include "tasks/ArmNode.h"
#include "drivers/EncoderDriver.h"
#include "drivers/LCDDriver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "core/StorageManager.h"
#include <cstdio>
#include "core/SequenceEngine.h"
#include "core/InputManager.h"
// Global queue handles (declared extern in controller.h)
QueueHandle_t armCmdQueue;
QueueHandle_t armTelQueue;
QueueHandle_t actuatorCmdQueue;
QueueHandle_t actuatorTelQueue;
QueueHandle_t motorCmdQueue;
QueueHandle_t motorTelQueue;
QueueHandle_t lcdDataQueue;

// Global Node instances (defined in main.cpp, extern here)
extern ArmNode g_armNode;
extern ActuatorNode g_actuatorNode;
extern MotorNode g_motorNode;


SemaphoreHandle_t systemStateMutex;
SemaphoreHandle_t encoderStateMutex;
EventGroupHandle_t controlEvents;

SystemState systemState = {.mode = IDLE,
                           .enc1MenuSelection = MENU_ACT_MAN,
                           .enc3MenuSelection = MENU_AUTO,
                           .collisionDetected = false,
                           .collisionTimestamp = 0};

void initSystemState() {
  systemStateMutex = xSemaphoreCreateMutex();
  encoderStateMutex = xSemaphoreCreateMutex();
  controlEvents = xEventGroupCreate();

  StorageManager::init();
  
  // Initialize minimal state - subsystem state is managed by Active Nodes
  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    systemState.mode = IDLE;
    systemState.enc1MenuSelection = MENU_ACT_MAN;
    systemState.enc3MenuSelection = MENU_AUTO;
    systemState.collisionDetected = false;
    xSemaphoreGive(systemStateMutex);
  }
}

// Removed unused state save wrappers

// ============================================================================
// Main Controller Task
// ============================================================================

void controller_task(void *pvParameters) {
  InputManager::init();

  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t CONTROLLER_INTERVAL = pdMS_TO_TICKS(10);

  while (1) {
    // Process encoder inputs and dispatch commands
    InputManager::process();

    // Build and push UIData
    UIData uiData;
    InputManager::populateUIData(uiData);
    if (lcdDataQueue) {
        xQueueOverwrite(lcdDataQueue, &uiData);
    }

    vTaskDelayUntil(&lastWakeTime, CONTROLLER_INTERVAL);
  }
}



// Removed unused motor calculators
