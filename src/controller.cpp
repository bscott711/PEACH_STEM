#include "controller.h"
#include "messaging.h"
#include "tasks/DishRotationNode.h"
#include "tasks/DishLiftNode.h"
#include "tasks/ScraperArmNode.h"
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
QueueHandle_t scraperArmCmdQueue;
QueueHandle_t scraperArmTelQueue;
QueueHandle_t dishRotationCmdQueue;
QueueHandle_t dishRotationTelQueue;
QueueHandle_t dishLiftCmdQueue;
QueueHandle_t dishLiftTelQueue;
QueueHandle_t lcdDataQueue;

// Global Node instances (defined in main.cpp, extern here)
extern ScraperArmNode g_scraperArmNode;
extern DishRotationNode g_dishRotationNode;
extern DishLiftNode g_dishLiftNode;


SemaphoreHandle_t systemStateMutex;
SemaphoreHandle_t encoderStateMutex;
SemaphoreHandle_t tmcUartMutex;
EventGroupHandle_t controlEvents;

SystemState systemState = {.mode = IDLE,
                           .s4Menu = S4_STOP,
                           .s4SubMenu = 0,
                           .s4InSubMenu = false,
                           .s4InSpeedEdit = false,
                           .scraperArmJogSpeed = 5000,
                           .scraperArmGoSpeed = 5000,
                           .dishRotationJogSpeed = 128,
                           .dishRotationGoSpeed = 128,
                           .dishLiftJogSpeed = 5000,
                           .dishLiftGoSpeed = 5000,
                           .collisionDetected = false,
                           .collisionTimestamp = 0};

void initSystemState() {
  systemStateMutex = xSemaphoreCreateMutex();
  encoderStateMutex = xSemaphoreCreateMutex();
  tmcUartMutex = xSemaphoreCreateMutex();
  controlEvents = xEventGroupCreate();

  StorageManager::init();
  
  // Initialize minimal state - subsystem state is managed by Active Nodes
  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    systemState.mode = IDLE;
    systemState.s4Menu = S4_STOP;
    systemState.s4SubMenu = 0;
    systemState.s4InSubMenu = false;
    systemState.s4InSpeedEdit = false;
    
    // Load speeds from NVS with defaults
    systemState.scraperArmJogSpeed = StorageManager::loadScraperArmJogSpeed(5000);
    systemState.scraperArmGoSpeed = StorageManager::loadScraperArmGoSpeed(5000);
    systemState.dishRotationJogSpeed = StorageManager::loadDishRotationJogSpeed(128);
    systemState.dishRotationGoSpeed = StorageManager::loadDishRotationGoSpeed(128);
    systemState.dishLiftJogSpeed = StorageManager::loadDishLiftJogSpeed(5000);
    systemState.dishLiftGoSpeed = StorageManager::loadDishLiftGoSpeed(5000);
    
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

#ifdef DEBUG_STACK
    static uint32_t lastLog = 0;
    if (xTaskGetTickCount() - lastLog > pdMS_TO_TICKS(5000)) {
        printf("Controller_task stack free: %u bytes\n", uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t));
        lastLog = xTaskGetTickCount();
    }
#endif
  }
}



// Removed unused motor calculators
