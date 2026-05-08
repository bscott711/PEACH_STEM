#include "tasks/servo_task.h"
#include "drivers/ServoDriver.h"
#include <esp_log.h>

void servo_task(void *pvParameters) {
  ServoDriver_Init();

  // Fetch the interval passed from main.cpp
  int interval = *(int *)pvParameters;
  TickType_t lastWakeTime = xTaskGetTickCount();

  // Track the current physical position locally
  int current = 0;
  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    current = systemState.servoPercent;
    xSemaphoreGive(systemStateMutex);
  }

  while (1) {
    int target = current;

    // 1. Safely read the target from the global state
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      target = systemState.servoTargetPercent;
      xSemaphoreGive(systemStateMutex);
    } else {
      ESP_LOGW("SERVO", "Mutex timeout reading target");
    }

    // 2. Software dampening: move smoothly toward target with strict hardware
    // limits
    if (current < target) {
      current = constrain(current + 1, SERVO_MIN_PERCENT, SERVO_MAX_PERCENT);
    } else if (current > target) {
      current = constrain(current - 1, SERVO_MIN_PERCENT, SERVO_MAX_PERCENT);
    }

    // 3. Command the hardware
    ServoDriver_WritePercent(current);

    // 4. Update the global state with our actual physical position
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      systemState.servoPercent = current;
      xSemaphoreGive(systemStateMutex);
    } else {
      ESP_LOGW("SERVO", "Mutex timeout writing current percent");
    }

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(interval));
  }
}