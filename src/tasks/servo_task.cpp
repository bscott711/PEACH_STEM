#include "tasks/servo_task.h"
#include "drivers/ServoDriver.h"
#include <esp_log.h>

// Speed divisor: 1 = full speed, 2 = half speed, 4 = quarter speed, etc.
// Higher value = slower, smoother movement
#define SERVO_SPEED_DIVISOR 1

void servo_task(void *pvParameters) {
  ServoDriver_Init();

  // Fetch the interval passed from main.cpp
  int interval = *(int *)pvParameters;
  TickType_t lastWakeTime = xTaskGetTickCount();

  // Track the current physical position in 10x fixed-point (0 = 0.0%, 1000 =
  // 100.0%)
  int current_x10 = 0;
  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    current_x10 = systemState.servoPercent * 10;
    xSemaphoreGive(systemStateMutex);
  }

  // Step size in 10x units: 10 = 1% per tick, 5 = 0.5% per tick, etc.
  const int step = 10 / SERVO_SPEED_DIVISOR;

  while (1) {
    int target_x10 = current_x10;

    bool isActive = false;

    // 1. Safely read the target from the global state
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      target_x10 = systemState.servoTargetPercent * 10;
      isActive = systemState.servoActive;
      xSemaphoreGive(systemStateMutex);
    } else {
      ESP_LOGW("SERVO", "Mutex timeout reading target");
    }

    // 2. Smooth ramp: move toward target in sub-percent steps
    if (current_x10 < target_x10) {
      current_x10 = min(current_x10 + step, target_x10);
    } else if (current_x10 > target_x10) {
      current_x10 = max(current_x10 - step, target_x10);
    }

    // 3. Command the hardware (convert back from 10x to percent)
    int percent = current_x10 / 10;
    
    if (isActive) {
      ServoDriver_WritePercent(percent);
    } else {
      ServoDriver_Disable(); // Leave servo limp
    }

    // 4. Update the global state with our actual physical position
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      systemState.servoPercent = percent;
      xSemaphoreGive(systemStateMutex);
    } else {
      ESP_LOGW("SERVO", "Mutex timeout writing current percent");
    }

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(interval));
  }
}