#include "tasks/servo_task.h"
#include "drivers/ServoDriver.h"
#include <esp_log.h>

// Speed divisor: 1.0 = full speed, 2.0 = half speed, 4.0 = quarter speed, etc.
// Higher value = slower, smoother movement
#define SERVO_SPEED_DIVISOR 1.0f

void servo_task(void *pvParameters) {
  ServoDriver_Init();

  // Fetch the interval passed from main.cpp
  int interval = *(int *)pvParameters;
  TickType_t lastWakeTime = xTaskGetTickCount();

  // Track the current physical position in high-resolution float
  float current_pos = 0.0f;
  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    current_pos = (float)systemState.servoPercent;
    xSemaphoreGive(systemStateMutex);
  }

  // Step size per tick
  const float step = 1.0f / SERVO_SPEED_DIVISOR;

  while (1) {
    float target_pos = current_pos;
    bool isActive = false;

    // 1. Safely read the target from the global state
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      target_pos = (float)systemState.servoTargetPercent;
      isActive = systemState.servoActive;
      xSemaphoreGive(systemStateMutex);
    } else {
      ESP_LOGW("SERVO", "Mutex timeout reading target");
    }

    // 2. Smooth ramp: move toward target in high-resolution sub-percent steps
    if (current_pos < target_pos) {
      current_pos += step;
      if (current_pos > target_pos) current_pos = target_pos;
    } else if (current_pos > target_pos) {
      current_pos -= step;
      if (current_pos < target_pos) current_pos = target_pos;
    }

    // 3. Command the hardware with the high-resolution float
    if (isActive) {
      ServoDriver_WritePercent(current_pos);
    } else {
      ServoDriver_Disable(); // Leave servo limp
    }

    // 4. Update the global state with our actual physical position
    // (Cast back to int to maintain compatibility with the UI's SystemState struct)
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      systemState.servoPercent = (int)current_pos;
      xSemaphoreGive(systemStateMutex);
    } else {
      ESP_LOGW("SERVO", "Mutex timeout writing current percent");
    }

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(interval));
  }
}