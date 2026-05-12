#include "tasks/actuator_task.h"
#include <esp_log.h>

// Time from 0% (retracted) to 100% (extended)
static const uint32_t FULL_EXTEND_TIME_MS = 1000; // Calibrate!

void actuator_task(void *pvParameters) {
  HBridge_Init();

  // Home (retract) on boot.
  // It is safe to block here since the RTOS scheduler is just starting.
  HBridge_Set(ACT_REVERSE);
  vTaskDelay(pdMS_TO_TICKS(FULL_EXTEND_TIME_MS));
  HBridge_Set(ACT_STOP);

  float currentPct = 0.0f;
  int interval = TASK_UPDATE_ACTUATOR;
  TickType_t lastWakeTime = xTaskGetTickCount();

  // Calculate the percentage traveled per task interval
  // e.g., (100.0 * 10ms) / 1000ms = 1.0% per tick
  float pctPerTick = (100.0f * (float)interval) / (float)FULL_EXTEND_TIME_MS;

  while (1) {
    int targetPct = currentPct;

    // 1. Safely read target from global state
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      targetPct = systemState.actuatorTargetPercent;
      xSemaphoreGive(systemStateMutex);
    } else {
      ESP_LOGW("ACTUATOR", "Mutex timeout reading target");
    }

    targetPct = constrain(targetPct, 0, 100);

    // 2. Non-blocking movement evaluation
    if (currentPct < targetPct) {
      currentPct += pctPerTick;
      if (currentPct > targetPct)
        currentPct = targetPct; // Clamp exact arrival
      HBridge_Set(ACT_FORWARD);
    } else if (currentPct > targetPct) {
      currentPct -= pctPerTick;
      if (currentPct < targetPct)
        currentPct = targetPct; // Clamp exact arrival
      HBridge_Set(ACT_REVERSE);
    } else {
      HBridge_Set(ACT_STOP);
    }

    // 3. Update the global state with our actual physical position
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      systemState.actuatorPercent = (int)currentPct;
      xSemaphoreGive(systemStateMutex);
    }

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(interval));
  }
}