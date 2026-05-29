#include "tasks/LCD_task.h"

TaskHandle_t lcdTaskHandle = NULL;

void LCD_task(void *parameter) {
  int interval = *(int *)parameter;
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (1) {
    if (g_otaActive) {
      draw_otaScreen();
    } else {
      draw_menu(); // Draw Screen
    }

    // Wait until next interval mark
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(interval));
  }
}