#include "LCD_Task.h"

TaskHandle_t lcdTaskHandle = NULL;

void LCD_task(void *parameter) {
  int interval = *(int *)parameter;
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (1) {
    draw_menu(); // Draw Screen

    // Wait until next interval mark
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(interval));
  }
}