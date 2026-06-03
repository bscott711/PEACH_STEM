#include "tasks/LCD_task.h"
#include "messaging.h"
#include "core/UIData.h"

TaskHandle_t lcdTaskHandle = NULL;

void LCD_task(void *parameter) {
  int interval = *(int *)parameter;
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (1) {
    if (g_otaActive) {
      draw_otaScreen();
    } else {
      UIData uiData;
      if (lcdDataQueue != NULL && xQueuePeek(lcdDataQueue, &uiData, pdMS_TO_TICKS(10)) == pdPASS) {
        draw_menu(uiData); // Draw Screen
      }
    }

    // Wait until next interval mark
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(interval));
  }
}