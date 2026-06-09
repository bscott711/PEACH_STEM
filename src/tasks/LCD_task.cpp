#include "tasks/LCD_task.h"
#include "messaging.h"
#include "core/UIData.h"
#include "core/NetworkManager.h"

TaskHandle_t lcdTaskHandle = NULL;

void LCD_task(void *parameter) {
  int interval = *(int *)parameter;
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (1) {
    if (NetworkManager::isOTAActive()) {
      draw_otaScreen();
    } else {
      UIData uiData;
      if (lcdDataQueue != NULL && xQueuePeek(lcdDataQueue, &uiData, pdMS_TO_TICKS(10)) == pdPASS) {
        draw_menu(uiData); // Draw Screen
      }
    }

    // Wait until next interval mark
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(interval));

#ifdef DEBUG_STACK
    static uint32_t lastLog = 0;
    if (xTaskGetTickCount() - lastLog > pdMS_TO_TICKS(5000)) {
        printf("LCD_task stack free: %u bytes\n", uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t));
        lastLog = xTaskGetTickCount();
    }
#endif
  }
}