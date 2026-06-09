#include "encoder_task.h"
#include "drivers/EncoderDriver.h"
#include "drivers/LCDDriver.h"
#include <Arduino.h>

static SemaphoreHandle_t encoderSem;

// Must be in IRAM so the interrupt fires extremely fast from RAM, not Flash
static void IRAM_ATTR encoderISR() {
  BaseType_t hp = pdFALSE;
  xSemaphoreGiveFromISR(encoderSem, &hp);
  portYIELD_FROM_ISR(hp);
}

void encoderTask(void *pv) {
  init_encoder();

  while (true) {
    // Check if interrupt pin is low OR just poll it
    if (digitalRead(ENCODER_INT_PIN) == LOW) {
      EncoderDriver_Service();
    } else {
      // Fallback: poll every 50ms anyway to guarantee we never miss events
      EncoderDriver_Service();
    }
    
    vTaskDelay(pdMS_TO_TICKS(20)); // Poll at 50Hz
  }
}

void encoderInit() {
  encoderSem = xSemaphoreCreateBinary();
  pinMode(ENCODER_INT_PIN, INPUT_PULLUP);
  attachInterrupt(ENCODER_INT_PIN, encoderISR, FALLING);
}