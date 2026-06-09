#include "encoder_task.h"
#include "drivers/EncoderDriver.h"
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
    // Sleep forever until the ISR yields the semaphore
    xSemaphoreTake(encoderSem, portMAX_DELAY);

    // Drain all pending interrupts while pin is held low
    while (digitalRead(ENCODER_INT_PIN) == LOW) {
      EncoderDriver_Service();

      // Critical RTOS yield to prevent I2C blocking the watchdog
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

void encoderInit() {
  encoderSem = xSemaphoreCreateBinary();
  pinMode(ENCODER_INT_PIN, INPUT_PULLUP);
  attachInterrupt(ENCODER_INT_PIN, encoderISR, FALLING);
}