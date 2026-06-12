#include "drivers/MotorDriver.h"
#include "controller.h"

void motorDriver::begin(HardwareSerial &serial,
                        TMC2209::SerialAddress address) {
  if (xSemaphoreTake(tmcUartMutex, portMAX_DELAY) == pdTRUE) {
    driver.setup(serial, SERIAL_BAUD_RATE, address, RXD1, TXD1);
    driver.setAllCurrentValues(RUN_CURRENT_PERCENT, RUN_CURRENT_PERCENT, 10);

  driver.disableCoolStep();
  driver.enableStealthChop();
  driver.setMicrostepsPerStep(16);

    driver.enable();

    driver.moveAtVelocity(0);
    xSemaphoreGive(tmcUartMutex);
  }
  vTaskDelay(pdMS_TO_TICKS(200));
}

void motorDriver::setVelocity(int newSpeed) {
  newSpeed = constrain(newSpeed, -MOTOR_MAX_SAFE_STEPS, MOTOR_MAX_SAFE_STEPS);

  if (xSemaphoreTake(tmcUartMutex, portMAX_DELAY) == pdTRUE) {
    // VACTUAL is a 24-bit signed integer, so we can just pass the signed speed directly!
    // Changing InverseMotorDirection resets the microstep counter and causes drift.
    driver.moveAtVelocity(newSpeed);
    xSemaphoreGive(tmcUartMutex);
  }
}

void motorDriver::stop() {
  if (xSemaphoreTake(tmcUartMutex, portMAX_DELAY) == pdTRUE) {
    driver.moveAtVelocity(0);
    xSemaphoreGive(tmcUartMutex);
  }
}

void motorDriver::setStallGuardThreshold(uint8_t threshold) {
  if (xSemaphoreTake(tmcUartMutex, portMAX_DELAY) == pdTRUE) {
    driver.setStallGuardThreshold(threshold);
    xSemaphoreGive(tmcUartMutex);
  }
}

uint16_t motorDriver::getStallGuardResult() {
  uint16_t result = 0;
  if (xSemaphoreTake(tmcUartMutex, portMAX_DELAY) == pdTRUE) {
    result = driver.getStallGuardResult();
    xSemaphoreGive(tmcUartMutex);
  }
  return result;
}

uint8_t motorDriver::getVersion() {
  uint8_t version = 0;
  if (xSemaphoreTake(tmcUartMutex, portMAX_DELAY) == pdTRUE) {
    version = driver.getVersion();
    xSemaphoreGive(tmcUartMutex);
  }
  return version;
}

void motorDriver::setCoolStepDurationThreshold(uint32_t threshold) {
  if (xSemaphoreTake(tmcUartMutex, portMAX_DELAY) == pdTRUE) {
    driver.setCoolStepDurationThreshold(threshold);
    xSemaphoreGive(tmcUartMutex);
  }
}

void motorDriver::setCurrent(uint8_t runCurrentPercent) {
  if (xSemaphoreTake(tmcUartMutex, portMAX_DELAY) == pdTRUE) {
    driver.setAllCurrentValues(runCurrentPercent, runCurrentPercent, 10);
    xSemaphoreGive(tmcUartMutex);
  }
}