#include "drivers/MotorDriver.h"
#include "controller.h"

void motorDriver::begin(HardwareSerial &serial,
                        TMC2209::SerialAddress address) {
  driver.setup(serial, SERIAL_BAUD_RATE, address, RXD1, TXD1);
  driver.setRunCurrent(RUN_CURRENT_PERCENT);

  driver.disableCoolStep();
  driver.enableStealthChop();
  driver.setMicrostepsPerStep(16);
  driver.setCoolStepDurationThreshold(0);

  driver.setStallGuardThreshold(16);

  driver.enable();

  driver.moveAtVelocity(0);
  vTaskDelay(pdMS_TO_TICKS(200));
}

void motorDriver::setVelocity(int newSpeed) {
  newSpeed = constrain(newSpeed, -MOTOR_MAX_SAFE_STEPS, MOTOR_MAX_SAFE_STEPS);

  if (newSpeed > 0) {
    driver.disableInverseMotorDirection();
  } else {
    driver.enableInverseMotorDirection();
  }
  driver.moveAtVelocity(abs(newSpeed));
}

void motorDriver::stop() { driver.moveAtVelocity(0); }

void motorDriver::setupHoming() {
  Serial.println("Starting Hardware Sensorless Homing...");

  driver.setRunCurrent(70);
  driver.enableStealthChop();
  driver.setCoolStepDurationThreshold(1048575);
  driver.setStallGuardThreshold(15);
}

void motorDriver::finishHoming(int restoreThreshold) {
  driver.setRunCurrent(RUN_CURRENT_PERCENT);
  driver.enableStealthChop();
  driver.setCoolStepDurationThreshold(0);
  updateSGThreshold(restoreThreshold);
}

void motorDriver::updateSGThreshold(int newThreshold) {
  newThreshold = constrain(newThreshold, 0, 255);
  driver.setStallGuardThreshold(newThreshold);
}