#include "MotorDriver.h"

// Initialize Motor
void motorDriver::begin(HardwareSerial &serial,
                        TMC2209::SerialAddress address) {
  driver.setup(serial, SERIAL_BAUD_RATE, address, RXD1, TXD1);
  driver.setRunCurrent(RUN_CURRENT_PERCENT);
  driver.enableCoolStep();
  driver.enable();
}

void motorDriver::setVelocity(int newSpeed) {
  // Constrain speed within bounds
  newSpeed = constrain(newSpeed, -MOTOR_MAX_SAFE_STEPS, MOTOR_MAX_SAFE_STEPS);

  if (newSpeed > 0) {
    driver.disableInverseMotorDirection();
  } else {
    driver.enableInverseMotorDirection();
  }
  // Update Driver
  driver.moveAtVelocity(abs(newSpeed));
}

void motorDriver::stop() { driver.moveAtVelocity(0); }

void motorDriver::homeSensorless() {
  Serial.println("Starting Hardware Sensorless Homing...");

  // 1. Setup the driver for a gentle bump
  driver.setRunCurrent(70);
  driver.enableStealthChop();

  // Open the velocity window so StallGuard runs
  driver.setCoolStepDurationThreshold(1048575);

  // 2. Set the trigger sensitivity (0-255).
  // Tune this up or down based on physical testing.
  driver.setStallGuardThreshold(
      15); // 8 was crazy force required. 12: Required some decent force. 28:
           // stopped on its own from a hitch. 48 stopped immediately

  // 3. Start moving up towards the hard stop
  setVelocity(-20000);

  // 4. Blind period to let the motor overcome static friction (ignoring the
  // DIAG pin)
  vTaskDelay(pdMS_TO_TICKS(1000));

  Serial.println("--- Listening to DIAG Pin ---");

  // Using the centralized macro from the header instead of '4'
  while (digitalRead(DIAG_PIN) == LOW) {
    // Just wait for the collision
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  // 6. The DIAG pin fired! Stop instantly.
  setVelocity(0);
  Serial.println("--- Homing Complete! ---");

  // 7. Restore full power for normal operation
  driver.setRunCurrent(RUN_CURRENT_PERCENT);

  // 7. Restore full power for normal operation
  driver.setRunCurrent(RUN_CURRENT_PERCENT);
}