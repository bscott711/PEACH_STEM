#include "MotorDriver.h"
#include "../controller.h"

// Initialize Motor
void motorDriver::begin(HardwareSerial &serial,
                        TMC2209::SerialAddress address) {
  driver.setup(serial, SERIAL_BAUD_RATE, address, RXD1, TXD1);
  driver.setRunCurrent(RUN_CURRENT_PERCENT);

  // --- CRITICAL FIX: GLOBAL STALLGUARD SETUP ---
  // We must configure the sensor here so it is ALWAYS on, not just during
  // homing.

  driver.disableCoolStep();   // CoolStep must be off or it blinds StallGuard
  driver.enableStealthChop(); // StealthChop must be on for StallGuard4
  driver.setCoolStepDurationThreshold(1048575); // Open the velocity window

  // Set to your perfectly tuned value!
  driver.setStallGuardThreshold(systemState.sgThreshold);

  // Energize the coils
  driver.enable();

  // --- NEW: STEALTHCHOP CALIBRATION WINDOW ---
  // Force a dead stop and wait 200ms so the chip can measure
  // the coil inductance and build its silent profile.
  driver.moveAtVelocity(0);
  vTaskDelay(pdMS_TO_TICKS(200));
  // -------------------------------------------
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
  
  // Set travel limit zero point
  systemState.currentPosition = 0.0;
  systemState.isHomed = true;

  // 7. Restore full power for normal operation
  driver.setRunCurrent(RUN_CURRENT_PERCENT);

  // --- RESTORE TUNED THRESHOLD ---
  updateSGThreshold(systemState.sgThreshold);
}

void motorDriver::updateSGThreshold(int newThreshold) {
  // The TMC2209 accepts threshold values from 0 to 255
  newThreshold = constrain(newThreshold, 0, 255);
  driver.setStallGuardThreshold(newThreshold);
}