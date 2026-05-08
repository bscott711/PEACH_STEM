#include "motor_task.h"
#include "drivers/LCDDriver.h"

static motorDriver motor;

// 1 Full Second blind window for heavy acceleration
// const unsigned long BLIND_WINDOW_MS = 1000;

// Set this to the speed value that corresponds to Encoder 4 (-3 or +3)
// const int MIN_STALLGUARD_SPEED = 12000;

void motor_task(void *parameter) {
  int interval = *(int *)parameter;
  TickType_t lastWakeTime = xTaskGetTickCount();

  int newSpeed = systemState.targetSpeed;
  bool motorLocked = false;
  // unsigned long lastMovementStartTime = 0;

  // Track the live threshold
  int currentSGThreshold = systemState.sgThreshold;

  while (1) {
    // --- LIVE THRESHOLD TUNING ---
    if (systemState.sgThreshold != currentSGThreshold) {
      currentSGThreshold = systemState.sgThreshold;
      motor.updateSGThreshold(currentSGThreshold);
      Serial.printf("Motor Driver: SG Threshold updated to %d\n",
                    currentSGThreshold);
    }

    // --- POLLING CRASH DETECTION ---
    // bool diagPinHigh = (digitalRead(DIAG_PIN) == HIGH);

    // LIVE SG DISABLED: SG4 requires StealthChop, which we are disabling for
    // normal movement.
    /*
    if (diagPinHigh && !systemState.isHoming && !motorLocked) {
      if ((millis() - lastMovementStartTime > BLIND_WINDOW_MS) &&
          (abs(newSpeed) > MIN_STALLGUARD_SPEED)) {
        motorLocked = true;
        systemState.collisionDetected = true;
        systemState.collisionTimestamp = millis();
        motor.stop();
        LCD_setMessage("COLLISION DETECTED");
        Serial.println("\n!!! EMERGENCY STOP: COLLISION DETECTED !!!");
        Serial.println(
            "--- MOTOR LOCKED. TURN SPEED DOWN TO 0 TO UNLOCK ---\n");
      }
    }
    */

    // --- CLEAR FAULT LOGIC ---
    if (motorLocked && systemState.targetSpeed == 0) {
      motorLocked = false;
      systemState.collisionDetected = false;
      LCD_setMessage("MOTOR UNLOCKED");
      Serial.println("--- MOTOR UNLOCKED. Ready for movement. ---");
    }

    // 1. Check if homing was requested
    if (systemState.triggerHoming) {
      motorLocked = false;
      systemState.triggerHoming = false;
      systemState.isHoming = true;
      motor.homeSensorless();
      systemState.isHoming = false;
      systemState.targetSpeed = 0;
      newSpeed = 0;
    }

    // 2. Live Position Tracking & Limits
    if (!motorLocked && !systemState.isHoming) {
      // Integrate velocity to track position (using motorDistanceCalculator
      // logic)
      systemState.currentPosition += (newSpeed * interval) * 0.000001372;

      // Enforce the hard travel limit (0 is the top limit defined by homing)
      if (systemState.isHomed && systemState.currentPosition <= 0.0 &&
          systemState.targetSpeed < 0) {
        systemState.targetSpeed = 0; // Sync back to controller
      }
    }

    // 3. Normal speed control
    if (newSpeed != systemState.targetSpeed) {
      newSpeed = systemState.targetSpeed;

      if (motorLocked) {
        motor.stop();
      } else {
        // Reset the blind window timer every time the speed changes
        // lastMovementStartTime = millis();
        motor.setVelocity(newSpeed);
      }

      // Save state to NVS when motor comes to a deliberate stop
      if (newSpeed == 0 && systemState.isHomed) {
        saveMotorState();
      }
    }

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(interval));
  }
}

void motorInit() {
  pinMode(DIAG_PIN, INPUT_PULLDOWN);

  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
  delay(200);
  motor.begin(Serial1, TMC2209::SERIAL_ADDRESS_0);
}