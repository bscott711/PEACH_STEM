#include "motor_task.h"

static motorDriver motor;

// 1 Full Second blind window for heavy acceleration
const unsigned long BLIND_WINDOW_MS = 1000;

// Set this to the speed value that corresponds to Encoder 4 (-4 or +4)
const int MIN_STALLGUARD_SPEED = 19000;

void motor_task(void *parameter) {
  int interval = *(int *)parameter;
  TickType_t lastWakeTime = xTaskGetTickCount();

  int newSpeed = systemState.targetSpeed;
  bool motorLocked = false;
  unsigned long lastMovementStartTime = 0;

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
    bool diagPinHigh = (digitalRead(DIAG_PIN) == HIGH);

    if (diagPinHigh && !systemState.isHoming && !motorLocked) {
      if ((millis() - lastMovementStartTime > BLIND_WINDOW_MS) &&
          (abs(newSpeed) > MIN_STALLGUARD_SPEED)) {
        motorLocked = true;
        motor.stop();
        Serial.println("\n!!! EMERGENCY STOP: COLLISION DETECTED !!!");
        Serial.println(
            "--- MOTOR LOCKED. TURN SPEED DOWN TO 0 TO UNLOCK ---\n");
      }
    }

    // --- CLEAR FAULT LOGIC ---
    if (motorLocked && systemState.targetSpeed == 0) {
      motorLocked = false;
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

    // 2. Normal speed control
    if (newSpeed != systemState.targetSpeed) {
      newSpeed = systemState.targetSpeed;

      if (motorLocked) {
        motor.stop();
      } else {
        // Reset the blind window timer every time the speed changes
        lastMovementStartTime = millis();
        motor.setVelocity(newSpeed);
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