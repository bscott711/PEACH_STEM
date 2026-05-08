#include "tasks/motor_task.h"
#include "drivers/LCDDriver.h"
#include <esp_log.h>

static motorDriver motor;

// Homing State Machine Tracker
enum HomingState { H_IDLE, H_MOVING, H_BLIND_WAIT, H_POLLING };
static HomingState homingState = H_IDLE;
static TickType_t homingStartTime = 0;
const TickType_t HOMING_TIMEOUT_MS = 10000; // 10 second timeout failsafe

void motor_task(void *parameter) {
  int interval = *(int *)parameter;
  TickType_t lastWakeTime = xTaskGetTickCount();

  int newSpeed = 0;
  bool motorLocked = false;

  int currentSGThreshold = 16;
  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    currentSGThreshold = systemState.sgThreshold;
    xSemaphoreGive(systemStateMutex);
  }

  while (1) {
    int targetSpeed = 0;
    int sgThreshold = 0;
    bool isHomed = false;
    float currentPos = 0.0f;

    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      targetSpeed = systemState.targetSpeed;
      sgThreshold = systemState.sgThreshold;
      isHomed = systemState.isHomed;
      currentPos = systemState.currentPosition;
      xSemaphoreGive(systemStateMutex);
    }

    if (sgThreshold != currentSGThreshold) {
      currentSGThreshold = sgThreshold;
      motor.updateSGThreshold(currentSGThreshold);
      Serial.printf("Motor Driver: SG Threshold updated to %d\n",
                    currentSGThreshold);
    }

    if (motorLocked && targetSpeed == 0) {
      motorLocked = false;
      if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        systemState.collisionDetected = false;
        xSemaphoreGive(systemStateMutex);
      }
      LCD_setMessage("MOTOR UNLOCKED");
      Serial.println("--- MOTOR UNLOCKED. ---");
    }

    // --- NON-BLOCKING HOMING TRIGGER ---
    EventBits_t events = xEventGroupWaitBits(controlEvents, BIT_HOMING_REQUEST,
                                             pdTRUE, pdFALSE, 0);
    if ((events & BIT_HOMING_REQUEST) && homingState == H_IDLE) {
      motorLocked = false;
      homingState = H_MOVING;

      if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        systemState.isHoming = true;
        xSemaphoreGive(systemStateMutex);
      }
    }

    // --- NON-BLOCKING HOMING STATE MACHINE ---
    if (homingState != H_IDLE) {
      switch (homingState) {
      case H_MOVING:
        motor.setupHoming();
        motor.setVelocity(-20000);
        homingStartTime = xTaskGetTickCount();
        homingState = H_BLIND_WAIT;
        break;

      case H_BLIND_WAIT:
        // Wait 1 second before listening to avoid static friction spike
        if (xTaskGetTickCount() - homingStartTime >= pdMS_TO_TICKS(1000)) {
          Serial.println("--- Listening to DIAG Pin ---");
          homingState = H_POLLING;
        }
        break;

      case H_POLLING:
        // Check for collision OR timeout
        if (digitalRead(DIAG_PIN) ==
            HIGH) { // DIAG normally pulls high on collision
          motor.setVelocity(0);
          Serial.println("--- Homing Complete! ---");

          motor.finishHoming(currentSGThreshold);

          if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            systemState.currentPosition = 0.0f;
            systemState.isHomed = true;
            systemState.isHoming = false;
            systemState.targetSpeed = 0;
            targetSpeed = 0;
            xSemaphoreGive(systemStateMutex);
          }
          newSpeed = 0;
          saveMotorState();
          homingState = H_IDLE;

        } else if (xTaskGetTickCount() - homingStartTime >
                   pdMS_TO_TICKS(HOMING_TIMEOUT_MS)) {
          ESP_LOGE("MOTOR", "Homing timeout - aborting");
          LCD_setMessage("Homing: TIMEOUT");

          motor.setVelocity(0);
          motor.finishHoming(currentSGThreshold);

          if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            systemState.isHoming = false;
            systemState.targetSpeed = 0;
            targetSpeed = 0;
            xSemaphoreGive(systemStateMutex);
          }
          newSpeed = 0;
          homingState = H_IDLE;
        }
        break;

      default:
        break;
      }
    }

    // --- LIVE POSITION TRACKING & LIMITS ---
    // Only execute normal limits and speed logic if NOT homing
    if (!motorLocked && homingState == H_IDLE) {
      if (newSpeed != 0) {
        float deltaPos = motorDistanceCalculator((float)newSpeed, interval);
        currentPos += deltaPos;

        if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          systemState.currentPosition = currentPos;
          xSemaphoreGive(systemStateMutex);
        }
      }

      if (isHomed && currentPos <= 0.0f && targetSpeed < 0) {
        targetSpeed = 0;
        if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          systemState.targetSpeed = 0;
          xSemaphoreGive(systemStateMutex);
        }
      }
    }

    // --- NORMAL SPEED CONTROL ---
    if (homingState == H_IDLE && newSpeed != targetSpeed) {
      newSpeed = targetSpeed;

      if (motorLocked) {
        motor.stop();
      } else {
        motor.setVelocity(newSpeed);
      }

      if (newSpeed == 0 && isHomed) {
        saveMotorState();
      }
    }

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(interval));
  }
}

void motorInit() {
  pinMode(DIAG_PIN, INPUT_PULLDOWN);

  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
  vTaskDelay(pdMS_TO_TICKS(200));
  motor.begin(Serial1, TMC2209::SERIAL_ADDRESS_0);
}