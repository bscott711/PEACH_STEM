#include "core/SequenceEngine.h"
#include "controller.h"
#include "messaging.h"
#include "tasks/MotorNode.h"
#include "tasks/ActuatorNode.h"
#include "tasks/ArmNode.h"
#include "drivers/LCDDriver.h"
#include "drivers/EncoderDriver.h"
#include "esp_log.h"
#include <cstdio>
#include <cmath>
#include <cstdint>

extern ArmNode g_armNode;
extern ActuatorNode g_actuatorNode;
extern MotorNode g_motorNode;

void autonomous_task(void *pvParameters) {
  uint8_t slowSpeed = 128;
  if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      slowSpeed = systemState.actGoSpeed;
      xSemaphoreGive(systemStateMutex);
  }

  int seqType = (int)(intptr_t)pvParameters;

  // Define the normal autonomous sequence steps
  const SequenceStep sequence_normal[] = {
      {SEQ_MOVE_Z, 0, 2, 0, "Auto: Raise Z"},
      {SEQ_MOVE_ARM, 100, 0, 0, "Auto: Arm Tip"},
      {SEQ_MOVE_ACTUATOR, 1, 0, 255, "Auto: Act Mid"},
      {SEQ_MOVE_Z, 0, 1, 0, "Auto: Lower Z Mid"},
      {SEQ_MOVE_ACTUATOR, 0, 0, 255, "Auto: Mix 1 Bot"},
      {SEQ_MOVE_ACTUATOR, 1, 0, 255, "Auto: Mix 1 Mid"},
      {SEQ_MOVE_ACTUATOR, 0, 0, 255, "Auto: Mix 2 Bot"},
      {SEQ_MOVE_ACTUATOR, 1, 0, 255, "Auto: Mix 2 Mid"},
      {SEQ_MOVE_ACTUATOR, 0, 0, 255, "Auto: Mix 3 Bot"},
      {SEQ_MOVE_ACTUATOR, 1, 0, 255, "Auto: Mix 3 Mid"},
      {SEQ_MOVE_Z, 0, 2, 0, "Auto: Raise Z"},
      {SEQ_MOVE_ARM, 200, 0, 0, "Auto: Arm Buffer"},
      {SEQ_MOVE_ARM_AND_Z, 0, 0, 0, "Auto: Clr & Lwr Z"},
      {SEQ_MOVE_ACTUATOR, 0, 0, 165, "Auto: Dispense"},
      {SEQ_WAIT_MS, 2000, 0, 0, "Auto: Pause"},
      {SEQ_MOVE_Z, 0, 2, 0, "Auto: Raise Z"},
      {SEQ_MOVE_ACTUATOR, 2, 0, 255, "Auto: Done"}
  };

  // Define the shutdown sequence
  const SequenceStep sequence_shutdown[] = {
      {SEQ_MOVE_ARM_AND_Z, 0, 2, 0, "Auto: Shutdown"}
  };

  const SequenceStep* sequence = (seqType == 1) ? sequence_shutdown : sequence_normal;
  const int numSteps = (seqType == 1) ? 1 : (sizeof(sequence_normal) / sizeof(sequence_normal[0]));
  bool aborted = false;

  LCD_setMessage("Auto: Running");
  printf("Starting Autonomous Sequence...\n");

  for (int i = 0; i < numSteps; i++) {
    const SequenceStep &step = sequence[i];

    // Check for E-STOP before each step
    EventBits_t ev = xEventGroupGetBits(controlEvents);
    if (ev & BIT_ESTOP_REQUEST) {
      aborted = true;
      break;
    }

    if (step.message != NULL) {
      LCD_setMessage(step.message);
    }

    bool stepComplete = false;
    while (!stepComplete) {
      ev = xEventGroupGetBits(controlEvents);
      if (ev & BIT_ESTOP_REQUEST) {
        aborted = true;
        break;
      }

      switch (step.action) {
      case SEQ_MOVE_Z: {
        MotorTelemetry motorTel;
        float currentPos = 0;
        float targetZ = 0;
        if (xQueuePeek(motorTelQueue, &motorTel, pdMS_TO_TICKS(10)) == pdPASS) {
          currentPos = motorTel.currentPosition;
          targetZ = motorTel.limits[step.limitIdx];
        }

        // Determine direction
        bool goingUp = (targetZ > currentPos);
        int velocity = 0;
        
        // Only command movement if we aren't already there (prevents instant speed spikes and locking)
        bool posReached = false;
        if (abs(targetZ - currentPos) <= 0.1f) {
            posReached = true;
        } else {
            int goSpeed = 5000;
            if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                goSpeed = systemState.zGoSpeed;
                xSemaphoreGive(systemStateMutex);
            }
            velocity = goingUp ? goSpeed : -goSpeed;
            g_motorNode.setSpeed(velocity);
        }

        // Wait until position is reached or E-STOP
        while (!posReached) {
          ev = xEventGroupGetBits(controlEvents);
          if (ev & BIT_ESTOP_REQUEST) {
            aborted = true;
            break;
          }

          if (xQueuePeek(motorTelQueue, &motorTel, pdMS_TO_TICKS(10)) == pdPASS) {
            currentPos = motorTel.currentPosition;
            if (goingUp && currentPos >= targetZ) posReached = true;
            if (!goingUp && currentPos <= targetZ) posReached = true;
          }

          if (!posReached) vTaskDelay(pdMS_TO_TICKS(10));
        }

        g_motorNode.setSpeed(0);
        stepComplete = true;
        break;
      }

      case SEQ_MOVE_ARM: {
        ArmTelemetry armTel;
        if (xQueuePeek(armTelQueue, &armTel, pdMS_TO_TICKS(10)) == pdPASS) {
          int outSteps = armTel.posOut;
          int inSteps = armTel.posIn;
          int bufferSteps = armTel.posBuffer;
          if (outSteps != -1 && inSteps != -1) {
            float targetAbs = 0;
            if (step.target == 200 && bufferSteps != -1) {
                targetAbs = bufferSteps;
            } else {
                targetAbs = outSteps + (step.target / 100.0f) * (inSteps - outSteps);
            }
            if (abs(armTel.currentPosition - targetAbs) < 10.0f) {
              stepComplete = true; // Reached target steps
            } else {
              int goSpeed = 5000;
              if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                  goSpeed = systemState.armGoSpeed;
                  xSemaphoreGive(systemStateMutex);
              }
              g_armNode.setTarget(step.target, goSpeed); // Sends target with speed
              vTaskDelay(pdMS_TO_TICKS(50));
            }
          } else {
            stepComplete = true; // Skip if uncalibrated
          }
        }
        break;
      }
      
      case SEQ_MOVE_ARM_AND_Z: {
        bool zReached = false;
        bool armReached = false;
        
        // --- Z Logic ---
        MotorTelemetry motorTel;
        float currentZPos = 0;
        float targetZ = 0;
        if (xQueuePeek(motorTelQueue, &motorTel, pdMS_TO_TICKS(10)) == pdPASS) {
          currentZPos = motorTel.currentPosition;
          targetZ = motorTel.limits[step.limitIdx];
        }
        bool zGoingUp = (targetZ > currentZPos);
        
        if (abs(targetZ - currentZPos) <= 0.1f || (zGoingUp && currentZPos >= targetZ) || (!zGoingUp && currentZPos <= targetZ)) {
            zReached = true;
            g_motorNode.setSpeed(0);
        } else {
            int goSpeed = 5000;
            if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                goSpeed = systemState.zGoSpeed;
                xSemaphoreGive(systemStateMutex);
            }
            g_motorNode.setSpeed(zGoingUp ? goSpeed : -goSpeed);
        }

        // --- Arm Logic ---
        ArmTelemetry armTel;
        if (xQueuePeek(armTelQueue, &armTel, pdMS_TO_TICKS(10)) == pdPASS) {
          int outSteps = armTel.posOut;
          int inSteps = armTel.posIn;
          int bufferSteps = armTel.posBuffer;
          if (outSteps != -1 && inSteps != -1) {
            float targetAbs = 0;
            if (step.target == 200 && bufferSteps != -1) {
                targetAbs = bufferSteps;
            } else {
                targetAbs = outSteps + (step.target / 100.0f) * (inSteps - outSteps);
            }
            if (abs(armTel.currentPosition - targetAbs) < 10.0f) {
              armReached = true;
            } else {
              int goSpeed = 5000;
              if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                  goSpeed = systemState.armGoSpeed;
                  xSemaphoreGive(systemStateMutex);
              }
              g_armNode.setTarget(step.target, goSpeed);
            }
          } else {
            armReached = true; // Skip if uncalibrated
          }
        }

        if (zReached && armReached) {
            stepComplete = true;
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        break;
      }

      case SEQ_MOVE_ACTUATOR: {
        ActuatorTelemetry actTel;
        int targetPct = 0;
        if (xQueuePeek(actuatorTelQueue, &actTel, pdMS_TO_TICKS(10)) == pdPASS) {
          targetPct = actTel.limits[step.target];
        }
        
        int speed = step.actuatorSpeed;
        if (speed == 255) { // Use configured speed if max speed is requested
            if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                speed = systemState.actGoSpeed;
                xSemaphoreGive(systemStateMutex);
            }
        }
        g_actuatorNode.setTarget(targetPct, speed);
        
        // Wait until actuator reaches the target percentage
        bool posReached = false;
        while (!posReached) {
          ev = xEventGroupGetBits(controlEvents);
          if (ev & BIT_ESTOP_REQUEST) {
            aborted = true;
            break;
          }

          if (xQueuePeek(actuatorTelQueue, &actTel, pdMS_TO_TICKS(10)) == pdPASS) {
            if (abs((int)actTel.currentPercent - targetPct) <= 1) {
              posReached = true;
            }
          }

          if (!posReached) vTaskDelay(pdMS_TO_TICKS(50));
        }
        
        stepComplete = true;
        break;
      }

      case SEQ_WAIT_MS: {
        // Interruptible delay: yield in 10ms chunks, checking for E-STOP
        int elapsed = 0;
        while (elapsed < step.target) {
          EventBits_t ev = xEventGroupGetBits(controlEvents);
          if (ev & BIT_ESTOP_REQUEST) {
            aborted = true;
            break;
          }
          vTaskDelay(pdMS_TO_TICKS(10));
          elapsed += 10;
        }
        stepComplete = true;
        break;
      }

      case SEQ_WAIT_USER: {
        // Wait for user button press OR E-STOP
        while (true) {
          EventBits_t ev = xEventGroupWaitBits(
              controlEvents, BIT_AUTO_RESUME | BIT_ESTOP_REQUEST, pdTRUE,
              pdFALSE, pdMS_TO_TICKS(100));

          if (ev & BIT_AUTO_RESUME) {
            // Clear any spurious ESTOP that arrived with the resume
            xEventGroupClearBits(controlEvents, BIT_ESTOP_REQUEST);
            LCD_setMessage("Auto: Resuming");
            stepComplete = true;
            break;
          }
          if (ev & BIT_ESTOP_REQUEST) {
            aborted = true;
            break;
          }
        }
        break;
      }

    } // end switch
    } // end while (!stepComplete)
  }   // end for

  // ---- Cleanup ----
  if (aborted) {
    // E-STOP: halt everything immediately
    g_motorNode.setSpeed(0);
    g_actuatorNode.setTarget(0);
    LCD_setMessage("Auto: E-STOPPED");
    printf("!!! Autonomous Sequence E-STOPPED !!!\n");
    xEventGroupClearBits(controlEvents, BIT_ESTOP_REQUEST);
  } else {
    LCD_setMessage("Auto: Complete");
    printf("Autonomous Sequence Complete.\n");
  }

  // Clear the running flag and delete self
  xEventGroupClearBits(controlEvents, BIT_AUTO_RUNNING);
  vTaskDelete(NULL);
}

void motor_goto_task(void *pvParameters) {
  int limitIdx = (int)(intptr_t)pvParameters;  // 0=Bot, 1=Mid, 2=Top
  float targetZ = 0.0f;

  // Read motor limits from telemetry
  MotorTelemetry motorTel;
  if (xQueuePeek(motorTelQueue, &motorTel, pdMS_TO_TICKS(10)) == pdPASS) {
    targetZ = motorTel.limits[limitIdx];
  }

  LCD_setMessage("Auto: GOTO");
  printf("Starting GOTO target %.2f...\n", targetZ);

  float currentPos = 0.0f;
  if (xQueuePeek(motorTelQueue, &motorTel, pdMS_TO_TICKS(10)) == pdPASS) {
    currentPos = motorTel.currentPosition;
  }

  int autoSpeed = 5000;
  if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    autoSpeed = systemState.zGoSpeed;
    xSemaphoreGive(systemStateMutex);
  }

  int velocity = (targetZ > currentPos) ? autoSpeed : -autoSpeed;
  bool goingUp = (velocity > 0);

  g_motorNode.setSpeed(velocity);

  bool aborted = false;
  while (true) {
    EventBits_t ev = xEventGroupGetBits(controlEvents);
    if (ev & BIT_ESTOP_REQUEST) {
      aborted = true;
      break;
    }

    if (xQueuePeek(motorTelQueue, &motorTel, pdMS_TO_TICKS(5)) == pdPASS) {
      currentPos = motorTel.currentPosition;
    }

    if (goingUp && currentPos >= targetZ) break;
    if (!goingUp && currentPos <= targetZ) break;

    vTaskDelay(pdMS_TO_TICKS(10));
  }

  g_motorNode.setSpeed(0);

  if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    g_encoderState.position[2] = 0;
    xSemaphoreGive(encoderStateMutex);
  }

  if (aborted) {
    LCD_setMessage("GOTO E-STOPPED");
    xEventGroupClearBits(controlEvents, BIT_ESTOP_REQUEST);
  } else {
    LCD_setMessage("GOTO Complete");
  }

  xEventGroupClearBits(controlEvents, BIT_AUTO_RUNNING);
  vTaskDelete(NULL);
}
