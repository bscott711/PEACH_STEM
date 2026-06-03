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

extern ArmNode g_armNode;
extern ActuatorNode g_actuatorNode;
extern MotorNode g_motorNode;

void autonomous_task(void *pvParameters) {
  // Define the autonomous sequence steps
  const SequenceStep sequence[] = {
      {SEQ_MOVE_Z, 0, Z_CLEARANCE_POS, "Auto: Raise Z"},
      {SEQ_MOVE_ARM, 0, 0, "Auto: Arm Out"},
      {SEQ_WAIT_MS, 500, 0, "Wait 500ms"},
      {SEQ_MOVE_Z, 0, Z_TUBE_POS, "Auto: Lower Z"},
      {SEQ_MOVE_ARM, 100, 0, "Auto: Arm In"},
      {SEQ_WAIT_MS, 500, 0, "Wait 500ms"},
      {SEQ_MOVE_Z, 0, Z_CLEARANCE_POS, "Auto: Raise Z"},
      {SEQ_MOVE_ARM, 0, 0, "Auto: Arm Out"},
      {SEQ_WAIT_MS, 500, 0, "Wait 500ms"},
      {SEQ_MOVE_ACTUATOR, 100, 0, "Auto: Actuator Extend"},
      {SEQ_WAIT_MS, 1500, 0, "Wait 1.5s"},
      {SEQ_MOVE_ACTUATOR, 0, 0, "Auto: Actuator Retract"},
      {SEQ_WAIT_MS, 1500, 0, "Wait 1.5s"},
      {SEQ_MOVE_Z, 0, Z_TUBE_POS, "Auto: Lower Z (Done)"}};

  const int numSteps = sizeof(sequence) / sizeof(sequence[0]);
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
        // Send speed command to motor node
        int velocity = (step.zTarget > 0) ? AUTO_SEQUENCE_SPEED : -AUTO_SEQUENCE_SPEED;
        
        MotorTelemetry motorTel;
        float currentPos = 0;
        if (xQueuePeek(motorTelQueue, &motorTel, pdMS_TO_TICKS(10)) == pdPASS) {
          currentPos = motorTel.currentPosition;
        }

        // Determine direction
        bool goingUp = (step.zTarget > currentPos);
        velocity = goingUp ? AUTO_SEQUENCE_SPEED : -AUTO_SEQUENCE_SPEED;
        
        g_motorNode.setSpeed(velocity);

        // Wait until position is reached or E-STOP
        bool posReached = false;
        while (!posReached) {
          ev = xEventGroupGetBits(controlEvents);
          if (ev & BIT_ESTOP_REQUEST) {
            aborted = true;
            break;
          }

          if (xQueuePeek(motorTelQueue, &motorTel, pdMS_TO_TICKS(10)) == pdPASS) {
            currentPos = motorTel.currentPosition;
            if (goingUp && currentPos >= step.zTarget) posReached = true;
            if (!goingUp && currentPos <= step.zTarget) posReached = true;
          }

          if (!posReached) vTaskDelay(pdMS_TO_TICKS(10));
        }

        g_motorNode.setSpeed(0);

        // Back-sync encoder 2 (motor) to 0
        if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          g_encoderState.position[2] = 0;
          xSemaphoreGive(encoderStateMutex);
        }
        stepComplete = true;
        break;
      }

      case SEQ_MOVE_ARM: {
        ArmTelemetry armTel;
        if (xQueuePeek(armTelQueue, &armTel, pdMS_TO_TICKS(10)) == pdPASS) {
          int outSteps = armTel.posOut;
          int inSteps = armTel.posIn;
          if (outSteps != -1 && inSteps != -1) {
            float targetAbs = outSteps + (step.target / 100.0f) * (inSteps - outSteps);
            if (abs(armTel.currentPosition - targetAbs) < 10.0f) {
              stepComplete = true; // Reached target steps
            } else {
              g_armNode.setTarget(step.target); // Sends 0-100%
              vTaskDelay(pdMS_TO_TICKS(50));
            }
          } else {
            stepComplete = true; // Skip if uncalibrated
          }
        }
        break;
      }

      case SEQ_MOVE_ACTUATOR: {
        g_actuatorNode.setTarget(step.target);

        // UI Back-Sync: update encoder 1 so manual controls stay in sync
        if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          g_encoderState.position[1] = step.target / ACTUATOR_STEP_PERCENT;
          xSemaphoreGive(encoderStateMutex);
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
  Enc3Menu sel = MENU_AUTO;
  float targetZ = 0.0f;

  if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    sel = systemState.enc3MenuSelection;
    xSemaphoreGive(systemStateMutex);
  }

  // Read motor limits from telemetry
  MotorTelemetry motorTel;
  if (xQueuePeek(motorTelQueue, &motorTel, pdMS_TO_TICKS(10)) == pdPASS) {
    if (sel == MENU_GOTO_TOP) targetZ = motorTel.limits[2];
    else if (sel == MENU_GOTO_MID) targetZ = motorTel.limits[1];
    else if (sel == MENU_GOTO_BOT) targetZ = motorTel.limits[0];
  }

  LCD_setMessage("Auto: GOTO");
  printf("Starting GOTO target %.2f...\n", targetZ);

  float currentPos = 0.0f;
  if (xQueuePeek(motorTelQueue, &motorTel, pdMS_TO_TICKS(10)) == pdPASS) {
    currentPos = motorTel.currentPosition;
  }

  int velocity = (targetZ > currentPos) ? AUTO_SEQUENCE_SPEED : -AUTO_SEQUENCE_SPEED;
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
