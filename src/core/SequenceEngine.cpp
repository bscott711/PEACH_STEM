#include "SequenceEngine.h"
#include "controller.h"
#include "messaging.h"
#include "tasks/DishLiftNode.h"
#include "tasks/DishRotationNode.h"
#include "tasks/ScraperArmNode.h"
#include "drivers/LCDDriver.h"
#include "drivers/EncoderDriver.h"
#include "esp_log.h"
#include <cstdio>
#include <cmath>
#include <cstdint>

extern ScraperArmNode g_scraperArmNode;
extern DishRotationNode g_dishRotationNode;
extern DishLiftNode g_dishLiftNode;

void autonomous_task(void *pvParameters) {
  uint8_t slowSpeed = 128;
  if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      slowSpeed = systemState.dishRotationGoSpeed;
      xSemaphoreGive(systemStateMutex);
  }

  int seqType = (int)(intptr_t)pvParameters;

  // Define the normal autonomous sequence steps
  const SequenceStep sequence_normal[] = {
      {SEQ_MOVE_LIFT, 0, 2, 0, "Auto: Raise Z"},
      {SEQ_MOVE_SCRAPER, 100, 0, 0, "Auto: Arm Tip"},
      {SEQ_MOVE_ROTATION, 1, 0, 255, "Auto: Act Mid"},
      {SEQ_MOVE_LIFT, 0, 1, 0, "Auto: Lower Z Mid"},
      {SEQ_MOVE_ROTATION, 0, 0, 255, "Auto: Mix 1 Bot"},
      {SEQ_MOVE_ROTATION, 1, 0, 255, "Auto: Mix 1 Mid"},
      {SEQ_MOVE_ROTATION, 0, 0, 255, "Auto: Mix 2 Bot"},
      {SEQ_MOVE_ROTATION, 1, 0, 255, "Auto: Mix 2 Mid"},
      {SEQ_MOVE_ROTATION, 0, 0, 255, "Auto: Mix 3 Bot"},
      {SEQ_MOVE_ROTATION, 1, 0, 255, "Auto: Mix 3 Mid"},
      {SEQ_MOVE_LIFT, 0, 2, 0, "Auto: Raise Z"},
      {SEQ_MOVE_SCRAPER, 200, 0, 0, "Auto: Arm Buffer"},
      {SEQ_MOVE_SCRAPER_AND_Z, 0, 0, 0, "Auto: Clr & Lwr Z"},
      {SEQ_MOVE_ROTATION, 0, 0, 165, "Auto: Dispense"},
      {SEQ_WAIT_MS, 2000, 0, 0, "Auto: Pause"},
      {SEQ_MOVE_LIFT, 0, 2, 0, "Auto: Raise Z"},
      {SEQ_MOVE_ROTATION, 2, 0, 255, "Auto: Done"}
  };

  // Define the shutdown sequence
  const SequenceStep sequence_shutdown[] = {
      {SEQ_MOVE_LIFT, 0, 2, 0, "Auto: Raise Z"},
      {SEQ_MOVE_SCRAPER, 0, 0, 0, "Auto: Arm Clear"}
  };

  const SequenceStep* sequence = (seqType == 1) ? sequence_shutdown : sequence_normal;
  const int numSteps = (seqType == 1) ? 2 : (sizeof(sequence_normal) / sizeof(sequence_normal[0]));
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
      case SEQ_MOVE_LIFT: {
        DishLiftTelemetry motorTel;
        float currentPos = 0;
        float targetZ = 0;
        if (xQueuePeek(dishLiftTelQueue, &motorTel, pdMS_TO_TICKS(10)) == pdPASS) {
          currentPos = motorTel.currentPosition;
          targetZ = motorTel.limits[step.limitIdx];
        }

        // Determine direction
        bool goingUp = (targetZ > currentPos);
        
        // Only command movement if we aren't already there (prevents instant speed spikes and locking)
        bool posReached = false;
        if (std::abs(targetZ - currentPos) <= 0.1f) {
            posReached = true;
        } else {
            int goSpeed = 5000;
            if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                goSpeed = systemState.dishLiftGoSpeed;
                xSemaphoreGive(systemStateMutex);
            }
            g_dishLiftNode.setTarget(targetZ, goSpeed);
        }

        if (!posReached) {
            xEventGroupClearBits(controlEvents, BIT_POS_REACHED_Z);
            while (true) {
                EventBits_t uxBits = xEventGroupWaitBits(
                    controlEvents, BIT_POS_REACHED_Z | BIT_ESTOP_REQUEST,
                    pdTRUE, pdFALSE, portMAX_DELAY);
                
                if (uxBits & BIT_ESTOP_REQUEST) {
                    xEventGroupSetBits(controlEvents, BIT_ESTOP_REQUEST);
                    aborted = true;
                    break;
                }
                if (uxBits & BIT_POS_REACHED_Z) {
                    posReached = true;
                    break;
                }
                
                printf("Timeout waiting for Z-axis\n");
                aborted = true;
                break;
            }
        }

        g_dishLiftNode.setSpeed(0);
        stepComplete = true;
        break;
      }

      case SEQ_MOVE_SCRAPER: {
        ScraperArmTelemetry armTel;
        if (xQueuePeek(scraperArmTelQueue, &armTel, pdMS_TO_TICKS(10)) == pdPASS) {
          int outSteps = armTel.posOut;
          int inSteps = armTel.posIn;
          int bufferSteps = armTel.posBuffer;
          if (outSteps != -1 && inSteps != -1) {
            float targetAbs = 0;
            if (step.target == SEQ_TARGET_BUFFER && bufferSteps != -1) {
                targetAbs = bufferSteps;
            } else {
                targetAbs = outSteps + (step.target / 100.0f) * (inSteps - outSteps);
            }
            
            bool posReached = false;
            if (std::abs(armTel.currentPosition - targetAbs) < 10.0f) {
              posReached = true;
            } else {
              int goSpeed = 5000;
              if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                  goSpeed = systemState.scraperArmGoSpeed;
                  xSemaphoreGive(systemStateMutex);
              }
              g_scraperArmNode.setTarget(step.target == SEQ_TARGET_BUFFER ? 200.0f : (float)step.target, goSpeed);
            }
            
            if (!posReached) {
                xEventGroupClearBits(controlEvents, BIT_POS_REACHED_ARM);
                while (true) {
                    EventBits_t uxBits = xEventGroupWaitBits(
                        controlEvents, BIT_POS_REACHED_ARM | BIT_ESTOP_REQUEST,
                        pdTRUE, pdFALSE, portMAX_DELAY);
                    
                    if (uxBits & BIT_ESTOP_REQUEST) {
                        xEventGroupSetBits(controlEvents, BIT_ESTOP_REQUEST);
                        aborted = true;
                        break;
                    }
                    if (uxBits & BIT_POS_REACHED_ARM) {
                        posReached = true;
                        break;
                    }
                    
                    printf("Timeout waiting for ARM\n");
                    aborted = true;
                    break;
                }
            }
            if (posReached) stepComplete = true;
          } else {
            stepComplete = true; // Skip if uncalibrated
          }
        }
        break;
      }
      
      case SEQ_MOVE_SCRAPER_AND_Z: {
        bool zReached = false;
        bool armReached = false;
        
        // --- Z Logic ---
        DishLiftTelemetry motorTel;
        float currentZPos = 0;
        float targetZ = 0;
        if (xQueuePeek(dishLiftTelQueue, &motorTel, pdMS_TO_TICKS(10)) == pdPASS) {
          currentZPos = motorTel.currentPosition;
          targetZ = motorTel.limits[step.limitIdx];
        }
        bool zGoingUp = (targetZ > currentZPos);
        
        if (std::abs(targetZ - currentZPos) <= 0.1f) {
            zReached = true;
            g_dishLiftNode.setSpeed(0);
        } else {
            int goSpeed = 5000;
            if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                goSpeed = systemState.dishLiftGoSpeed;
                xSemaphoreGive(systemStateMutex);
            }
            g_dishLiftNode.setTarget(targetZ, goSpeed);
        }

        // --- Arm Logic ---
        ScraperArmTelemetry armTel;
        if (xQueuePeek(scraperArmTelQueue, &armTel, pdMS_TO_TICKS(10)) == pdPASS) {
          int outSteps = armTel.posOut;
          int inSteps = armTel.posIn;
          int bufferSteps = armTel.posBuffer;
          if (outSteps != -1 && inSteps != -1) {
            float targetAbs = 0;
            if (step.target == SEQ_TARGET_BUFFER && bufferSteps != -1) {
                targetAbs = bufferSteps;
            } else {
                targetAbs = outSteps + (step.target / 100.0f) * (inSteps - outSteps);
            }
            if (std::abs(armTel.currentPosition - targetAbs) < 10.0f) {
              armReached = true;
            } else {
              int goSpeed = 5000;
              if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                  goSpeed = systemState.scraperArmGoSpeed;
                  xSemaphoreGive(systemStateMutex);
              }
              g_scraperArmNode.setTarget(step.target == SEQ_TARGET_BUFFER ? 200.0f : (float)step.target, goSpeed);
            }
          } else {
            armReached = true; // Skip if uncalibrated
          }
        }

        xEventGroupClearBits(controlEvents, BIT_POS_REACHED_Z | BIT_POS_REACHED_ARM);
        
        while (!zReached || !armReached) {
            EventBits_t waitBits = 0;
            if (!zReached) waitBits |= BIT_POS_REACHED_Z;
            if (!armReached) waitBits |= BIT_POS_REACHED_ARM;
            waitBits |= BIT_ESTOP_REQUEST;
            
            EventBits_t uxBits = xEventGroupWaitBits(
                controlEvents, waitBits,
                pdTRUE, pdFALSE, portMAX_DELAY);
                
            if (uxBits & BIT_ESTOP_REQUEST) {
                xEventGroupSetBits(controlEvents, BIT_ESTOP_REQUEST);
                aborted = true;
                break;
            }
            if (uxBits & BIT_POS_REACHED_Z) zReached = true;
            if (uxBits & BIT_POS_REACHED_ARM) armReached = true;
            
            if (!(uxBits & waitBits)) {
                printf("Timeout waiting for ARM_AND_Z\n");
                aborted = true;
                break;
            }
        }
        
        if (zReached && armReached) {
            stepComplete = true;
        }
        break;
      }

      case SEQ_MOVE_ROTATION: {
        DishRotationTelemetry actTel;
        int targetPct = 0;
        if (xQueuePeek(dishRotationTelQueue, &actTel, pdMS_TO_TICKS(10)) == pdPASS) {
          targetPct = actTel.limits[step.target];
        }
        
        int speed = step.actuatorSpeed;
        if (speed == 255) { // Use configured speed if max speed is requested
            if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                speed = systemState.dishRotationGoSpeed;
                xSemaphoreGive(systemStateMutex);
            }
        }
        
        bool posReached = false;
        if (std::abs((int)actTel.currentPercent - targetPct) <= 1) {
          posReached = true;
        }
        
        g_dishRotationNode.setTarget(targetPct, speed);
        
        if (!posReached) {
            xEventGroupClearBits(controlEvents, BIT_POS_REACHED_ACT);
            while (true) {
                EventBits_t uxBits = xEventGroupWaitBits(
                    controlEvents, BIT_POS_REACHED_ACT | BIT_ESTOP_REQUEST,
                    pdTRUE, pdFALSE, portMAX_DELAY);
                
                if (uxBits & BIT_ESTOP_REQUEST) {
                    xEventGroupSetBits(controlEvents, BIT_ESTOP_REQUEST);
                    aborted = true;
                    break;
                }
                if (uxBits & BIT_POS_REACHED_ACT) {
                    posReached = true;
                    break;
                }
                
                printf("Timeout waiting for Actuator\n");
                aborted = true;
                break;
            }
        }
        
        if (posReached) stepComplete = true;
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
    g_dishLiftNode.setSpeed(0);
    g_dishRotationNode.setTarget(0);
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
  DishLiftTelemetry motorTel;
  if (xQueuePeek(dishLiftTelQueue, &motorTel, pdMS_TO_TICKS(10)) == pdPASS) {
    targetZ = motorTel.limits[limitIdx];
  }

  LCD_setMessage("Auto: GOTO");
  printf("Starting GOTO target %.2f...\n", targetZ);

  float currentPos = 0.0f;
  if (xQueuePeek(dishLiftTelQueue, &motorTel, pdMS_TO_TICKS(10)) == pdPASS) {
    currentPos = motorTel.currentPosition;
  }

  int autoSpeed = 5000;
  if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    autoSpeed = systemState.dishLiftGoSpeed;
    xSemaphoreGive(systemStateMutex);
  }

  g_dishLiftNode.setTarget(targetZ, autoSpeed);

  xEventGroupClearBits(controlEvents, BIT_POS_REACHED_Z);
  bool aborted = false;
  while (true) {
    EventBits_t uxBits = xEventGroupWaitBits(
        controlEvents, BIT_POS_REACHED_Z | BIT_ESTOP_REQUEST,
        pdTRUE, pdFALSE, portMAX_DELAY);
        
    if (uxBits & BIT_ESTOP_REQUEST) {
        xEventGroupSetBits(controlEvents, BIT_ESTOP_REQUEST);
        aborted = true;
        break;
    }
    if (uxBits & BIT_POS_REACHED_Z) {
        break;
    }
    
    printf("Timeout waiting for Z-axis GOTO\n");
    aborted = true;
    break;
  }

  g_dishLiftNode.setSpeed(0);

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
