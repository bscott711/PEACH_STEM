#include "core/InputManager.h"
#include "controller.h"
#include "messaging.h"
#include "tasks/MotorNode.h"
#include "tasks/ActuatorNode.h"
#include "tasks/ArmNode.h"
#include "drivers/LCDDriver.h"
#include "drivers/EncoderDriver.h"
#include "core/SequenceEngine.h"
#include "esp_log.h"
#include <cstdio>
#include <cmath>

extern ArmNode g_armNode;
extern ActuatorNode g_actuatorNode;
extern MotorNode g_motorNode;

void InputManager::init() {
  if (xSemaphoreTake(encoderStateMutex, portMAX_DELAY) == pdTRUE) {
    g_encoderState.position[0] = 0;
    g_encoderState.position[1] = 0;
    g_encoderState.position[2] = 0;
    g_encoderState.position[3] = 0;
    xSemaphoreGive(encoderStateMutex);
  }
}

void InputManager::process() {
    handleArmEncoder();
    handleActuatorEncoder();
    handleMotorEncoder();
    handleAutonomousEncoder();
}

void InputManager::handleArmEncoder() {
  int32_t currentPos = 0;
  bool btnPressed = false;
  bool longPressed = false;
  bool doublePressed = false;

  // 1. Thread-safe copy and clear encoder state
  if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    // DO NOT CONSTRAIN HERE. We want unbounded relative movement for the stepper!
    currentPos = g_encoderState.position[0];

    if (g_encoderState.buttonPressed[0]) {
      btnPressed = true;
      g_encoderState.buttonPressed[0] = false;
    }
    if (g_encoderState.buttonLongPressed[0]) {
      longPressed = true;
      g_encoderState.buttonLongPressed[0] = false;
    }
    if (g_encoderState.buttonDoublePressed[0]) {
      doublePressed = true;
      g_encoderState.buttonDoublePressed[0] = false;
    }
    xSemaphoreGive(encoderStateMutex);
  }

  // 2. Read current calibration state from arm telemetry
  ArmTelemetry armTel;
  int posOut = -1, posIn = -1;
  float currentAbsPos = 0;
  if (xQueuePeek(armTelQueue, &armTel, pdMS_TO_TICKS(10)) == pdPASS) {
    posOut = armTel.posOut;
    posIn = armTel.posIn;
    currentAbsPos = armTel.currentPosition;
  }

  // 3. Calibration state machine (persistent across calls)
  enum ArmCalibrationStep { CAL_OFF, CAL_SET_OUT, CAL_SET_IN };
  static ArmCalibrationStep calStep = CAL_OFF;
  static int32_t lastEncPos = 0;

  // 4. Very Long Press (Double): Clear Calibration
  if (doublePressed) {
    if (posOut == -1 && posIn == -1) {
      LCD_setMessage("CAL: Already Clear");
    } else {
      g_armNode.clearCal();
      LCD_setMessage("CAL: Cleared");
      printf("Arm CAL: Cleared preset positions\n");
    }
    calStep = CAL_OFF;
    return;
  }

  // 5. Long press: enter calibration mode
  if (longPressed && calStep == CAL_OFF) {
    calStep = CAL_SET_OUT;
    lastEncPos = currentPos;
    LCD_notifyButtonPress(0);
    LCD_setMessage("CAL: Set OUT");
    printf("Entering Arm Calibration Mode\n");
    return;
  }

  // 6. Calibration mode logic
  if (calStep != CAL_OFF) {
    // Jog the arm directly based on encoder delta (1 click = 100 steps)
    int delta = currentPos - lastEncPos;
    if (delta != 0) {
      lastEncPos = currentPos;
      g_armNode.jog(delta * 100.0f);
    }

    if (btnPressed) {
      LCD_notifyButtonPress(0);
      g_armNode.stop(); // Stop any residual motion

      if (calStep == CAL_SET_OUT) {
        g_armNode.setPosOut();
        calStep = CAL_SET_IN;
        LCD_setMessage("CAL: Set IN");
        printf("Arm CAL: Out saved, now set In\n");

      } else if (calStep == CAL_SET_IN) {
        g_armNode.setPosIn();
        calStep = CAL_OFF;
        LCD_setMessage("CAL: Saved!");
        printf("Arm CAL: In saved, calibration finished\n");
        
        // Convert the current physical position to a percentage for normal mode
        float range = (float)(posIn - posOut);
        int currentPct = 100;
        if (abs(range) > 1.0f) {
           currentPct = (int)(((currentAbsPos - posOut) / range) * 100.0f);
        }
        currentPct = constrain(currentPct, 0, 100);

        // Reset encoder position to match the percentage
        if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          g_encoderState.position[0] = currentPct;
          xSemaphoreGive(encoderStateMutex);
        }
      }
    }
    return; // Skip normal mode processing while calibrating
  }

  // 7. Normal mode: encoder adjusts arm target percentage (0-100%)
  static int32_t lastTargetPct = -1;
  int targetPct = currentPos;
  targetPct = constrain(targetPct, 0, 100);

  // Sync the hardware encoder back so it doesn't run away outside 0-100
  if (targetPct != currentPos) {
    if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      g_encoderState.position[0] = targetPct;
      currentPos = targetPct;
      xSemaphoreGive(encoderStateMutex);
    }
  }

  if (targetPct != lastTargetPct) {
    lastTargetPct = targetPct;
    if (posOut != -1 && posIn != -1) {
      g_armNode.setTarget((float)targetPct);
    }
  }

  // 8. Short press: toggle between Out (0%) and In (100%)
  if (btnPressed) {
    LCD_notifyButtonPress(0);

    if (posOut == -1 || posIn == -1) {
      LCD_setMessage("Not Calibrated");
      return;
    }

    int newTargetPct = (targetPct < 50) ? 100 : 0; // Toggle to opposite end

    // Interlock: Block movement to 0% (Out) if Z-axis is not at Top clearance
    if (newTargetPct == 0) {
      MotorTelemetry motorTel;
      bool atTop = false;
      if (xQueuePeek(motorTelQueue, &motorTel, pdMS_TO_TICKS(10)) == pdPASS) {
        atTop = motorTel.topEndstopTriggered;
      }
      if (!atTop) {
        LCD_setMessage("Raise Z-Axis First!");
        printf("Arm Interlock: Blocked move to Out because Z-axis is not at Top.\n");
        return; // Abort
      }
    }

    // Sync encoder so next turn starts from the toggled position
    if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      g_encoderState.position[0] = newTargetPct;
      xSemaphoreGive(encoderStateMutex);
    }
    lastTargetPct = newTargetPct;

    LCD_setMessage(newTargetPct == 100 ? "Arm: In" : "Arm: Out");
    g_armNode.setTarget((float)newTargetPct);
  }
}

void InputManager::handleActuatorEncoder() {
  int32_t currentPos = 0;
  bool btnPressed = false;
  bool longPress = false;
  bool doublePress = false;

  if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    int maxPos = 100 / ACTUATOR_STEP_PERCENT;
    g_encoderState.position[1] = constrain(g_encoderState.position[1], 0, maxPos);
    currentPos = g_encoderState.position[1];

    if (g_encoderState.buttonPressed[1]) {
      btnPressed = true;
      g_encoderState.buttonPressed[1] = false;
    }
    if (g_encoderState.buttonLongPressed[1]) {
      longPress = true;
      g_encoderState.buttonLongPressed[1] = false;
    }
    if (g_encoderState.buttonDoublePressed[1]) {
      doublePress = true;
      g_encoderState.buttonDoublePressed[1] = false;
    }
    xSemaphoreGive(encoderStateMutex);
  }

  // Read actuator telemetry
  ActuatorTelemetry actTel;
  int actLimits[3] = {0, 0, 0};
  bool actLimitSet[3] = {false, false, false};
  if (xQueuePeek(actuatorTelQueue, &actTel, pdMS_TO_TICKS(10)) == pdPASS) {
    actLimits[0] = actTel.limits[0];
    actLimits[1] = actTel.limits[1];
    actLimits[2] = actTel.limits[2];
    actLimitSet[0] = actTel.limitSet[0];
    actLimitSet[1] = actTel.limitSet[1];
    actLimitSet[2] = actTel.limitSet[2];
  }

  // 1. Encoder Turn: Jog Actuator
  static int32_t d1 = 0;
  if (d1 != currentPos) {
    d1 = currentPos;
    int targetPct = d1 * ACTUATOR_STEP_PERCENT;
    g_actuatorNode.setTarget(targetPct);
  }

  // 2. Short Press: Cycle Menu and Execute GOTO
  if (btnPressed) {
    LCD_notifyButtonPress(1);
    
    Enc1Menu sel = systemState.enc1MenuSelection;
    int newSel = (int)sel + 1;
    if (newSel > 3) newSel = 0; // 4 menu items (0-3): MAN, GOTO_TOP, GOTO_MID, GOTO_BOT
    systemState.enc1MenuSelection = (Enc1Menu)newSel;
    
    // Auto-GOTO if the limit is set
    if (newSel == MENU_ACT_GOTO_TOP && actLimitSet[2]) {
      g_actuatorNode.setTarget(actLimits[2]);
      d1 = (actLimits[2] + ACTUATOR_STEP_PERCENT/2) / ACTUATOR_STEP_PERCENT;
    } else if (newSel == MENU_ACT_GOTO_MID && actLimitSet[1]) {
      g_actuatorNode.setTarget(actLimits[1]);
      d1 = (actLimits[1] + ACTUATOR_STEP_PERCENT/2) / ACTUATOR_STEP_PERCENT;
    } else if (newSel == MENU_ACT_GOTO_BOT && actLimitSet[0]) {
      g_actuatorNode.setTarget(actLimits[0]);
      d1 = (actLimits[0] + ACTUATOR_STEP_PERCENT/2) / ACTUATOR_STEP_PERCENT;
    }
    
    // Update encoder position to match auto-goto
    if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
       g_encoderState.position[1] = d1;
       xSemaphoreGive(encoderStateMutex);
    }
  }

  // 3. Long Press: Set Limit
  if (longPress) {
    Enc1Menu sel = systemState.enc1MenuSelection;
    if (sel != MENU_ACT_MAN) {
      int idx = 2; // Default GOTO_TOP
      if (sel == MENU_ACT_GOTO_MID) idx = 1;
      else if (sel == MENU_ACT_GOTO_BOT) idx = 0;

      int targetPct = 0;
      if (xQueuePeek(actuatorTelQueue, &actTel, 0) == pdPASS) {
        targetPct = (int)actTel.currentPercent;
      }
      
      if (idx == 0) g_actuatorNode.setLimitBot(targetPct);
      else if (idx == 1) g_actuatorNode.setLimitMid(targetPct);
      else if (idx == 2) g_actuatorNode.setLimitTop(targetPct);
      
      LCD_setMessage("Actuator: Set");
      printf("Actuator Limit %d set to %d%%\n", idx, targetPct);
    }
  }

  // 4. Double Press: Clear Limit
  if (doublePress) {
    Enc1Menu sel = systemState.enc1MenuSelection;
    if (sel != MENU_ACT_MAN) {
      int idx = 2;
      if (sel == MENU_ACT_GOTO_MID) idx = 1;
      else if (sel == MENU_ACT_GOTO_BOT) idx = 0;

      if (idx == 0) g_actuatorNode.clearLimitBot();
      else if (idx == 1) g_actuatorNode.clearLimitMid();
      else if (idx == 2) g_actuatorNode.clearLimitTop();
      
      LCD_setMessage("Actuator: Cleared");
      printf("Actuator Limit %d cleared\n", idx);
    }
  }
}

void InputManager::handleMotorEncoder() {
  int32_t currentPos = 0;
  bool btnPressed = false;
  int limit = 15;

  if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    g_encoderState.position[2] =
        constrain(g_encoderState.position[2], -limit, limit);
    currentPos = g_encoderState.position[2];

    if (g_encoderState.buttonPressed[2]) {
      btnPressed = true;
      g_encoderState.buttonPressed[2] = false;
    }
    xSemaphoreGive(encoderStateMutex);
  }

  static bool motorPaused = false;
  static int32_t last_d2 = 0;
  int32_t d2 = currentPos;

  if (motorPaused && (d2 != last_d2)) {
    motorPaused = false;
    LCD_setMessage("Motor: Unpaused");
    printf("Motor Unpaused via Encoder Turn\n");
  }
  last_d2 = d2;

  if (btnPressed) {
    LCD_notifyButtonPress(2);

    motorPaused = !motorPaused;
    LCD_setMessage(motorPaused ? "Motor: PAUSED" : "Motor: RUNNING");
    printf(motorPaused ? "Motor PAUSED\n" : "Motor RUNNING\n");

    if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      g_encoderState.position[2] = 0;
      xSemaphoreGive(encoderStateMutex);
    }
    last_d2 = 0;
    d2 = 0;
  }

  // Ensure speed is only updated if autonomous sequence isn't running
  EventBits_t events = xEventGroupGetBits(controlEvents);
  if (!(events & BIT_AUTO_RUNNING)) {
    int speed = motorPaused ? 0 : (d2 * MOTOR_SPEED_SCALE_FACTOR);
    g_motorNode.setSpeed(speed);
  }
}

void InputManager::handleAutonomousEncoder() {
  bool longPress = false;
  bool shortPress = false;
  bool doublePress = false;
  int32_t delta3 = 0;

  if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (g_encoderState.buttonLongPressed[3]) {
      longPress = true;
      g_encoderState.buttonLongPressed[3] = false;
    }
    if (g_encoderState.buttonDoublePressed[3]) {
      doublePress = true;
      g_encoderState.buttonDoublePressed[3] = false;
    }
    if (g_encoderState.buttonPressed[3]) {
      shortPress = true;
      g_encoderState.buttonPressed[3] = false;
    }

    static int32_t last_d3 = 0;
    delta3 = g_encoderState.position[3] - last_d3;
    last_d3 = g_encoderState.position[3];

    xSemaphoreGive(encoderStateMutex);
  }

  // Read motor telemetry for limit checks
  MotorTelemetry motorTel;
  float motorLimits[3] = {0, 0, 0};
  bool motorLimitSet[3] = {false, false, false};
  if (xQueuePeek(motorTelQueue, &motorTel, pdMS_TO_TICKS(10)) == pdPASS) {
    motorLimits[0] = motorTel.limits[0];
    motorLimits[1] = motorTel.limits[1];
    motorLimits[2] = motorTel.limits[2];
    motorLimitSet[0] = motorTel.limitSet[0];
    motorLimitSet[1] = motorTel.limitSet[1];
    motorLimitSet[2] = motorTel.limitSet[2];
  }

  // Read arm telemetry for calibration check
  ArmTelemetry armTel;
  int posOut = -1, posIn = -1;
  if (xQueuePeek(armTelQueue, &armTel, pdMS_TO_TICKS(10)) == pdPASS) {
    posOut = armTel.posOut;
    posIn = armTel.posIn;
  }

  // 1. Long Press: Motor Limits Setup
  if (longPress) {
    Enc3Menu sel = systemState.enc3MenuSelection;
    if (sel != MENU_AUTO) {
      int idx = 2; // Default GOTO_TOP
      if (sel == MENU_GOTO_MID) idx = 1;
      else if (sel == MENU_GOTO_BOT) idx = 0;

      float currentPos = 0;
      if (xQueuePeek(motorTelQueue, &motorTel, 0) == pdPASS) {
        currentPos = motorTel.currentPosition;
      }

      if (idx == 0) g_motorNode.setLimitBot(currentPos);
      else if (idx == 1) g_motorNode.setLimitMid(currentPos);
      else if (idx == 2) g_motorNode.setLimitTop(currentPos);

      LCD_setMessage("Position Set");
      printf("Limit %d set to %.2f\n", idx, currentPos);
    }
  }

  // 1b. Double Press: Clear Position
  if (doublePress) {
    Enc3Menu sel = systemState.enc3MenuSelection;
    if (sel != MENU_AUTO) {
      int idx = 2;
      if (sel == MENU_GOTO_MID) idx = 1;
      else if (sel == MENU_GOTO_BOT) idx = 0;

      if (idx == 0) g_motorNode.clearLimitBot();
      else if (idx == 1) g_motorNode.clearLimitMid();
      else if (idx == 2) g_motorNode.clearLimitTop();

      LCD_setMessage("Position Cleared");
      printf("Limit %d cleared\n", idx);
    }
  }

  // 2. Encoder Turn: Cycle Menu
  if (delta3 != 0) {
    int sel = (int)systemState.enc3MenuSelection + delta3;
    while (sel < 0) sel += 4;
    sel = sel % 4;
    systemState.enc3MenuSelection = (Enc3Menu)sel;
  }

  // 3. Short Press: Launch, Resume, or E-STOP
  if (shortPress) {
    LCD_notifyButtonPress(3);

    EventBits_t events = xEventGroupGetBits(controlEvents);
    if (!(events & BIT_AUTO_RUNNING)) {
      Enc3Menu sel = systemState.enc3MenuSelection;

      bool canRun = false;
      if (sel == MENU_AUTO) {
        canRun = motorLimitSet[0] && motorLimitSet[1] && motorLimitSet[2] &&
                 (posOut != -1) && (posIn != -1);
      } else {
        int idx = 2;
        if (sel == MENU_GOTO_MID) idx = 1;
        else if (sel == MENU_GOTO_BOT) idx = 0;
        canRun = motorLimitSet[idx];
      }

      if (canRun) {
        if (sel == MENU_AUTO) {
          TaskHandle_t autoTaskHandle = NULL;
          if (xTaskCreate(autonomous_task, "AutoTask", 4096, NULL, 2,
                          &autoTaskHandle) == pdPASS) {
            xEventGroupSetBits(controlEvents, BIT_AUTO_RUNNING);
          } else {
            LCD_setMessage("Error: Task Failed");
            ESP_LOGE("CTRL", "Failed to create autonomous_task");
          }
        } else {
          TaskHandle_t gotoTaskHandle = NULL;
          if (xTaskCreate(motor_goto_task, "GotoTask", 4096, NULL, 2,
                          &gotoTaskHandle) == pdPASS) {
            xEventGroupSetBits(controlEvents, BIT_AUTO_RUNNING);
          } else {
            LCD_setMessage("Error: Task Failed");
          }
        }
      } else {
        LCD_setMessage("Missing Limits");
      }
    } else {
      // Sequence IS running — acts as an E-STOP.
      xEventGroupSetBits(controlEvents,
                         BIT_AUTO_RESUME | BIT_ESTOP_REQUEST);
      LCD_setMessage("Auto: STOPPING...");
    }
  }
}
