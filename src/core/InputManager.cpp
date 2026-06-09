#include "core/InputManager.h"
#include "controller.h"
#include "messaging.h"
#include "tasks/DishLiftNode.h"
#include "tasks/DishRotationNode.h"
#include "tasks/ScraperArmNode.h"
#include "drivers/LCDDriver.h"
#include "drivers/EncoderDriver.h"
#include "core/SequenceEngine.h"
#include "core/NetworkManager.h"
#include <cstdio>
#include <cmath>

extern ScraperArmNode g_scraperArmNode;
extern DishRotationNode g_dishRotationNode;
extern DishLiftNode g_dishLiftNode;

// ============================================================================
// Jog direction tracking (file-scoped for populateUIData access)
// ============================================================================
static int g_jogDirArm = 0;       // -1, 0, +1
static int g_jogDirActuator = 0;
static int g_jogDirMotor = 0;

// Configurable speeds are stored in systemState

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
    // 1. Button feedback
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (int i = 0; i < 4; i++) {
            if (g_encoderState.buttonHeld[i]) {
                uint32_t duration = now - (g_encoderState.buttonPressTime[i] * portTICK_PERIOD_MS);
                if (duration >= 2500) {
                    LCD_setMessage("Very Long Press");
                } else if (duration >= 800) {
                    LCD_setMessage("Long Press");
                } else if (duration >= 100) {
                    LCD_setMessage("Press...");
                }
            }
        }
        xSemaphoreGive(encoderStateMutex);
    }

    handleArmEncoder();
    handleActuatorEncoder();
    handleMotorEncoder();
    handleMenuEncoder();
}

void InputManager::populateUIData(UIData& data) {
    if (xSemaphoreTake(systemStateMutex, 0) == pdTRUE) {
        data.currentMode = systemState.mode;
        data.s4Menu = systemState.s4Menu;
        data.s4SubMenu = systemState.s4SubMenu;
        data.s4InSubMenu = systemState.s4InSubMenu;
        data.s4InSpeedEdit = systemState.s4InSpeedEdit;
        
        data.scraperArmJogSpeed = systemState.scraperArmJogSpeed;
        data.scraperArmGoSpeed = systemState.scraperArmGoSpeed;
        data.dishRotationJogSpeed = systemState.dishRotationJogSpeed;
        data.dishRotationGoSpeed = systemState.dishRotationGoSpeed;
        data.dishLiftJogSpeed = systemState.dishLiftJogSpeed;
        data.dishLiftGoSpeed = systemState.dishLiftGoSpeed;
        xSemaphoreGive(systemStateMutex);
    }
    
    EventBits_t events = xEventGroupGetBits(controlEvents);
    data.isAutoRunning = (events & BIT_AUTO_RUNNING) != 0;

    // Jog direction indicators
    data.scraperArmJogDir = g_jogDirArm;
    data.dishRotationJogDir = g_jogDirActuator;
    data.dishLiftJogDir = g_jogDirMotor;

    // Arm telemetry
    ScraperArmTelemetry armTel;
    if (xQueuePeek(scraperArmTelQueue, &armTel, 0) == pdPASS) {
        data.scraperArmPosition = armTel.currentPosition;
        data.scraperArmPosOut = armTel.posOut;
        data.scraperArmPosBuffer = armTel.posBuffer;
        data.scraperArmPosIn = armTel.posIn;
    }

    // Actuator telemetry
    DishRotationTelemetry actTel;
    if (xQueuePeek(dishRotationTelQueue, &actTel, 0) == pdPASS) {
        data.dishRotationPercent = (int)actTel.currentPercent;
        for(int i=0; i<3; i++) {
            data.dishRotationLimits[i] = actTel.limits[i];
            data.dishRotationLimitSet[i] = actTel.limitSet[i];
        }
    }

    // Motor telemetry
    DishLiftTelemetry motTel;
    if (xQueuePeek(dishLiftTelQueue, &motTel, 0) == pdPASS) {
        data.dishLiftPos = motTel.currentPosition;
        for(int i=0; i<3; i++) {
            data.dishLiftLimits[i] = motTel.limits[i];
            data.dishLiftLimitSet[i] = motTel.limitSet[i];
        }
    }
}

// ============================================================================
// S1: Arm — Simple directional jog
// ============================================================================
void InputManager::handleArmEncoder() {
  static int32_t lastPos = 0;
  int32_t delta = 0;
  bool btnPressed = false;

  if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    delta = g_encoderState.position[0] - lastPos;
    lastPos = g_encoderState.position[0];

    if (g_encoderState.buttonPressed[0]) {
      btnPressed = true;
      g_encoderState.buttonPressed[0] = false;
    }
    // Clear unused button events for S1
    g_encoderState.buttonLongPressed[0] = false;
    g_encoderState.buttonDoublePressed[0] = false;
    xSemaphoreGive(encoderStateMutex);
  }

  // Encoder turn: set jog direction
  if (delta != 0) {
    int speed = 5000;
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      speed = systemState.scraperArmJogSpeed;
      xSemaphoreGive(systemStateMutex);
    }
    if (delta > 0) {
      g_jogDirArm = 1;
      g_scraperArmNode.setSpeed(speed);
    } else {
      g_jogDirArm = -1;
      g_scraperArmNode.setSpeed(-speed);
    }
  }

  // Short press: stop motor
  if (btnPressed) {
    LCD_notifyButtonPress(0);
    g_jogDirArm = 0;
    g_scraperArmNode.setSpeed(0);
    LCD_setMessage("Arm: Stopped");
  }
}

// ============================================================================
// S2: Actuator — Simple directional jog
// ============================================================================
void InputManager::handleActuatorEncoder() {
  static int32_t lastPos = 0;
  int32_t delta = 0;
  bool btnPressed = false;

  if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    delta = g_encoderState.position[1] - lastPos;
    lastPos = g_encoderState.position[1];

    if (g_encoderState.buttonPressed[1]) {
      btnPressed = true;
      g_encoderState.buttonPressed[1] = false;
    }
    // Clear unused button events for S2
    g_encoderState.buttonLongPressed[1] = false;
    g_encoderState.buttonDoublePressed[1] = false;
    xSemaphoreGive(encoderStateMutex);
  }

  // Read current actuator position for stop-in-place
  DishRotationTelemetry actTel;
  int currentPct = 50; // fallback
  if (xQueuePeek(dishRotationTelQueue, &actTel, 0) == pdPASS) {
    currentPct = (int)actTel.currentPercent;
  }

  // Encoder turn: jog actuator
  if (delta != 0) {
    int speed = 128;
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      speed = systemState.dishRotationJogSpeed;
      xSemaphoreGive(systemStateMutex);
    }
    if (delta > 0) {
      g_jogDirActuator = 1;
      g_dishRotationNode.setTarget(100, speed); // Extend toward 100%
    } else {
      g_jogDirActuator = -1;
      g_dishRotationNode.setTarget(0, speed); // Retract toward 0%
    }
  }

  // Short press: stop actuator at current position
  if (btnPressed) {
    LCD_notifyButtonPress(1);
    g_jogDirActuator = 0;
    g_dishRotationNode.setTarget(currentPct, 255);
    LCD_setMessage("Act: Stopped");
  }
}

// ============================================================================
// S3: Z Motor — Simple directional jog
// ============================================================================
void InputManager::handleMotorEncoder() {
  static int32_t lastPos = 0;
  int32_t delta = 0;
  bool btnPressed = false;

  if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    delta = g_encoderState.position[2] - lastPos;
    lastPos = g_encoderState.position[2];

    if (g_encoderState.buttonPressed[2]) {
      btnPressed = true;
      g_encoderState.buttonPressed[2] = false;
    }
    xSemaphoreGive(encoderStateMutex);
  }

  // Encoder turn: set jog direction
  if (delta != 0) {
    int speed = 5000;
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      speed = systemState.dishLiftJogSpeed;
      xSemaphoreGive(systemStateMutex);
    }
    if (delta > 0) {
      g_jogDirMotor = 1;
      g_dishLiftNode.setSpeed(speed);
    } else {
      g_jogDirMotor = -1;
      g_dishLiftNode.setSpeed(-speed);
    }
  }

  // Short press: stop motor
  if (btnPressed) {
    LCD_notifyButtonPress(2);
    g_jogDirMotor = 0;
    g_dishLiftNode.setSpeed(0);
    LCD_setMessage("Z: Stopped");
  }

  // Auto-update jog indicator when motor is stopped by limit
  if (g_jogDirMotor != 0) {
    DishLiftTelemetry motTel;
    if (xQueuePeek(dishLiftTelQueue, &motTel, 0) == pdPASS) {
      if (motTel.targetSpeed == 0) {
        g_jogDirMotor = 0; // Motor hit a limit
      }
    }
  }
}

// ============================================================================
// S4: Unified Hierarchical Menu
// ============================================================================

// Helper: get the number of items in the current sub-menu
static int getSubMenuCount(S4Level0 menu) {
  switch (menu) {
    case S4_SCRAPER: return S4_SCRAPER_COUNT;
    case S4_ROTATION: return S4_POS_COUNT;
    case S4_LIFT:   return S4_POS_COUNT;
    default:     return 0;
  }
}

void InputManager::handleMenuEncoder() {
  bool shortPress = false;
  bool longPress = false;
  bool doublePress = false;
  int32_t delta = 0;

  if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    static int32_t lastPos = 0;
    delta = g_encoderState.position[3] - lastPos;
    lastPos = g_encoderState.position[3];

    if (g_encoderState.buttonPressed[3]) {
      shortPress = true;
      g_encoderState.buttonPressed[3] = false;
    }
    if (g_encoderState.buttonLongPressed[3]) {
      longPress = true;
      g_encoderState.buttonLongPressed[3] = false;
    }
    if (g_encoderState.buttonDoublePressed[3]) {
      doublePress = true;
      g_encoderState.buttonDoublePressed[3] = false;
    }
    xSemaphoreGive(encoderStateMutex);
  }

  // --- Read telemetry for position operations ---
  ScraperArmTelemetry armTel;
  int scraperArmPosOut = -1;
  int scraperArmPosBuffer = -1;
  int scraperArmPosIn = -1;
  float armCurrentPos = 0;
  if (scraperArmTelQueue != NULL && xQueuePeek(scraperArmTelQueue, &armTel, 0) == pdPASS) {
    scraperArmPosOut = armTel.posOut;
    scraperArmPosBuffer = armTel.posBuffer;
    scraperArmPosIn = armTel.posIn;
    armCurrentPos = armTel.currentPosition;
  }

  DishRotationTelemetry actTel;
  int actLimits[3] = {0};
  bool actLimitSet[3] = {false};
  if (xQueuePeek(dishRotationTelQueue, &actTel, 0) == pdPASS) {
    for (int i = 0; i < 3; i++) {
      actLimits[i] = actTel.limits[i];
      actLimitSet[i] = actTel.limitSet[i];
    }
  }

  DishLiftTelemetry motorTel;
  float dishLiftLimits[3] = {0};
  bool dishLiftLimitSet[3] = {false};
  float motorCurrentPos = 0;
  if (xQueuePeek(dishLiftTelQueue, &motorTel, 0) == pdPASS) {
    motorCurrentPos = motorTel.currentPosition;
    for (int i = 0; i < 3; i++) {
      dishLiftLimits[i] = motorTel.limits[i];
      dishLiftLimitSet[i] = motorTel.limitSet[i];
    }
  }

  // Check if auto is running — block menu actions
  EventBits_t events = xEventGroupGetBits(controlEvents);
  bool autoRunning = (events & BIT_AUTO_RUNNING) != 0;

  // ===========================
  // LEVEL 0: Axis selection
  // ===========================
  if (!systemState.s4InSubMenu) {
    // Encoder turn: cycle through Level 0 options
    if (delta != 0) {
      int sel = (int)systemState.s4Menu + delta;
      while (sel < 0) sel += S4_LEVEL0_COUNT;
      sel = sel % S4_LEVEL0_COUNT;
      systemState.s4Menu = (S4Level0)sel;
    }

    // Short press: enter sub-menu or launch Auto
    if (shortPress) {
      LCD_notifyButtonPress(3);

      if (autoRunning) {
        // E-STOP if auto is running
        xEventGroupSetBits(controlEvents, BIT_AUTO_RESUME | BIT_ESTOP_REQUEST);
        LCD_setMessage("Auto: STOPPING...");
        return;
      }

      if (systemState.s4Menu == S4_AUTO) {
        // Launch autonomous sequence (check prerequisites)
        bool canRun = dishLiftLimitSet[0] && dishLiftLimitSet[1] && dishLiftLimitSet[2] &&
                      actLimitSet[0] && actLimitSet[1] && actLimitSet[2] &&
                      (scraperArmPosOut != -1) && (scraperArmPosIn != -1);
        
        if (canRun) {
          TaskHandle_t autoTaskHandle = NULL;
          if (xTaskCreate(autonomous_task, "AutoTask", 4096, (void*)0, 2,
                          &autoTaskHandle) == pdPASS) {
            xEventGroupSetBits(controlEvents, BIT_AUTO_RUNNING);
          } else {
            LCD_setMessage("Error: Task Failed");
            PEACH_LOGE("MENU", "Failed to create autonomous_task");
          }
        } else {
          LCD_setMessage("Missing Limits");
        }
      } else if (systemState.s4Menu == S4_STOP) {
        TaskHandle_t autoTaskHandle = NULL;
        if (xTaskCreate(autonomous_task, "AutoTask", 4096, (void*)1, 2,
                        &autoTaskHandle) == pdPASS) {
          xEventGroupSetBits(controlEvents, BIT_AUTO_RUNNING);
        } else {
          LCD_setMessage("Error: Task Failed");
          PEACH_LOGE("MENU", "Failed to create shutdown task");
        }
      } else {
        // Enter sub-menu for Arm/Act/Z
        systemState.s4InSubMenu = true;
        systemState.s4SubMenu = 0;
      }
    }
    return;
  }

  // ===========================
  // LEVEL 1: Position sub-menu
  // ===========================
  int itemCount = getSubMenuCount(systemState.s4Menu);

  S4Level0 axis = systemState.s4Menu;
  int item = systemState.s4SubMenu;

  // Encoder turn: adjust speed OR cycle through sub-menu items
  if (delta != 0) {
    if (systemState.s4InSpeedEdit) {
      if (axis == S4_SCRAPER) {
        if (item == S4_SCRAPER_JOG_SPD) {
          int stepSize = (systemState.scraperArmJogSpeed <= 100 && delta < 0) || (systemState.scraperArmJogSpeed < 100 && delta > 0) ? 10 : 100;
          systemState.scraperArmJogSpeed += delta * stepSize;
          if (systemState.scraperArmJogSpeed < 10) systemState.scraperArmJogSpeed = 10;
          if (systemState.scraperArmJogSpeed > 5000) systemState.scraperArmJogSpeed = 5000;
        } else if (item == S4_SCRAPER_GO_SPD) {
          int stepSize = (systemState.scraperArmGoSpeed <= 100 && delta < 0) || (systemState.scraperArmGoSpeed < 100 && delta > 0) ? 10 : 100;
          systemState.scraperArmGoSpeed += delta * stepSize;
          if (systemState.scraperArmGoSpeed < 10) systemState.scraperArmGoSpeed = 10;
          if (systemState.scraperArmGoSpeed > 5000) systemState.scraperArmGoSpeed = 5000;
        }
      } else if (axis == S4_ROTATION) {
        if (item == S4_POS_JOG_SPD) {
          systemState.dishRotationJogSpeed += delta * 10;
          if (systemState.dishRotationJogSpeed < 10) systemState.dishRotationJogSpeed = 10;
          if (systemState.dishRotationJogSpeed > 255) systemState.dishRotationJogSpeed = 255;
        } else if (item == S4_POS_GO_SPD) {
          systemState.dishRotationGoSpeed += delta * 10;
          if (systemState.dishRotationGoSpeed < 10) systemState.dishRotationGoSpeed = 10;
          if (systemState.dishRotationGoSpeed > 255) systemState.dishRotationGoSpeed = 255;
        }
      } else if (axis == S4_LIFT) {
        if (item == S4_POS_JOG_SPD) {
          int stepSize = (systemState.dishLiftJogSpeed <= 100 && delta < 0) || (systemState.dishLiftJogSpeed < 100 && delta > 0) ? 10 : 100;
          systemState.dishLiftJogSpeed += delta * stepSize;
          if (systemState.dishLiftJogSpeed < 10) systemState.dishLiftJogSpeed = 10;
          if (systemState.dishLiftJogSpeed > 8500) systemState.dishLiftJogSpeed = 8500;
        } else if (item == S4_POS_GO_SPD) {
          int stepSize = (systemState.dishLiftGoSpeed <= 100 && delta < 0) || (systemState.dishLiftGoSpeed < 100 && delta > 0) ? 10 : 100;
          systemState.dishLiftGoSpeed += delta * stepSize;
          if (systemState.dishLiftGoSpeed < 10) systemState.dishLiftGoSpeed = 10;
          if (systemState.dishLiftGoSpeed > 8500) systemState.dishLiftGoSpeed = 8500;
        }
      }
    } else {
      int sel = systemState.s4SubMenu + delta;
      while (sel < 0) sel += itemCount;
      sel = sel % itemCount;
      systemState.s4SubMenu = sel;
      item = sel; // Update local item
    }
  }

  // --- Check for "Back" item ---
  bool isBack = false;
  if (axis == S4_SCRAPER && item == S4_SCRAPER_BACK) isBack = true;
  if ((axis == S4_ROTATION || axis == S4_LIFT) && item == S4_POS_BACK) isBack = true;

  // ---- Short Press: GOTO or Back ----
  if (shortPress) {
    LCD_notifyButtonPress(3);

    if (autoRunning) {
      xEventGroupSetBits(controlEvents, BIT_AUTO_RESUME | BIT_ESTOP_REQUEST);
      LCD_setMessage("Auto: STOPPING...");
      return;
    }

    if (isBack) {
      systemState.s4InSubMenu = false;
      return;
    }

    // Check if we are interacting with a Speed menu item
    bool isSpeedItem = false;
    if (axis == S4_SCRAPER && (item == S4_SCRAPER_JOG_SPD || item == S4_SCRAPER_GO_SPD)) isSpeedItem = true;
    if ((axis == S4_ROTATION || axis == S4_LIFT) && (item == S4_POS_JOG_SPD || item == S4_POS_GO_SPD)) isSpeedItem = true;

    if (isSpeedItem) {
      if (systemState.s4InSpeedEdit) {
        // Exit edit mode and save
        systemState.s4InSpeedEdit = false;
        if (axis == S4_SCRAPER) {
          StorageManager::saveScraperArmJogSpeed(systemState.scraperArmJogSpeed);
          StorageManager::saveScraperArmGoSpeed(systemState.scraperArmGoSpeed);
        } else if (axis == S4_ROTATION) {
          StorageManager::saveDishRotationJogSpeed(systemState.dishRotationJogSpeed);
          StorageManager::saveDishRotationGoSpeed(systemState.dishRotationGoSpeed);
        } else if (axis == S4_LIFT) {
          StorageManager::saveDishLiftJogSpeed(systemState.dishLiftJogSpeed);
          StorageManager::saveDishLiftGoSpeed(systemState.dishLiftGoSpeed);
        }
        LCD_setMessage("Speed Saved");
      } else {
        // Enter edit mode
        systemState.s4InSpeedEdit = true;
      }
      return;
    }

    // GOTO action based on axis and item
    if (axis == S4_SCRAPER) {
      if (scraperArmPosOut == -1 || scraperArmPosIn == -1) {
        LCD_setMessage("Arm: Not Cal'd");
        return;
      }
      if (item == S4_SCRAPER_TIP) {
        g_scraperArmNode.setTarget(100.0f, systemState.scraperArmGoSpeed);
        LCD_setMessage("Arm: Go Tip");
      } else if (item == S4_SCRAPER_BUFFER) {
        if (scraperArmPosBuffer == -1) {
          LCD_setMessage("Arm: Buf Not Set");
          return;
        }
        float percent = 100.0f * (float)(scraperArmPosBuffer - scraperArmPosOut) / (float)(scraperArmPosIn - scraperArmPosOut);
        g_scraperArmNode.setTarget(percent, systemState.scraperArmGoSpeed);
        LCD_setMessage("Arm: Go Buffer");
      } else if (item == S4_SCRAPER_CLEAR) {
        g_scraperArmNode.setTarget(0.0f, systemState.scraperArmGoSpeed);
        LCD_setMessage("Arm: Go Clear");
      }
    } else if (axis == S4_ROTATION) {
      int limitIdx = (item == S4_POS_TOP) ? 2 : ((item == S4_POS_BOT) ? 0 : 1);
      if (!actLimitSet[limitIdx]) {
        LCD_setMessage("Act: Not Set");
        return;
      }
      g_dishRotationNode.setTarget(actLimits[limitIdx], systemState.dishRotationGoSpeed);
      LCD_setMessage("Act: GOTO");
    } else if (axis == S4_LIFT) {
      int limitIdx = (item == S4_POS_TOP) ? 2 : ((item == S4_POS_BOT) ? 0 : 1);
      if (!dishLiftLimitSet[limitIdx]) {
        LCD_setMessage("Z: Not Set");
        return;
      }

      TaskHandle_t gotoTaskHandle = NULL;
      if (xTaskCreate(motor_goto_task, "GotoTask", 4096,
                      (void*)(intptr_t)limitIdx, 2,
                      &gotoTaskHandle) == pdPASS) {
        xEventGroupSetBits(controlEvents, BIT_AUTO_RUNNING);
      } else {
        LCD_setMessage("Error: Task Failed");
      }
    }
  }

  // ---- Long Press: SET current position ----
  if (longPress && !isBack && systemState.s4InSubMenu) {
    if (axis == S4_SCRAPER) {
      if (item == S4_SCRAPER_TIP) {
        g_scraperArmNode.setPosIn();
        LCD_setMessage("Arm: Tip Set");
        printf("Arm Tip (posIn) set at current position\n");
      } else if (item == S4_SCRAPER_BUFFER) {
        g_scraperArmNode.setPosBuffer();
        LCD_setMessage("Arm: Buffer Set");
        printf("Arm Buffer (posBuffer) set at current position\n");
      } else if (item == S4_SCRAPER_CLEAR) {
        g_scraperArmNode.setPosOut();
        LCD_setMessage("Arm: Clear Set");
        printf("Arm Clear (posOut) set at current position\n");
      }
    } else if (axis == S4_ROTATION) {
      int pct = (int)actTel.currentPercent;
      if (item == S4_POS_TOP) g_dishRotationNode.setLimitTop(pct);
      else if (item == S4_POS_MID) g_dishRotationNode.setLimitMid(pct);
      else if (item == S4_POS_BOT) g_dishRotationNode.setLimitBot(pct);
      LCD_setMessage("Act: Position Set");
      printf("Actuator limit %d set to %d%%\n", item, pct);
    } else if (axis == S4_LIFT) {
      if (item == S4_POS_TOP) {
          g_dishLiftNode.setLimitTop(motorCurrentPos);
      }
      else if (item == S4_POS_MID) g_dishLiftNode.setLimitMid(motorCurrentPos);
      else if (item == S4_POS_BOT) g_dishLiftNode.setLimitBot(motorCurrentPos);
      LCD_setMessage("Z: Position Set");
      printf("Motor limit %d set to %.2f\n", item, motorCurrentPos);
    }
  }

  // ---- Double Press: CLEAR position ----
  if (doublePress && !isBack && systemState.s4InSubMenu) {
    if (axis == S4_SCRAPER) {
      // Clear both arm calibration points
      g_scraperArmNode.clearCal();
      LCD_setMessage("Arm: Cal Cleared");
      printf("Arm calibration cleared\n");
    } else if (axis == S4_ROTATION) {
      if (item == S4_POS_TOP) g_dishRotationNode.clearLimitTop();
      else if (item == S4_POS_MID) g_dishRotationNode.clearLimitMid();
      else if (item == S4_POS_BOT) g_dishRotationNode.clearLimitBot();
      LCD_setMessage("Act: Cleared");
      printf("Actuator limit %d cleared\n", item);
    } else if (axis == S4_LIFT) {
      if (item == S4_POS_TOP) g_dishLiftNode.clearLimitTop();
      else if (item == S4_POS_MID) g_dishLiftNode.clearLimitMid();
      else if (item == S4_POS_BOT) g_dishLiftNode.clearLimitBot();
      LCD_setMessage("Z: Cleared");
      printf("Motor limit %d cleared\n", item);
    }
  }
}
