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
        data.scraperArmSGThreshold = systemState.scraperArmSGThreshold;
        data.scraperArmDropPos = systemState.scraperArmDropPos;
        data.scraperArmTenCur = systemState.scraperArmTenCur;

        data.dishRotationJogSpeed = systemState.dishRotationJogSpeed;
        data.dishRotationGoSpeed = systemState.dishRotationGoSpeed;
        data.dishRotationNumRotations = systemState.dishRotationNumRotations;
        data.dishRotationSGThreshold = systemState.dishRotationSGThreshold;

        data.dishLiftJogSpeed = systemState.dishLiftJogSpeed;
        data.dishLiftGoSpeed = systemState.dishLiftGoSpeed;
        data.dishLiftNumMix = systemState.dishLiftNumMix;
        data.dishLiftSGThreshold = systemState.dishLiftSGThreshold;
        xSemaphoreGive(systemStateMutex);
    }
    
    EventBits_t events = xEventGroupGetBits(controlEvents);
    data.isAutoRunning = (events & BIT_AUTO_RUNNING) != 0;

    // Jog direction indicators
    data.scraperArmJogDir = g_jogDirArm;
    data.dishRotationJogDir = g_jogDirActuator;
    data.dishLiftJogDir = g_jogDirMotor;

    // Arm telemetry
    AxisTelemetry armTel;
    if (xQueuePeek(scraperArmTelQueue, &armTel, 0) == pdPASS) {
        data.scraperArmPosition = armTel.currentPosition;
        data.scraperArmPosClear = (int)armTel.posA;
        data.scraperArmPosScrape = (int)armTel.posB;
        data.scraperArmIsMoving = armTel.isMoving;
        data.scraperArmSGResult = armTel.sgResult;
    }

    // Actuator telemetry
    AxisTelemetry actTel;
    if (xQueuePeek(dishRotationTelQueue, &actTel, 0) == pdPASS) {
        data.dishRotationPos = actTel.currentPosition;
        data.dishRotationIsMoving = actTel.isMoving;
        data.dishRotationSGResult = actTel.sgResult;
    }

    // Z Motor telemetry
    AxisTelemetry zTel;
    if (xQueuePeek(dishLiftTelQueue, &zTel, 0) == pdPASS) {
        data.dishLiftPos = zTel.currentPosition;
        data.dishLiftPosHome = zTel.posA;
        data.dishLiftPosTilt = zTel.posB;
        data.dishLiftPosHomeSet = zTel.posASet;
        data.dishLiftPosTiltSet = zTel.posBSet;
        data.dishLiftIsMoving = zTel.isMoving;
        data.dishLiftSGResult = zTel.sgResult;
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

  // Encoder turn: jog actuator
  if (delta != 0) {
    int speed = 5000;
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      speed = systemState.dishRotationJogSpeed;
      xSemaphoreGive(systemStateMutex);
    }
    if (delta > 0) {
      g_jogDirActuator = 1;
      g_dishRotationNode.setSpeed(speed);
    } else {
      g_jogDirActuator = -1;
      g_dishRotationNode.setSpeed(-speed);
    }
  }

  // Short press: stop actuator at current position
  if (btnPressed) {
    LCD_notifyButtonPress(1);
    g_jogDirActuator = 0;
    g_dishRotationNode.setSpeed(0);
    LCD_setMessage("Rot: Stopped");
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
    AxisTelemetry motTel;
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
  AxisTelemetry armTel;
  int scraperArmPosClear = -1;
  int scraperArmPosScrape = -1;
  float armCurrentPos = 0;
  if (scraperArmTelQueue != NULL && xQueuePeek(scraperArmTelQueue, &armTel, 0) == pdPASS) {
    scraperArmPosClear = armTel.posA;
    scraperArmPosScrape = armTel.posB;
    armCurrentPos = armTel.currentPosition;
  }

  AxisTelemetry actTel;
  float actCurrentPos = 0;
  if (xQueuePeek(dishRotationTelQueue, &actTel, 0) == pdPASS) {
    actCurrentPos = actTel.currentPosition;
  }

  AxisTelemetry motorTel;
  float motorPosHome = 0;
  float motorPosTilt = 0;
  bool motorPosHomeSet = false;
  bool motorPosTiltSet = false;
  float motorCurrentPos = 0;
  if (xQueuePeek(dishLiftTelQueue, &motorTel, 0) == pdPASS) {
    motorCurrentPos = motorTel.currentPosition;
    motorPosHome = motorTel.posA;
    motorPosTilt = motorTel.posB;
    motorPosHomeSet = motorTel.posASet;
    motorPosTiltSet = motorTel.posBSet;
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
        bool canRun = motorPosHomeSet && motorPosTiltSet &&
                      (scraperArmPosClear != -1) && (scraperArmPosScrape != -1);
        
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
  int itemCount = 0;
  if (systemState.s4Menu == S4_SCRAPER) itemCount = S4_SCRAPER_COUNT;
  else if (systemState.s4Menu == S4_ROTATION) itemCount = S4_ROT_COUNT;
  else if (systemState.s4Menu == S4_LIFT) itemCount = S4_LIFT_COUNT;

  S4Level0 axis = systemState.s4Menu;
  int item = systemState.s4SubMenu;

  // Encoder turn: adjust speed/setting OR cycle through sub-menu items
  if (delta != 0) {
    if (systemState.s4InSpeedEdit) {
      if (axis == S4_SCRAPER) {
        if (item == S4_SCRAPER_JOG_SPD || item == S4_SCRAPER_GO_SPD) {
          int& spd = (item == S4_SCRAPER_JOG_SPD) ? systemState.scraperArmJogSpeed : systemState.scraperArmGoSpeed;
          int stepSize = (spd <= 100 && delta < 0) || (spd < 100 && delta > 0) ? 10 : 100;
          spd += delta * stepSize;
          if (spd < 10) spd = 10;
          if (spd > 5000) spd = 5000;
        } else if (item == S4_SCRAPER_SG_TUNE) {
          systemState.scraperArmSGThreshold += delta;
          if (systemState.scraperArmSGThreshold < 0) systemState.scraperArmSGThreshold = 0;
          if (systemState.scraperArmSGThreshold > 255) systemState.scraperArmSGThreshold = 255;
        } else if (item == S4_SCRAPER_TEN_CUR) {
          systemState.scraperArmTenCur += delta * 5;
          if (systemState.scraperArmTenCur < 10) systemState.scraperArmTenCur = 10;
          if (systemState.scraperArmTenCur > 100) systemState.scraperArmTenCur = 100;
        }
      } else if (axis == S4_ROTATION) {
        if (item == S4_ROT_JOG_SPD || item == S4_ROT_GO_SPD) {
          int& spd = (item == S4_ROT_JOG_SPD) ? systemState.dishRotationJogSpeed : systemState.dishRotationGoSpeed;
          int stepSize = (spd <= 100 && delta < 0) || (spd < 100 && delta > 0) ? 10 : 100;
          spd += delta * stepSize;
          if (spd < 10) spd = 10;
          if (spd > 5000) spd = 5000;
        } else if (item == S4_ROT_NUM_ROTATIONS) {
          systemState.dishRotationNumRotations += delta;
          if (systemState.dishRotationNumRotations < 1) systemState.dishRotationNumRotations = 1;
        } else if (item == S4_ROT_SG_TUNE) {
          systemState.dishRotationSGThreshold += delta;
          if (systemState.dishRotationSGThreshold < 0) systemState.dishRotationSGThreshold = 0;
          if (systemState.dishRotationSGThreshold > 255) systemState.dishRotationSGThreshold = 255;
        }
      } else if (axis == S4_LIFT) {
        if (item == S4_LIFT_JOG_SPD || item == S4_LIFT_GO_SPD) {
          int& spd = (item == S4_LIFT_JOG_SPD) ? systemState.dishLiftJogSpeed : systemState.dishLiftGoSpeed;
          int stepSize = (spd <= 100 && delta < 0) || (spd < 100 && delta > 0) ? 10 : 100;
          spd += delta * stepSize;
          if (spd < 10) spd = 10;
          if (spd > 5000) spd = 5000;
        } else if (item == S4_LIFT_NUM_MIX) {
          systemState.dishLiftNumMix += delta;
          if (systemState.dishLiftNumMix < 1) systemState.dishLiftNumMix = 1;
        } else if (item == S4_LIFT_SG_TUNE) {
          systemState.dishLiftSGThreshold += delta;
          if (systemState.dishLiftSGThreshold < 0) systemState.dishLiftSGThreshold = 0;
          if (systemState.dishLiftSGThreshold > 255) systemState.dishLiftSGThreshold = 255;
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
  if (axis == S4_ROTATION && item == S4_ROT_BACK) isBack = true;
  if (axis == S4_LIFT && item == S4_LIFT_BACK) isBack = true;

  // ---- Short Press: GOTO, Edit Settings, or Back ----
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

    // Check if we are interacting with an Edit setting item
    bool isSettingItem = false;
    if (axis == S4_SCRAPER && (item == S4_SCRAPER_JOG_SPD || item == S4_SCRAPER_GO_SPD || item == S4_SCRAPER_SG_TUNE || item == S4_SCRAPER_TEN_CUR)) isSettingItem = true;
    if (axis == S4_ROTATION && (item == S4_ROT_JOG_SPD || item == S4_ROT_GO_SPD || item == S4_ROT_NUM_ROTATIONS || item == S4_ROT_SG_TUNE)) isSettingItem = true;
    if (axis == S4_LIFT && (item == S4_LIFT_JOG_SPD || item == S4_LIFT_GO_SPD || item == S4_LIFT_NUM_MIX || item == S4_LIFT_SG_TUNE)) isSettingItem = true;

    if (isSettingItem) {
      if (systemState.s4InSpeedEdit) {
        // Exit edit mode and save
        systemState.s4InSpeedEdit = false;
        if (axis == S4_SCRAPER) {
          if (item == S4_SCRAPER_JOG_SPD) {
            StorageManager::saveScraperArmJogSpeed(systemState.scraperArmJogSpeed);
          } else if (item == S4_SCRAPER_GO_SPD) {
            StorageManager::saveScraperArmGoSpeed(systemState.scraperArmGoSpeed);
          } else if (item == S4_SCRAPER_TEN_CUR) {
            StorageManager::saveScraperArmTenCur(systemState.scraperArmTenCur);
          } else if (item == S4_SCRAPER_SG_TUNE) {
            StorageManager::saveScraperArmSGThreshold(systemState.scraperArmSGThreshold);
            g_scraperArmNode.setSGThreshold(systemState.scraperArmSGThreshold);
          }
        } else if (axis == S4_ROTATION) {
          if (item == S4_ROT_JOG_SPD) StorageManager::saveDishRotationJogSpeed(systemState.dishRotationJogSpeed);
          else if (item == S4_ROT_GO_SPD) StorageManager::saveDishRotationGoSpeed(systemState.dishRotationGoSpeed);
          else if (item == S4_ROT_NUM_ROTATIONS) StorageManager::saveDishRotationNumRotations(systemState.dishRotationNumRotations);
          else if (item == S4_ROT_SG_TUNE) {
            StorageManager::saveDishRotationSGThreshold(systemState.dishRotationSGThreshold);
            g_dishRotationNode.setSGThreshold(systemState.dishRotationSGThreshold);
          }
        } else if (axis == S4_LIFT) {
          if (item == S4_LIFT_JOG_SPD) StorageManager::saveDishLiftJogSpeed(systemState.dishLiftJogSpeed);
          else if (item == S4_LIFT_GO_SPD) StorageManager::saveDishLiftGoSpeed(systemState.dishLiftGoSpeed);
          else if (item == S4_LIFT_NUM_MIX) StorageManager::saveDishLiftNumMix(systemState.dishLiftNumMix);
          else if (item == S4_LIFT_SG_TUNE) {
            StorageManager::saveDishLiftSGThreshold(systemState.dishLiftSGThreshold);
            g_dishLiftNode.setSGThreshold(systemState.dishLiftSGThreshold);
          }
        }
        LCD_setMessage("Saved");
      } else {
        // Enter edit mode
        systemState.s4InSpeedEdit = true;
      }
      return;
    }

    // GOTO action based on axis and item
    if (axis == S4_SCRAPER) {
      if (scraperArmPosClear == -1 || scraperArmPosScrape == -1) {
        LCD_setMessage("Arm: Not Cal'd");
        return;
      }
      if (item == S4_SCRAPER_CLEAR) {
        g_scraperArmNode.setTarget(0.0f, systemState.scraperArmGoSpeed); // 0%
        LCD_setMessage("Arm: Go Clear");
      } else if (item == S4_SCRAPER_SCRAPE) {
        g_scraperArmNode.setTarget(100.0f, systemState.scraperArmGoSpeed); // 100%
        LCD_setMessage("Arm: Go Scrape");
      }
    } else if (axis == S4_LIFT) {
      if (item == S4_LIFT_HOME) {
        if (!motorPosHomeSet) { LCD_setMessage("Z: Home Not Set"); return; }
        g_dishLiftNode.setTarget(motorPosHome, systemState.dishLiftGoSpeed);
        LCD_setMessage("Z: Go Home");
      } else if (item == S4_LIFT_TILT) {
        if (!motorPosTiltSet) { LCD_setMessage("Z: Tilt Not Set"); return; }
        g_dishLiftNode.setTarget(motorPosTilt, systemState.dishLiftGoSpeed);
        LCD_setMessage("Z: Go Tilt");
      }
    }
  }

  // ---- Long Press: SET current position ----
  if (longPress && !isBack && systemState.s4InSubMenu) {
    if (axis == S4_SCRAPER) {
      if (item == S4_SCRAPER_CLEAR) {
        g_scraperArmNode.setLimitA(armCurrentPos);
        LCD_setMessage("Arm: Clear Set");
      } else if (item == S4_SCRAPER_SCRAPE) {
        g_scraperArmNode.setLimitB(armCurrentPos);
        LCD_setMessage("Arm: Scrape Set");
      }
    } else if (axis == S4_LIFT) {
      if (item == S4_LIFT_HOME) {
        g_dishLiftNode.setLimitA(motorCurrentPos);
        LCD_setMessage("Z: Home Set");
      } else if (item == S4_LIFT_TILT) {
        g_dishLiftNode.setLimitB(motorCurrentPos);
        LCD_setMessage("Z: Tilt Set");
      }
    }
  }

  // ---- Double Press: CLEAR position/calibration ----
  if (doublePress && !isBack && systemState.s4InSubMenu) {
    if (axis == S4_SCRAPER) {
      g_scraperArmNode.clearLimits();
      LCD_setMessage("Arm: Cal Cleared");
    } else if (axis == S4_LIFT) {
      g_dishLiftNode.clearLimits();
      LCD_setMessage("Z: Cal Cleared");
    }
  }
}
