#include "controller.h"
#include "drivers/EncoderDriver.h"
#include "drivers/LCDDriver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Preferences.h>
#include <cstdio>

Preferences preferences;
SemaphoreHandle_t systemStateMutex;
SemaphoreHandle_t encoderStateMutex;
EventGroupHandle_t controlEvents;

// Instantiate Global System State
SystemState systemState = {.mode = IDLE,
                           .servoAdjustMode = true,
                           .servoPercent = 0,
                           .servoTargetPercent = 0,
                           .servoCalStart = 0,
                           .servoCalCenter = 50,
                           .servoCalStep = CAL_OFF,
                           .actuatorDir = ACT_STOP,
                           .actuatorTargetPercent = 0,
                           .actuatorPercent = 0,
                           .actualSpeed = 0,
                           .targetSpeed = 0,
                           .isHoming = false,
                           .sgDiagMode = false,
                           .sgThreshold = 16,
                           .currentPosition = 0.0f,
                           .isHomed = false,
                           .motorEncoderLimit = 15,
                           .motorLimits = {0.0f, 0.0f, 0.0f},
                           .motorLimitSet = {false, false, false},
                           .enc3MenuSelection = MENU_AUTO,
                           .collisionDetected = false,
                           .collisionTimestamp = 0};

void initSystemState() {
  systemStateMutex = xSemaphoreCreateMutex();
  encoderStateMutex = xSemaphoreCreateMutex();
  controlEvents = xEventGroupCreate();

  if (!preferences.begin("peach", false)) {
    ESP_LOGE("CTRL", "Failed to open NVS namespace");
    LCD_setMessage("NVS Init Error");
    return;
  }

  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    // Always require re-homing on boot (clears stale NVS homing data)
    systemState.isHomed = false;
    systemState.currentPosition = 0.0f;
    preferences.putBool("isHomed", false);
    preferences.putFloat("pos", 0.0f);

    // Load servo calibration from NVS
    systemState.servoCalStart = preferences.getInt("srvStart", -1);
    systemState.servoCalCenter = preferences.getInt("srvCenter", -1);

    // Load motor limits from NVS
    systemState.motorLimits[0] = preferences.getFloat("limB", 0.0f);
    systemState.motorLimits[1] = preferences.getFloat("limM", 0.0f);
    systemState.motorLimits[2] = preferences.getFloat("limT", 0.0f);
    systemState.motorLimitSet[0] = preferences.getBool("limS_B", false);
    systemState.motorLimitSet[1] = preferences.getBool("limS_M", false);
    systemState.motorLimitSet[2] = preferences.getBool("limS_T", false);

    // Set servo to the saved start position on boot
    if (systemState.servoCalStart != -1) {
      systemState.servoTargetPercent = systemState.servoCalStart;
      systemState.servoPercent = systemState.servoCalStart;
    } else {
      systemState.servoTargetPercent = 0;
      systemState.servoPercent = 0;
    }

    xSemaphoreGive(systemStateMutex);
  }
}

void saveMotorState() {
  if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    preferences.putBool("isHomed", systemState.isHomed);
    preferences.putFloat("pos", systemState.currentPosition);
    xSemaphoreGive(systemStateMutex);
    Serial.println("--- Saved Motor State to NVS ---");
  } else {
    ESP_LOGW("CTRL", "saveMotorState mutex timeout, skipping");
  }
}

void saveMotorLimits() {
  if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    preferences.putFloat("limB", systemState.motorLimits[0]);
    preferences.putFloat("limM", systemState.motorLimits[1]);
    preferences.putFloat("limT", systemState.motorLimits[2]);
    preferences.putBool("limS_B", systemState.motorLimitSet[0]);
    preferences.putBool("limS_M", systemState.motorLimitSet[1]);
    preferences.putBool("limS_T", systemState.motorLimitSet[2]);
    xSemaphoreGive(systemStateMutex);
    Serial.println("--- Saved Motor Limits to NVS ---");
  } else {
    ESP_LOGW("CTRL", "saveMotorLimits mutex timeout, skipping");
  }
}

void saveServoCalibration() {
  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    preferences.putInt("srvStart", systemState.servoCalStart);
    preferences.putInt("srvCenter", systemState.servoCalCenter);
    xSemaphoreGive(systemStateMutex);
  }
  Serial.println("--- Saved Servo Calibration to NVS ---");
}

// ------------------------------------------------------------------------
// Sub-Handlers for Controller Readability
// ------------------------------------------------------------------------

static void handleServoEncoder() {
  int32_t currentPos = 0;
  bool btnPressed = false;
  bool longPressed = false;
  bool doublePressed = false;

  // 1. Thread-safe copy and clear
  if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    g_encoderState.position[0] = constrain(
        g_encoderState.position[0], SERVO_MIN_PERCENT, SERVO_MAX_PERCENT);
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

  // Read current calibration state
  ServoCalibrationStep calStep = CAL_OFF;
  int calStart = 0, calCenter = 50;
  if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    calStep = systemState.servoCalStep;
    calStart = systemState.servoCalStart;
    calCenter = systemState.servoCalCenter;
    xSemaphoreGive(systemStateMutex);
  }

  // 1.5 Very Long Press: Clear Calibration
  if (doublePressed && calStep == CAL_OFF) {
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      systemState.servoCalStart = -1;
      systemState.servoCalCenter = -1;
      xSemaphoreGive(systemStateMutex);
    }
    saveServoCalibration();
    LCD_setMessage("CAL: Cleared");
    printf("Servo CAL: Cleared preset positions to -1\n");
    return;
  }

  // 2. Long press: enter calibration mode (only from CAL_OFF)
  if (longPressed && calStep == CAL_OFF) {
    LCD_notifyButtonPress(0);
    LCD_setMessage("CAL: Set START");
    printf("Entering Servo Calibration Mode\n");

    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      systemState.servoCalStep = CAL_SET_START;
      xSemaphoreGive(systemStateMutex);
    }
    return;
  }

  // 3. Calibration mode logic
  if (calStep != CAL_OFF) {
    // In calibration: encoder freely moves servo across full range
    static int32_t calD0 = 0;
    if (calD0 != currentPos) {
      calD0 = currentPos;
      if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        systemState.servoTargetPercent = calD0;
        xSemaphoreGive(systemStateMutex);
      }
    }

    if (btnPressed) {
      LCD_notifyButtonPress(0);

      if (calStep == CAL_SET_START) {
        // Save start position, advance to center
        if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          systemState.servoCalStart = currentPos;
          systemState.servoCalStep = CAL_SET_CENTER;
          xSemaphoreGive(systemStateMutex);
        }
        LCD_setMessage("CAL: Set CENTER");
        printf("Servo CAL: Start=%d, now set center\n", (int)currentPos);

      } else if (calStep == CAL_SET_CENTER) {
        // Save center position, persist to NVS, exit calibration
        if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          systemState.servoCalCenter = currentPos;
          systemState.servoCalStep = CAL_OFF;
          xSemaphoreGive(systemStateMutex);
        }
        saveServoCalibration();
        LCD_setMessage("CAL: Saved!");
        printf("Servo CAL: Center=%d, calibration saved\n", (int)currentPos);

        // Reset encoder position to the start position for normal operation
        int savedStart = 0;
        if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          savedStart = systemState.servoCalStart;
          systemState.servoTargetPercent = savedStart;
          xSemaphoreGive(systemStateMutex);
        }
        if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          g_encoderState.position[0] = savedStart;
          xSemaphoreGive(encoderStateMutex);
        }
        calD0 = savedStart;
      }
    }
    return; // Skip normal mode processing while calibrating
  }

  // 4. Normal mode: encoder adjusts servo target
  static int32_t d0 = 0;
  if (d0 != currentPos) {
    d0 = currentPos;
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      systemState.servoTargetPercent = d0;
      xSemaphoreGive(systemStateMutex);
    }
    printf("Servo Percent: %d\n", (int)d0);
    LCD_setMessage("Servo Adjusted");
  }

  // 5. Short press: toggle between saved start and center positions
  if (btnPressed) {
    LCD_notifyButtonPress(0);

    int currentTarget = 0;
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      currentTarget = systemState.servoTargetPercent;
      xSemaphoreGive(systemStateMutex);
    }

    if (calStart == -1 && calCenter == -1) {
      LCD_setMessage("No presets");
      return;
    }

    int newTarget = currentTarget;
    if (calStart != -1 && calCenter != -1) {
      int distToStart = abs(currentTarget - calStart);
      int distToCenter = abs(currentTarget - calCenter);
      newTarget = (distToStart <= distToCenter) ? calCenter : calStart;
      LCD_setMessage(newTarget == calCenter ? "Servo: Center" : "Servo: Start");
    } else if (calStart != -1) {
      newTarget = calStart;
      LCD_setMessage("Servo: Start");
    } else if (calCenter != -1) {
      newTarget = calCenter;
      LCD_setMessage("Servo: Center");
    }

    if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      g_encoderState.position[0] = newTarget;
      d0 = newTarget;
      xSemaphoreGive(encoderStateMutex);
    }

    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      systemState.servoTargetPercent = newTarget;
      xSemaphoreGive(systemStateMutex);
    }
    printf("Servo Button Pressed. New Pos: %d\n", newTarget);
  }
}

static void handleActuatorEncoder() {
  int32_t currentPos = 0;
  bool btnPressed = false;

  if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    g_encoderState.position[1] = constrain(g_encoderState.position[1], 0, 10);
    currentPos = g_encoderState.position[1];

    if (g_encoderState.buttonPressed[1]) {
      btnPressed = true;
      g_encoderState.buttonPressed[1] = false;
    }
    xSemaphoreGive(encoderStateMutex);
  }

  static int32_t d1 = 0;
  if (d1 != currentPos) {
    d1 = currentPos;
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      systemState.actuatorTargetPercent = d1 * ACTUATOR_STEP_PERCENT;
      xSemaphoreGive(systemStateMutex);
    }
  }

  if (btnPressed) {
    LCD_notifyButtonPress(1);

    int percent = 0;
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      percent = systemState.actuatorTargetPercent;
      xSemaphoreGive(systemStateMutex);
    }

    int newPercent = (percent == 0) ? 50 : (percent == 50) ? 100 : 0;

    if (newPercent == 0 && percent == 100)
      LCD_setMessage("Actuator: 0%");
    else if (newPercent == 0)
      LCD_setMessage("Actuator: Reset");
    else if (newPercent == 50)
      LCD_setMessage("Actuator: 50%");
    else
      LCD_setMessage("Actuator: 100%");

    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      systemState.actuatorTargetPercent = newPercent;
      xSemaphoreGive(systemStateMutex);
    }
    printf("Actuator Button Pressed. New Target: %d%%\n", newPercent);
  }
}

static void handleMotorEncoder() {
  int32_t currentPos = 0;
  bool btnPressed = false;
  int limit = 15;

  if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    limit = systemState.motorEncoderLimit;
    xSemaphoreGive(systemStateMutex);
  }

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
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      systemState.targetSpeed =
          motorPaused ? 0 : (d2 * MOTOR_SPEED_SCALE_FACTOR);
      xSemaphoreGive(systemStateMutex);
    }
  }
}

static void handleAutonomousEncoder() {
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

  // 1. Long Press: Motor Limits Setup
  if (longPress) {
    bool callSaveLimits = false;
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (systemState.enc3MenuSelection != MENU_AUTO) {
        int idx = 2; // Default GOTO_TOP
        if (systemState.enc3MenuSelection == MENU_GOTO_MID) idx = 1;
        else if (systemState.enc3MenuSelection == MENU_GOTO_BOT) idx = 0;

        systemState.motorLimits[idx] = systemState.currentPosition;
        systemState.motorLimitSet[idx] = true;
        callSaveLimits = true;
        LCD_setMessage("Position Set");
        printf("Limit %d set to %.2f\n", idx, systemState.currentPosition);
      }
      xSemaphoreGive(systemStateMutex);
    }
    if (callSaveLimits) saveMotorLimits();
  }

  // 1b. Double Press: Clear Position
  if (doublePress) {
    bool callSaveLimits = false;
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (systemState.enc3MenuSelection != MENU_AUTO) {
        int idx = 2;
        if (systemState.enc3MenuSelection == MENU_GOTO_MID) idx = 1;
        else if (systemState.enc3MenuSelection == MENU_GOTO_BOT) idx = 0;

        systemState.motorLimitSet[idx] = false;
        callSaveLimits = true;
        LCD_setMessage("Position Cleared");
        printf("Limit %d cleared\n", idx);
      }
      xSemaphoreGive(systemStateMutex);
    }
    if (callSaveLimits) saveMotorLimits();
  }

  // 2. Encoder Turn: Cycle Menu
  if (delta3 != 0) {
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      int sel = (int)systemState.enc3MenuSelection + delta3;
      while (sel < 0) sel += 4;
      sel = sel % 4;
      systemState.enc3MenuSelection = (Enc3Menu)sel;
      xSemaphoreGive(systemStateMutex);
    }
  }

  // 3. Short Press: Launch, Resume, or E-STOP
  if (shortPress) {
    LCD_notifyButtonPress(3);

    EventBits_t events = xEventGroupGetBits(controlEvents);
    if (!(events & BIT_AUTO_RUNNING)) {
      Enc3Menu sel = MENU_AUTO;
      if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        sel = systemState.enc3MenuSelection;
        xSemaphoreGive(systemStateMutex);
      }

      bool canRun = false;
      if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (sel == MENU_AUTO) {
          canRun = systemState.motorLimitSet[0] &&
                   systemState.motorLimitSet[1] &&
                   systemState.motorLimitSet[2] &&
                   (systemState.servoCalStart != -1) &&
                   (systemState.servoCalCenter != -1);
        } else {
          int idx = 2;
          if (sel == MENU_GOTO_MID) idx = 1;
          else if (sel == MENU_GOTO_BOT) idx = 0;
          canRun = systemState.motorLimitSet[idx];
        }
        xSemaphoreGive(systemStateMutex);
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

// ------------------------------------------------------------------------
// Main Task Loops
// ------------------------------------------------------------------------

void controller_task(void *pvParameters) {
  // Sync static handler variables on boot
  if (xSemaphoreTake(encoderStateMutex, portMAX_DELAY) == pdTRUE) {
    g_encoderState.position[0] = 0;
    g_encoderState.position[1] = 0;
    g_encoderState.position[2] = 0;
    g_encoderState.position[3] = 0;
    xSemaphoreGive(encoderStateMutex);
  }

  while (1) {
    handleServoEncoder();
    handleActuatorEncoder();
    handleMotorEncoder();
    handleAutonomousEncoder();

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void autonomous_task(void *pvParameters) {
  // ---- Sequence Blueprint ----


  // clang-format off
  const SequenceStep peachSequence[] = {
    // --- Initialize ---
    {SEQ_WAIT_MS,       3000,     0,              "Auto: Starting... "},
    {SEQ_MOVE_ACTUATOR, 0,        0,              "Auto: Retracting"},
    {SEQ_MOVE_Z,        2,        0,              "Auto: To Clearance"}, // 2 = Top
    {SEQ_WAIT_MS,       1000,     0,              NULL},

    // --- Move to Aspirate Position ---
    {SEQ_MOVE_SERVO,    1,        0,              "Auto: Servo Center"}, // 1 = Center
    {SEQ_WAIT_MS,       2500,     0,              NULL},
    {SEQ_MOVE_Z,        1,        0,              "Auto: Descending"},   // 1 = Mid
    {SEQ_WAIT_MS,       1000,     0,              NULL},

    // --- Aspiration Mixing (3 Cycles) ---
    // Push plunger down (100) to expel, Pull (0) to aspirate
    {SEQ_MOVE_ACTUATOR, 100,      0,              "Auto: Mix 1 (Push)"},
    {SEQ_WAIT_MS,       750,      0,              NULL},
    {SEQ_MOVE_ACTUATOR, 0,        0,              "Auto: Mix 1 (Pull)"},
    {SEQ_WAIT_MS,       750,      0,              NULL},

    {SEQ_MOVE_ACTUATOR, 100,      0,              "Auto: Mix 2 (Push)"},
    {SEQ_WAIT_MS,       750,      0,              NULL},
    {SEQ_MOVE_ACTUATOR, 0,        0,              "Auto: Mix 2 (Pull)"},
    {SEQ_WAIT_MS,       750,      0,              NULL},

    {SEQ_MOVE_ACTUATOR, 100,      0,              "Auto: Mix 3 (Push)"},
    {SEQ_WAIT_MS,       750,      0,              NULL},
    {SEQ_MOVE_ACTUATOR, 0,        0,              "Auto: Aspirating"},
    {SEQ_WAIT_MS,       1500,     0,              NULL},

    // --- Move to Dispense Position ---
    {SEQ_MOVE_Z,        2,        0,              "Auto: Up to Clearance"}, // 2 = Top
    {SEQ_WAIT_MS,       500,      0,              NULL},
    {SEQ_MOVE_SERVO,    0,        0,              "Auto: Servo Start"},     // 0 = Start
    {SEQ_WAIT_MS,       1500,     0,              NULL},
    {SEQ_MOVE_Z,        0,        0,              "Auto: Down to Dispense"},// 0 = Bot
    {SEQ_WAIT_MS,       1000,     0,              NULL},

    // --- Dispense Cells ---
    {SEQ_MOVE_ACTUATOR, 100,      0,              "Auto: Dispensing"},
    {SEQ_WAIT_MS,       1500,     0,              NULL},

    // --- Return Home ---
    {SEQ_MOVE_Z,        2,        0,              "Auto: Returning Home"},  // 2 = Top
    {SEQ_WAIT_MS,       500,      0,              NULL},
    {SEQ_MOVE_ACTUATOR, 0,        0,              "Auto: Reset Actuator"},
    {SEQ_WAIT_MS,       1000,     0,              NULL},
  };
  // clang-format on

  const int numSteps = sizeof(peachSequence) / sizeof(peachSequence[0]);

  LCD_setMessage("Auto Sequence Start");
  printf("Starting Autonomous Sequence (%d steps)...\n", numSteps);

  bool aborted = false;

  // ---- Sequence Engine ----
  for (int i = 0; i < numSteps && !aborted; i++) {
    const SequenceStep &step = peachSequence[i];

    // Display step message on LCD if provided
    if (step.message != NULL) {
      LCD_setMessage(step.message);
      printf("[Step %d/%d] %s\n", i + 1, numSteps, step.message);
    }

    switch (step.action) {

    case SEQ_MOVE_Z: {
      // Deterministic Z-axis positioning using virtual position tracking
      float currentPos = 0.0f;
      float targetZ = 0.0f;
      if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        currentPos = systemState.currentPosition;
        targetZ = systemState.motorLimits[step.target];
        xSemaphoreGive(systemStateMutex);
      }

      // Set direction based on whether we need to go up or down
      int velocity =
          (targetZ > currentPos) ? AUTO_SEQUENCE_SPEED : -AUTO_SEQUENCE_SPEED;
      bool goingUp = (velocity > 0);

      if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        systemState.targetSpeed = velocity;
        xSemaphoreGive(systemStateMutex);
      }

      // Poll position until we reach the target, checking for E-STOP
      while (true) {
        EventBits_t ev = xEventGroupGetBits(controlEvents);
        if (ev & BIT_ESTOP_REQUEST) {
          aborted = true;
          break;
        }

        if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          currentPos = systemState.currentPosition;
          xSemaphoreGive(systemStateMutex);
        }

        // Check if we've crossed the target threshold
        if (goingUp && currentPos >= targetZ)
          break;
        if (!goingUp && currentPos <= targetZ)
          break;

        vTaskDelay(pdMS_TO_TICKS(10));
      }

      // Stop the motor
      if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        systemState.targetSpeed = 0;
        xSemaphoreGive(systemStateMutex);
      }

      // Back-sync encoder 2 (motor) to 0
      if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_encoderState.position[2] = 0;
        xSemaphoreGive(encoderStateMutex);
      }
      break;
    }

    case SEQ_MOVE_SERVO: {
      int servoTargetPercent = 0;
      if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        servoTargetPercent = (step.target == 0) ? systemState.servoCalStart : systemState.servoCalCenter;
        systemState.servoTargetPercent = servoTargetPercent;
        xSemaphoreGive(systemStateMutex);
      }

      // UI Back-Sync: update encoder 0 so manual controls stay in sync
      if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_encoderState.position[0] = servoTargetPercent;
        xSemaphoreGive(encoderStateMutex);
      }
      break;
    }

    case SEQ_MOVE_ACTUATOR: {
      if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        systemState.actuatorTargetPercent = step.target;
        xSemaphoreGive(systemStateMutex);
      }

      // UI Back-Sync: update encoder 1 so manual controls stay in sync
      if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_encoderState.position[1] = step.target;
        xSemaphoreGive(encoderStateMutex);
      }
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
      break;
    }

    case SEQ_WAIT_USER: {
      // Wait for user button press OR E-STOP
      // Check RESUME before ESTOP so pressing the button at a prompt resumes
      while (true) {
        EventBits_t ev = xEventGroupWaitBits(
            controlEvents, BIT_AUTO_RESUME | BIT_ESTOP_REQUEST, pdTRUE,
            pdFALSE, pdMS_TO_TICKS(100));

        if (ev & BIT_AUTO_RESUME) {
          // Clear any spurious ESTOP that arrived with the resume
          xEventGroupClearBits(controlEvents, BIT_ESTOP_REQUEST);
          LCD_setMessage("Auto: Resuming");
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
  }   // end for

  // ---- Cleanup ----
  if (aborted) {
    // E-STOP: halt everything immediately
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      systemState.targetSpeed = 0;
      systemState.actuatorTargetPercent = 0;
      xSemaphoreGive(systemStateMutex);
    }
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

// ------------------------------------------------------------------------
// Utility Functions
// ------------------------------------------------------------------------

float motorDistanceCalculator(float speed, int timeInMS) {
  return 1.372e-6f * speed * timeInMS;
}

float motorSpeedCalculator(float position, int timeInMS) {
  return (position / timeInMS) / 1.372e-6f;
}

void motor_goto_task(void *pvParameters) {
  Enc3Menu sel = MENU_AUTO;
  float targetZ = 0.0f;
  
  if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    sel = systemState.enc3MenuSelection;
    if (sel == MENU_GOTO_TOP) targetZ = systemState.motorLimits[2];
    else if (sel == MENU_GOTO_MID) targetZ = systemState.motorLimits[1];
    else if (sel == MENU_GOTO_BOT) targetZ = systemState.motorLimits[0];
    xSemaphoreGive(systemStateMutex);
  }

  LCD_setMessage("Auto: GOTO");
  printf("Starting GOTO target %.2f...\n", targetZ);

  float currentPos = 0.0f;
  if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    currentPos = systemState.currentPosition;
    xSemaphoreGive(systemStateMutex);
  }

  int velocity = (targetZ > currentPos) ? AUTO_SEQUENCE_SPEED : -AUTO_SEQUENCE_SPEED;
  bool goingUp = (velocity > 0);

  if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    systemState.targetSpeed = velocity;
    xSemaphoreGive(systemStateMutex);
  }

  bool aborted = false;
  while (true) {
    EventBits_t ev = xEventGroupGetBits(controlEvents);
    if (ev & BIT_ESTOP_REQUEST) {
      aborted = true;
      break;
    }

    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      currentPos = systemState.currentPosition;
      xSemaphoreGive(systemStateMutex);
    }

    if (goingUp && currentPos >= targetZ) break;
    if (!goingUp && currentPos <= targetZ) break;

    vTaskDelay(pdMS_TO_TICKS(10));
  }

  if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    systemState.targetSpeed = 0;
    xSemaphoreGive(systemStateMutex);
  }

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