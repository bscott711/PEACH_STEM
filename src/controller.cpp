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
                           .motorLimitStep = MOTOR_LIMIT_OFF,
                           .motorLimitBottom = 0.0f,
                           .motorLimitTop = 0.0f,
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
    systemState.servoCalStart = preferences.getInt("srvStart", 0);
    systemState.servoCalCenter = preferences.getInt("srvCenter", 50);

    // Load motor limits from NVS
    systemState.motorLimitStep = (MotorLimitStep)preferences.getInt("limitStep", MOTOR_LIMIT_OFF);
    systemState.motorLimitBottom = preferences.getFloat("limitBot", 0.0f);
    systemState.motorLimitTop = preferences.getFloat("limitTop", 0.0f);

    // Set servo to the saved start position on boot
    systemState.servoTargetPercent = systemState.servoCalStart;
    systemState.servoPercent = systemState.servoCalStart;

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
    preferences.putInt("limitStep", (int)systemState.motorLimitStep);
    preferences.putFloat("limitBot", systemState.motorLimitBottom);
    preferences.putFloat("limitTop", systemState.motorLimitTop);
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

    // Toggle: go to whichever saved position we're furthest from
    int distToStart = abs(currentTarget - calStart);
    int distToCenter = abs(currentTarget - calCenter);
    int newTarget = (distToStart <= distToCenter) ? calCenter : calStart;
    LCD_setMessage(newTarget == calCenter ? "Servo: Center" : "Servo: Start");

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
  int32_t delta3 = 0;

  if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (g_encoderState.buttonLongPressed[3]) {
      longPress = true;
      g_encoderState.buttonLongPressed[3] = false;
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
      if (systemState.motorLimitStep == MOTOR_LIMIT_OFF || systemState.motorLimitStep == MOTOR_LIMIT_SET_2) {
        systemState.currentPosition = 0.0f;
        systemState.motorLimitBottom = 0.0f;
        systemState.motorLimitStep = MOTOR_LIMIT_SET_1;
        LCD_setMessage("Limit 1 Set (0.0)");
        printf("Motor Limit 1 set to 0.0\n");
      } else if (systemState.motorLimitStep == MOTOR_LIMIT_SET_1) {
        float limit2 = systemState.currentPosition;
        if (limit2 < systemState.motorLimitBottom) {
          systemState.motorLimitTop = systemState.motorLimitBottom;
          systemState.motorLimitBottom = limit2;
        } else {
          systemState.motorLimitTop = limit2;
        }
        systemState.motorLimitStep = MOTOR_LIMIT_SET_2;
        callSaveLimits = true;
        LCD_setMessage("Limits Set & Saved");
        printf("Motor Limits set: Bottom=%.2f, Top=%.2f\n", systemState.motorLimitBottom, systemState.motorLimitTop);
      }
      xSemaphoreGive(systemStateMutex);
    }
    
    if (callSaveLimits) {
      saveMotorLimits();
    }
  }

  // 2. Encoder Turn: SG Threshold Tuning (Deactivated for Manual Limits Update)
#if 0
  if (delta3 != 0) {
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      systemState.sgThreshold =
          constrain(systemState.sgThreshold + delta3, 0, 255);
      printf("Enc 3 (SG Threshold): %d\n", systemState.sgThreshold);
      xSemaphoreGive(systemStateMutex);
    }
  }
#endif

  // 3. Short Press: Launch, Resume, or E-STOP
  if (shortPress) {
    LCD_notifyButtonPress(3);

    EventBits_t events = xEventGroupGetBits(controlEvents);
    if (!(events & BIT_AUTO_RUNNING)) {
      // No sequence running — launch a new one
      TaskHandle_t autoTaskHandle = NULL;
      if (xTaskCreate(autonomous_task, "AutoTask", 4096, NULL, 2,
                      &autoTaskHandle) == pdPASS) {
        xEventGroupSetBits(controlEvents, BIT_AUTO_RUNNING);
      } else {
        LCD_setMessage("Error: Task Failed");
        ESP_LOGE("CTRL", "Failed to create autonomous_task");
      }
    } else {
      // Sequence IS running — context-dependent action:
      // BIT_AUTO_RESUME is used by SEQ_WAIT_USER to continue.
      // If the sequence is NOT waiting, this acts as an E-STOP.
      // We fire both bits — the engine checks ESTOP first.
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
  // Read calibration values for servo positions
  int calStart = 0, calCenter = 50;
  if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    calStart = systemState.servoCalStart;
    calCenter = systemState.servoCalCenter;
    xSemaphoreGive(systemStateMutex);
  }

  // clang-format off
  const SequenceStep peachSequence[] = {
    // --- Initialize ---
    {SEQ_MOVE_SERVO,    calStart, 0,              "Auto: Servo Start"},
    {SEQ_MOVE_ACTUATOR, 0,        0,              "Auto: Retract"},
    {SEQ_WAIT_MS,       2500,     0,              NULL},

    // --- Move up to clearance height ---
    {SEQ_MOVE_Z,        0,        Z_CLEARANCE_POS, "Auto: Moving Up"},
    {SEQ_WAIT_MS,       500,      0,               NULL},
    {SEQ_WAIT_USER,     0,        0,               "Press Btn to Continue"},

    // --- Position servo and descend to tube ---
    {SEQ_MOVE_SERVO,    calStart, 0,               "Auto: Servo Start"},
    {SEQ_WAIT_MS,       2500,     0,               NULL},
    {SEQ_MOVE_Z,        0,        Z_TUBE_POS,      "Auto: Down to Tube"},
    {SEQ_WAIT_MS,       500,      0,               NULL},

    // --- Aspiration mixing (3 cycles) ---
    {SEQ_MOVE_ACTUATOR, 80,       0,               "Auto: Aspiration 1"},
    {SEQ_WAIT_MS,       750,      0,               NULL},
    {SEQ_MOVE_ACTUATOR, 0,        0,               NULL},
    {SEQ_WAIT_MS,       750,      0,               NULL},
    {SEQ_MOVE_ACTUATOR, 80,       0,               "Auto: Aspiration 2"},
    {SEQ_WAIT_MS,       750,      0,               NULL},
    {SEQ_MOVE_ACTUATOR, 0,        0,               NULL},
    {SEQ_WAIT_MS,       750,      0,               NULL},
    {SEQ_MOVE_ACTUATOR, 80,       0,               "Auto: Aspiration 3"},
    {SEQ_WAIT_MS,       750,      0,               NULL},
    {SEQ_MOVE_ACTUATOR, 0,        0,               NULL},
    {SEQ_WAIT_MS,       750,      0,               NULL},

    // --- Pickup cells ---
    {SEQ_MOVE_ACTUATOR, 100,      0,               "Auto: Pickup Cells"},
    {SEQ_WAIT_MS,       500,      0,               NULL},
    {SEQ_MOVE_ACTUATOR, 0,        0,               NULL},
    {SEQ_WAIT_MS,       1000,     0,               NULL},

    // --- Retract up and rotate to microscope ---
    {SEQ_MOVE_Z,        0,        Z_CLEARANCE_POS, "Auto: Moving Up"},
    {SEQ_WAIT_MS,       500,      0,               NULL},
    {SEQ_MOVE_SERVO,    calCenter, 0,              "Auto: To Microscope"},
    {SEQ_WAIT_MS,       1000,     0,               NULL},

    // --- Descend to microscope position ---
    {SEQ_MOVE_Z,        0,        Z_TUBE_POS,      "Auto: Descending"},
    {SEQ_WAIT_MS,       500,      0,               NULL},

    // --- Drop cells ---
    {SEQ_MOVE_ACTUATOR, 100,      0,               "Auto: Dropping Cells"},
    {SEQ_WAIT_MS,       1000,     0,               NULL},

    // --- Retract and return home ---
    {SEQ_MOVE_Z,        0,        Z_CLEARANCE_POS, "Auto: Retreating"},
    {SEQ_WAIT_MS,       500,      0,               NULL},
    {SEQ_MOVE_ACTUATOR, 0,        0,               "Auto: Reset Actuator"},
    {SEQ_MOVE_SERVO,    calStart, 0,               "Auto: Reset Servo"},
    {SEQ_MOVE_Z,        0,        Z_TUBE_POS,      "Auto: Return Home"},
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
      if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        currentPos = systemState.currentPosition;
        xSemaphoreGive(systemStateMutex);
      }

      // Set direction based on whether we need to go up or down
      int velocity =
          (step.zTarget > currentPos) ? AUTO_SEQUENCE_SPEED : -AUTO_SEQUENCE_SPEED;
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
        if (goingUp && currentPos >= step.zTarget)
          break;
        if (!goingUp && currentPos <= step.zTarget)
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
      if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        systemState.servoTargetPercent = step.target;
        xSemaphoreGive(systemStateMutex);
      }

      // UI Back-Sync: update encoder 0 so manual controls stay in sync
      if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_encoderState.position[0] = step.target;
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