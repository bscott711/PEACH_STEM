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
                           .actuatorDir = ACT_STOP,
                           .actuatorTargetPercent = 0,
                           .actualSpeed = 0,
                           .targetSpeed = 0,
                           .isHoming = false,
                           .sgDiagMode = false,
                           .sgThreshold = 16,
                           .currentPosition = 0.0f,
                           .isHomed = false,
                           .motorEncoderLimit = 15,
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
    systemState.isHomed = preferences.getBool("isHomed", false);
    systemState.currentPosition = preferences.getFloat("pos", 0.0f);
    xSemaphoreGive(systemStateMutex);
  }
}

void saveMotorState() {
  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    preferences.putBool("isHomed", systemState.isHomed);
    preferences.putFloat("pos", systemState.currentPosition);
    xSemaphoreGive(systemStateMutex);
  }
  Serial.println("--- Saved Motor State to NVS ---");
}

// ------------------------------------------------------------------------
// Sub-Handlers for Controller Readability
// ------------------------------------------------------------------------

static void handleServoEncoder() {
  int32_t currentPos = 0;
  bool btnPressed = false;

  // 1. Thread-safe copy and clear
  if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    g_encoderState.position[0] = constrain(
        g_encoderState.position[0], SERVO_MIN_PERCENT, SERVO_MAX_PERCENT);
    currentPos = g_encoderState.position[0];

    if (g_encoderState.buttonPressed[0]) {
      btnPressed = true;
      g_encoderState.buttonPressed[0] = false; // Clear immediately
    }
    xSemaphoreGive(encoderStateMutex);
  }

  // 2. Process logic safely
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

  if (btnPressed) {
    LCD_notifyButtonPress(0);

    int currentTarget = 0;
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      currentTarget = systemState.servoTargetPercent;
      xSemaphoreGive(systemStateMutex);
    }

    int newTarget = (currentTarget <= SERVO_CENTER_PERCENT)
                        ? SERVO_CENTER_PERCENT
                        : SERVO_MIN_PERCENT;
    LCD_setMessage(newTarget == SERVO_CENTER_PERCENT ? "Servo: Center"
                                                     : "Servo: Start");

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

  // 1. Long Press: Homing Request via Event Group
  if (longPress) {
    printf("\nTriggering Hardware Sensorless Homing...\n");
    LCD_setMessage("Homing Started...");
    xEventGroupSetBits(controlEvents, BIT_HOMING_REQUEST);
  }

  // 2. Encoder Turn: SG Threshold Tuning
  if (delta3 != 0) {
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      systemState.sgThreshold =
          constrain(systemState.sgThreshold + delta3, 0, 255);
      printf("Enc 3 (SG Threshold): %d\n", systemState.sgThreshold);
      xSemaphoreGive(systemStateMutex);
    }
  }

  // 3. Short Press: Launch or Resume Autonomous Sequence
  if (shortPress) {
    LCD_notifyButtonPress(3);

    EventBits_t events = xEventGroupGetBits(controlEvents);
    if (!(events & BIT_AUTO_RUNNING)) {
      TaskHandle_t autoTaskHandle = NULL;
      if (xTaskCreate(autonomous_task, "AutoTask", 4096, NULL, 2,
                      &autoTaskHandle) == pdPASS) {
        xEventGroupSetBits(controlEvents, BIT_AUTO_RUNNING);
      } else {
        LCD_setMessage("Error: Task Failed");
        ESP_LOGE("CTRL", "Failed to create autonomous_task");
      }
    } else {
      // If sequence is active and waiting for user input, this triggers the
      // resume
      xEventGroupSetBits(controlEvents, BIT_AUTO_RESUME);
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
  LCD_setMessage("Auto Sequence Start");
  printf("Starting Autonomous Sequence...\n");

  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    systemState.targetSpeed = 0;
    systemState.actuatorTargetPercent = 0;
    systemState.servoTargetPercent = 0;
    xSemaphoreGive(systemStateMutex);
  }
  vTaskDelay(pdMS_TO_TICKS(2500));

  LCD_setMessage("Auto: Moving Away");
  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    systemState.targetSpeed = AUTO_SEQUENCE_SPEED;
    xSemaphoreGive(systemStateMutex);
  }
  vTaskDelay(pdMS_TO_TICKS(AUTO_SEQUENCE_DURATION_MS));

  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    systemState.targetSpeed = 0;
    xSemaphoreGive(systemStateMutex);
  }
  vTaskDelay(pdMS_TO_TICKS(500));

  LCD_setMessage("Press Btn to Continue");

  // Wait indefinitely for the user to press the button (which fires
  // BIT_AUTO_RESUME) pdTRUE automatically clears the bit so it doesn't trigger
  // again immediately
  xEventGroupWaitBits(controlEvents, BIT_AUTO_RESUME, pdTRUE, pdFALSE,
                      portMAX_DELAY);

  LCD_setMessage("Auto: Resuming");

  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    systemState.servoTargetPercent = 0;
    xSemaphoreGive(systemStateMutex);
  }
  vTaskDelay(pdMS_TO_TICKS(2500));

  LCD_setMessage("Auto: Down to Tube");
  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    systemState.targetSpeed = -AUTO_SEQUENCE_SPEED;
    xSemaphoreGive(systemStateMutex);
  }
  vTaskDelay(pdMS_TO_TICKS(AUTO_SEQUENCE_DURATION_MS));

  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    systemState.targetSpeed = 0;
    xSemaphoreGive(systemStateMutex);
  }
  vTaskDelay(pdMS_TO_TICKS(500));

  LCD_setMessage("Auto: Aspiration");
  for (int i = 0; i < 3; i++) {
    if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
      systemState.actuatorTargetPercent = 80;
      xSemaphoreGive(systemStateMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(750));

    if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
      systemState.actuatorTargetPercent = 0;
      xSemaphoreGive(systemStateMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(750));
  }

  LCD_setMessage("Auto: Pickup Cells");
  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    systemState.actuatorTargetPercent = 100;
    xSemaphoreGive(systemStateMutex);
  }
  vTaskDelay(pdMS_TO_TICKS(500));

  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    systemState.actuatorTargetPercent = 0;
    xSemaphoreGive(systemStateMutex);
  }
  vTaskDelay(pdMS_TO_TICKS(1000));

  LCD_setMessage("Auto: Moving Up");
  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    systemState.targetSpeed = AUTO_SEQUENCE_SPEED;
    xSemaphoreGive(systemStateMutex);
  }
  vTaskDelay(pdMS_TO_TICKS(AUTO_SEQUENCE_DURATION_MS));

  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    systemState.targetSpeed = 0;
    xSemaphoreGive(systemStateMutex);
  }
  vTaskDelay(pdMS_TO_TICKS(500));

  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    systemState.servoTargetPercent = 30;
    xSemaphoreGive(systemStateMutex);
  }
  vTaskDelay(pdMS_TO_TICKS(1000));

  LCD_setMessage("Auto: To Microscope");
  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    systemState.targetSpeed = -AUTO_SEQUENCE_SPEED;
    xSemaphoreGive(systemStateMutex);
  }
  vTaskDelay(pdMS_TO_TICKS(AUTO_SEQUENCE_DURATION_MS));

  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    systemState.targetSpeed = 0;
    xSemaphoreGive(systemStateMutex);
  }
  vTaskDelay(pdMS_TO_TICKS(500));

  LCD_setMessage("Auto: Dropping Cells");
  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    systemState.actuatorTargetPercent = 100;
    xSemaphoreGive(systemStateMutex);
  }
  vTaskDelay(pdMS_TO_TICKS(1000));

  LCD_setMessage("Auto: Retreating");
  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    systemState.targetSpeed = AUTO_SEQUENCE_SPEED;
    xSemaphoreGive(systemStateMutex);
  }
  vTaskDelay(pdMS_TO_TICKS(AUTO_SEQUENCE_DURATION_MS));

  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    systemState.targetSpeed = 0;
    systemState.actuatorTargetPercent = 0;
    xSemaphoreGive(systemStateMutex);
  }

  LCD_setMessage("Auto: Complete");
  vTaskDelay(pdMS_TO_TICKS(500));

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