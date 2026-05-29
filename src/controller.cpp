#include "controller.h"
#include "messaging.h"
#include "tasks/ServoNode.h"
#include "tasks/ActuatorNode.h"
#include "tasks/MotorNode.h"
#include "drivers/EncoderDriver.h"
#include "drivers/LCDDriver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Preferences.h>
#include <cstdio>

// Global queue handles (declared extern in controller.h)
QueueHandle_t servoCmdQueue;
QueueHandle_t servoTelQueue;
QueueHandle_t actuatorCmdQueue;
QueueHandle_t actuatorTelQueue;
QueueHandle_t motorCmdQueue;
QueueHandle_t motorTelQueue;

// --- OTA & WiFi Global States ---
volatile bool g_otaActive = false;
volatile int g_otaProgress = 0;
const char* g_otaStatus = "Initializing";

// Global Node instances (defined in main.cpp, extern here)
extern ServoNode g_servoNode;
extern ActuatorNode g_actuatorNode;
extern MotorNode g_motorNode;

Preferences preferences;
SemaphoreHandle_t systemStateMutex;
SemaphoreHandle_t encoderStateMutex;
EventGroupHandle_t controlEvents;

SystemState systemState = {.mode = IDLE,
                           .enc1MenuSelection = MENU_ACT_MAN_FAST,
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
  
  // Initialize minimal state - subsystem state is managed by Active Nodes
  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    systemState.mode = IDLE;
    systemState.enc1MenuSelection = MENU_ACT_MAN_FAST;
    systemState.enc3MenuSelection = MENU_AUTO;
    systemState.collisionDetected = false;
    xSemaphoreGive(systemStateMutex);
  }
}

void saveMotorState() {
  // Motor node manages its own NVS saves internally
  ESP_LOGI("CTRL", "Motor state saved by MotorNode");
}

void saveMotorLimits() {
  // Motor and Actuator nodes manage their own NVS saves
  ESP_LOGI("CTRL", "Limits saved by respective Nodes");
}

void saveServoCalibration() {
  // Servo node manages its own NVS saves
  ESP_LOGI("CTRL", "Servo calibration saved by ServoNode");
}

void saveActuatorLimits() {
  // Actuator node manages its own NVS saves
  ESP_LOGI("CTRL", "Actuator limits saved by ActuatorNode");
}

// ============================================================================
// Sub-Handlers for Controller Readability - Refactored for Message Passing
// ============================================================================

static void handleServoEncoder() {
  int32_t currentPos = 0;
  bool btnPressed = false;
  bool longPressed = false;
  bool doublePressed = false;

  // 1. Thread-safe copy and clear encoder state
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

  // 2. Read current calibration state from servo telemetry
  ServoTelemetry servoTel;
  int calStart = -1, calCenter = -1;
  float currentPercent = 0;
  if (xQueuePeek(servoTelQueue, &servoTel, pdMS_TO_TICKS(10)) == pdPASS) {
    calStart = servoTel.calStart;
    calCenter = servoTel.calCenter;
    currentPercent = servoTel.currentPercent;
  }

  // 3. Very Long Press (Double): Clear Calibration
  if (doublePressed && calStart == -1 && calCenter == -1) {
    // Already cleared
    LCD_setMessage("CAL: Already Cleared");
    return;
  }
  if (doublePressed) {
    // Clear calibration by sending commands
    g_servoNode.setCalStart(-1);
    g_servoNode.setCalCenter(-1);
    LCD_setMessage("CAL: Cleared");
    printf("Servo CAL: Cleared preset positions to -1\n");
    return;
  }

  // 4. Long press: enter calibration mode
  if (longPressed) {
    LCD_notifyButtonPress(0);
    LCD_setMessage("CAL: Set START");
    printf("Entering Servo Calibration Mode\n");
    
    // In calibration mode, we directly command the servo via messages
    // The servo node will track position internally
    return;
  }

  // 5. Calibration mode logic - read from telemetry to check if calibrating
  // For simplicity, we track calibration state locally during the session
  static ServoCalibrationStep calStep = CAL_OFF;
  static int32_t calD0 = 0;
  
  if (calStep != CAL_OFF) {
    // Update servo target based on encoder position
    if (calD0 != currentPos) {
      calD0 = currentPos;
      g_servoNode.setTarget((float)currentPos);
    }

    if (btnPressed) {
      LCD_notifyButtonPress(0);

      if (calStep == CAL_SET_START) {
        // Save start position, advance to center
        g_servoNode.setCalStart(currentPos);
        calStep = CAL_SET_CENTER;
        LCD_setMessage("CAL: Set CENTER");
        printf("Servo CAL: Start=%d, now set center\n", (int)currentPos);

      } else if (calStep == CAL_SET_CENTER) {
        // Save center position, exit calibration
        g_servoNode.setCalCenter(currentPos);
        calStep = CAL_OFF;
        LCD_setMessage("CAL: Saved!");
        printf("Servo CAL: Center=%d, calibration saved\n", (int)currentPos);

        // Reset encoder position to the start position for normal operation
        int savedStart = calStart;
        if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          g_encoderState.position[0] = savedStart;
          xSemaphoreGive(encoderStateMutex);
        }
        calD0 = savedStart;
        g_servoNode.setTarget((float)savedStart);
      }
    }
    return; // Skip normal mode processing while calibrating
  }

  // Handle entering calibration mode from CAL_OFF
  if (longPressed && calStep == CAL_OFF) {
    calStep = CAL_SET_START;
    LCD_setMessage("CAL: Set START");
    return;
  }

  // 6. Normal mode: encoder adjusts servo target
  static int32_t d0 = 0;
  if (d0 != currentPos) {
    d0 = currentPos;
    g_servoNode.setTarget((float)d0);
    printf("Servo Percent: %d\n", (int)d0);
    LCD_setMessage("Servo Adjusted");
  }

  // 7. Short press: toggle between saved start and center positions
  if (btnPressed) {
    LCD_notifyButtonPress(0);

    if (calStart == -1 && calCenter == -1) {
      LCD_setMessage("No presets");
      return;
    }

    int newTarget = (int)currentPercent;
    if (calStart != -1 && calCenter != -1) {
      int distToStart = abs((int)currentPercent - calStart);
      int distToCenter = abs((int)currentPercent - calCenter);
      newTarget = (distToStart <= distToCenter) ? calCenter : calStart;
    } else if (calStart != -1) {
      newTarget = calStart;
    } else if (calCenter != -1) {
      newTarget = calCenter;
    }

    // Interlock: Block movement to Start if Z-axis is not at Top clearance
    if (newTarget == calStart) {
      MotorTelemetry motorTel;
      bool atTop = false;
      if (xQueuePeek(motorTelQueue, &motorTel, pdMS_TO_TICKS(10)) == pdPASS) {
        atTop = (motorTel.limitSet[2] && motorTel.currentPosition >= motorTel.limits[2] - 5.0f);
      }
      if (!atTop) {
        LCD_setMessage("Raise Z-Axis First!");
        printf("Servo Interlock: Blocked move to Start because Z-axis is not at Top.\n");
        return; // Abort
      }
    }

    LCD_setMessage(newTarget == calCenter ? "Servo: Center" : "Servo: Start");

    if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      g_encoderState.position[0] = newTarget;
      d0 = newTarget;
      xSemaphoreGive(encoderStateMutex);
    }

    g_servoNode.setTarget((float)newTarget);
    printf("Servo Button Pressed. New Pos: %d\n", newTarget);
  }
}

static void handleActuatorEncoder() {
  int32_t currentPos = 0;
  bool btnPressed = false;
  bool longPress = false;
  bool doublePress = false;

  if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    g_encoderState.position[1] = constrain(g_encoderState.position[1], 0, 10);
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
    ActSpeed s = (systemState.enc1MenuSelection == MENU_ACT_MAN_SLOW) ? ActSpeed::SLOW : ActSpeed::FAST;
    g_actuatorNode.setTarget(targetPct, s);
  }

  // 2. Short Press: Cycle Menu and Execute GOTO
  if (btnPressed) {
    LCD_notifyButtonPress(1);
    
    Enc1Menu sel = systemState.enc1MenuSelection;
    int newSel = (int)sel + 1;
    if (newSel > 4) newSel = 0; // 5 menu items (0-4)
    systemState.enc1MenuSelection = (Enc1Menu)newSel;
    
    // Auto-GOTO if the limit is set
    if (newSel == MENU_ACT_GOTO_TOP && actLimitSet[2]) {
      g_actuatorNode.setTarget(actLimits[2], ActSpeed::FAST);
      d1 = actLimits[2] / ACTUATOR_STEP_PERCENT;
    } else if (newSel == MENU_ACT_GOTO_MID && actLimitSet[1]) {
      g_actuatorNode.setTarget(actLimits[1], ActSpeed::FAST);
      d1 = actLimits[1] / ACTUATOR_STEP_PERCENT;
    } else if (newSel == MENU_ACT_GOTO_BOT && actLimitSet[0]) {
      g_actuatorNode.setTarget(actLimits[0], ActSpeed::FAST);
      d1 = actLimits[0] / ACTUATOR_STEP_PERCENT;
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
    if (sel != MENU_ACT_MAN_FAST && sel != MENU_ACT_MAN_SLOW) {
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
    if (sel != MENU_ACT_MAN_FAST && sel != MENU_ACT_MAN_SLOW) {
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

static void handleMotorEncoder() {
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

  // Read servo telemetry for calibration check
  ServoTelemetry servoTel;
  int calStart = -1, calCenter = -1;
  if (xQueuePeek(servoTelQueue, &servoTel, pdMS_TO_TICKS(10)) == pdPASS) {
    calStart = servoTel.calStart;
    calCenter = servoTel.calCenter;
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
                 (calStart != -1) && (calCenter != -1);
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

// ============================================================================
// Main Controller Task
// ============================================================================

void controller_task(void *pvParameters) {
  // Sync static handler variables on boot
  if (xSemaphoreTake(encoderStateMutex, portMAX_DELAY) == pdTRUE) {
    g_encoderState.position[0] = 0;
    g_encoderState.position[1] = 0;
    g_encoderState.position[2] = 0;
    g_encoderState.position[3] = 0;
    xSemaphoreGive(encoderStateMutex);
  }

  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t CONTROLLER_INTERVAL = pdMS_TO_TICKS(50);

  while (1) {
    // Process encoder inputs and dispatch commands
    handleServoEncoder();
    handleActuatorEncoder();
    handleMotorEncoder();
    handleAutonomousEncoder();

    vTaskDelayUntil(&lastWakeTime, CONTROLLER_INTERVAL);
  }
}

// ============================================================================
// Autonomous Sequence Task (unchanged logic, uses queues for state access)
// ============================================================================

void autonomous_task(void *pvParameters) {
  // Define the autonomous sequence steps
  const SequenceStep sequence[] = {
      {SEQ_MOVE_Z, 0, Z_CLEARANCE_POS, "Auto: Raise Z"},
      {SEQ_MOVE_SERVO, 0, 0, "Auto: Gripper Open"},
      {SEQ_WAIT_MS, 500, 0, "Wait 500ms"},
      {SEQ_MOVE_Z, 0, Z_TUBE_POS, "Auto: Lower Z"},
      {SEQ_MOVE_SERVO, 100, 0, "Auto: Gripper Close"},
      {SEQ_WAIT_MS, 500, 0, "Wait 500ms"},
      {SEQ_MOVE_Z, 0, Z_CLEARANCE_POS, "Auto: Raise Z"},
      {SEQ_MOVE_ACTUATOR, 1, 0, "Auto: Extend"},
      {SEQ_WAIT_MS, 1000, 0, "Wait 1s"},
      {SEQ_MOVE_SERVO, 0, 0, "Auto: Gripper Open"},
      {SEQ_WAIT_USER, 0, 0, "Press to Drop"},
      {SEQ_MOVE_SERVO, 100, 0, "Auto: Gripper Close"},
      {SEQ_MOVE_ACTUATOR, 0, 0, "Auto: Retract"},
      {SEQ_WAIT_MS, 500, 0, "Wait 500ms"},
      {SEQ_MOVE_Z, 0, Z_TUBE_POS, "Auto: Lower Z"},
      {SEQ_MOVE_SERVO, 0, 0, "Auto: Gripper Open"},
      {SEQ_WAIT_MS, 500, 0, "Wait 500ms"},
      {SEQ_MOVE_Z, 0, Z_CLEARANCE_POS, "Auto: Raise Z"},
      {SEQ_MOVE_ACTUATOR, 1, 0, "Auto: Extend"},
      {SEQ_WAIT_MS, 500, 0, "Wait 500ms"}};

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

    switch (step.action) {
    case SEQ_MOVE_Z: {
      // Send speed command to motor node
      int velocity = (step.zTarget > 0) ? AUTO_SEQUENCE_SPEED : -AUTO_SEQUENCE_SPEED;
      
      // Read current position from motor telemetry
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
      break;
    }

    case SEQ_MOVE_SERVO: {
      ServoTelemetry servoTel;
      int calStart = 0, calCenter = 50;
      bool atTop = false;

      if (xQueuePeek(servoTelQueue, &servoTel, pdMS_TO_TICKS(10)) == pdPASS) {
        calStart = servoTel.calStart;
        calCenter = servoTel.calCenter;
      }

      // Check if at top using motor telemetry
      MotorTelemetry motorTel;
      if (xQueuePeek(motorTelQueue, &motorTel, pdMS_TO_TICKS(10)) == pdPASS) {
        atTop = (motorTel.limitSet[2] && motorTel.currentPosition >= motorTel.limits[2] - 5.0f);
      }

      // Interlock
      if (step.target == 0 && !atTop) {
        LCD_setMessage("Auto Abort: Z not Top");
        printf("Autonomous Sequence Aborted: Cannot move servo to Start while Z is not at Top!\n");
        aborted = true;
        break;
      }

      int servoTargetPercent = (step.target == 0) ? calStart : calCenter;
      g_servoNode.setTarget((float)servoTargetPercent);

      // UI Back-Sync: update encoder 0 so manual controls stay in sync
      if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_encoderState.position[0] = servoTargetPercent;
        xSemaphoreGive(encoderStateMutex);
      }
      break;
    }

    case SEQ_MOVE_ACTUATOR: {
      ActuatorTelemetry actTel;
      int actuatorTargetPercent = 0;
      if (xQueuePeek(actuatorTelQueue, &actTel, pdMS_TO_TICKS(10)) == pdPASS) {
        actuatorTargetPercent = actTel.limits[step.target];
      }
      g_actuatorNode.setTarget(actuatorTargetPercent);

      // UI Back-Sync: update encoder 1 so manual controls stay in sync
      if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_encoderState.position[1] = actuatorTargetPercent / ACTUATOR_STEP_PERCENT;
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

// ============================================================================
// Motor GOTO Task (uses queues for state access)
// ============================================================================

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

// ============================================================================
// Utility Functions
// ============================================================================

float motorDistanceCalculator(float speed, int timeInMS) {
  return 1.372e-6f * speed * timeInMS;
}

float motorSpeedCalculator(float position, int timeInMS) {
  return (position / timeInMS) / 1.372e-6f;
}
