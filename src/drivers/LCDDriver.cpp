#include "drivers/LCDDriver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include <cstdio>
#include <cstring>
#include <esp_log.h>

/**
 * Mutex Lock Order Protocol (ALWAYS acquire in this order to prevent deadlock):
 * 1. lcdMutex          (LCD driver internal state message buffer)
 * 2. encoderStateMutex (g_encoderState - encoder input)
 * 3. systemStateMutex  (SystemState - main control state)
 * * Rule: Never hold a "lower" mutex while waiting for a "higher" one.
 * Rule: Keep critical sections short; copy data to local vars before releasing.
 */

// Instantiation for our LCD screen
U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, LCD_CS, LCD_DC, LCD_RESET);

// ============ Encapsulated LCD Globals ============
static char lcdActionMessage[32] = "";
static bool lcdMessagePending = false;
static uint32_t lcdMessageTimestamp = 0;
static uint32_t lcdBtnPressTime[4] = {0, 0, 0, 0};

// LCD-specific mutex for thread-safe message buffer access
static SemaphoreHandle_t lcdMutex = NULL;

void LCDInit() {
  u8g2.begin();
  u8g2.setFont(u8g2_font_tiny5_tf);

  lcdMutex = xSemaphoreCreateMutex();
  if (lcdMutex == NULL) {
    ESP_LOGE("LCD", "Failed to create LCD string mutex");
  }
}

void LCD_setMessage(const char *msg) {
  if (lcdMutex != NULL &&
      xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    strncpy(lcdActionMessage, msg, sizeof(lcdActionMessage) - 1);
    lcdActionMessage[sizeof(lcdActionMessage) - 1] = '\0';
    lcdMessageTimestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    lcdMessagePending = true;
    xSemaphoreGive(lcdMutex);
  } else {
    ESP_LOGW("LCD", "Mutex timeout setting message");
  }
}

void LCD_notifyButtonPress(int index) {
  if (index >= 0 && index < 4) {
    if (lcdMutex != NULL &&
        xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      lcdBtnPressTime[index] = xTaskGetTickCount() * portTICK_PERIOD_MS;
      xSemaphoreGive(lcdMutex);
    }
  }
}

// ============ Drawing Helpers ============

static void draw_displayTimer() {
  uint32_t t = (xTaskGetTickCount() * portTICK_PERIOD_MS) / 1000;
  uint32_t m = t / 60;
  uint32_t s = t % 60;
  char timerBuffer[32];
  snprintf(timerBuffer, sizeof(timerBuffer), "RUN:%02u:%02u", (unsigned int)m,
           (unsigned int)s);
  u8g2.drawStr(0, 6, timerBuffer);
}

static void draw_buttonStatus() {
  uint32_t localBtnTime[4] = {0};

  // 1. Grab encoder state: clear long-press flags and record press time
  if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    for (int i = 0; i < 4; i++) {
      if (g_encoderState.buttonLongPressed[i]) {
        g_encoderState.buttonLongPressed[i] = false;
        // Notify LCD so the icon flashes briefly, same as short press
        LCD_notifyButtonPress(i);
      }
    }
    xSemaphoreGive(encoderStateMutex);
  } else {
    ESP_LOGW("LCD", "encoderStateMutex timeout in buttonStatus");
  }

  // 2. Grab LCD internal state (lower priority mutex)
  if (lcdMutex != NULL &&
      xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    for (int i = 0; i < 4; i++) {
      localBtnTime[i] = lcdBtnPressTime[i];
    }
    xSemaphoreGive(lcdMutex);
  }

  uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

  // 3. Render using local copies (no mutexes held during drawing)
  for (int i = 0; i < 4; i++) {
    bool recentlyPressed = (now - localBtnTime[i] < 200);

    if (recentlyPressed) {
      u8g2.drawDisc(100 + (i * 6), 4, 2); // filled = pressed
    } else {
      u8g2.drawCircle(100 + (i * 6), 4, 2); // outline = unpressed
    }
  }
}

static void draw_encoderStatus() {
  char statusBuffer[32];
  bool staleData = false;

  // Static caches so the UI doesn't zero out if the mutex times out
  static int servoTarget = 0, servoActual = 0, actuatorTarget = 0,
             motorTarget = 0, sgThreshold = 16;
  static int servoCalStart = 0, servoCalCenter = 50;
  static DeviceMode currentMode = IDLE;
  static bool isHoming = false, sgDiagMode = false;
  static ServoCalibrationStep servoCalStep = CAL_OFF;

  bool triggerHoming = false;

  if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    servoTarget = systemState.servoTargetPercent;
    servoActual = systemState.servoPercent;
    actuatorTarget = systemState.actuatorTargetPercent;
    motorTarget = systemState.targetSpeed;
    sgThreshold = systemState.sgThreshold;
    currentMode = systemState.mode;
    isHoming = systemState.isHoming;
    sgDiagMode = systemState.sgDiagMode;
    servoCalStep = systemState.servoCalStep;
    servoCalStart = systemState.servoCalStart;
    servoCalCenter = systemState.servoCalCenter;
    xSemaphoreGive(systemStateMutex);
  } else {
    staleData = true;
    ESP_LOGW("LCD", "systemStateMutex timeout; showing cached values");
  }

  // Grab the homing request from the Event Group
  EventBits_t events = xEventGroupGetBits(controlEvents);
  triggerHoming = (events & BIT_HOMING_REQUEST) != 0;

  // Draw visual warning if data is stale
  if (staleData) {
    u8g2.drawStr(115, 6, "[!]");
  }

  // Encoder 0: Servo
  if (servoCalStep == CAL_SET_START) {
    snprintf(statusBuffer, sizeof(statusBuffer), "S0:CAL START:%03d%%",
             servoTarget);
    u8g2.drawStr(0, 16, statusBuffer);
  } else if (servoCalStep == CAL_SET_CENTER) {
    snprintf(statusBuffer, sizeof(statusBuffer), "S0:CAL CTR:%03d%%",
             servoTarget);
    u8g2.drawStr(0, 16, statusBuffer);
  } else {
    snprintf(statusBuffer, sizeof(statusBuffer), "S0:Srv:%03d%%", servoTarget);
    u8g2.drawStr(0, 16, statusBuffer);

    // Live sliding dot: track from x=72 to x=124, y centered at 13
    const int trackL = 72, trackR = 124, trackY = 13;
    u8g2.drawHLine(trackL, trackY, trackR - trackL);

    // Tick marks at calibrated start and center positions
    int startX = map(constrain(servoCalStart, 0, 100), 0, 100, trackL, trackR);
    int centerX = map(constrain(servoCalCenter, 0, 100), 0, 100, trackL, trackR);
    u8g2.drawVLine(startX, trackY - 2, 5);
    u8g2.drawVLine(centerX, trackY - 2, 5);

    // Filled dot at the actual servo position (real-time from servo_task)
    int dotX = map(constrain(servoActual, 0, 100), 0, 100, trackL, trackR);
    u8g2.drawDisc(dotX, trackY, 2);
  }

  // Encoder 1: Actuator
  snprintf(statusBuffer, sizeof(statusBuffer), "S1:Act:%03d%%", actuatorTarget);
  u8g2.drawStr(0, 24, statusBuffer);

  if (actuatorTarget > 50) {
    u8g2.drawTriangle(75, 18, 71, 24, 79, 24); // UP
  } else if (actuatorTarget < 50) {
    u8g2.drawTriangle(75, 24, 71, 18, 79, 18); // DOWN
  } else {
    u8g2.drawDisc(75, 21, 2); // Center dot
  }

  // Encoder 2: Motor
  int step = motorTarget / MOTOR_SPEED_SCALE_FACTOR;
  snprintf(statusBuffer, sizeof(statusBuffer), "S2:Mot:%+03d", step);
  u8g2.drawStr(0, 32, statusBuffer);

  u8g2.drawFrame(51, 25, 69, 9);
  u8g2.drawLine(85, 25, 85, 33);

  int clicks = constrain(abs(step), 0, 15);

  if (motorTarget > 0) {
    u8g2.drawBox(87, 27, clicks * 2, 5);
  } else if (motorTarget < 0) {
    u8g2.drawBox(83 - (clicks * 2), 27, clicks * 2, 5);
  }

  if (motorTarget == 0 && currentMode != IDLE) {
    u8g2.drawStr(88, 32, "[P]");
  }

  // Encoder 3: StallGuard threshold + flags
  snprintf(statusBuffer, sizeof(statusBuffer), "S3:SG:%03d", sgThreshold);
  u8g2.drawStr(0, 40, statusBuffer);
  if (isHoming)
    u8g2.drawStr(72, 40, "[H]");
  else if (sgDiagMode)
    u8g2.drawStr(72, 40, "[D]");
  else if (triggerHoming)
    u8g2.drawStr(72, 40, "[*]");
}

static void draw_actionMessage() {
  bool pending = false;
  char localMsg[32] = "";
  uint32_t timestamp = 0;

  // Safely read the message buffer
  if (lcdMutex != NULL &&
      xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    pending = lcdMessagePending;
    timestamp = lcdMessageTimestamp;
    if (pending) {
      strncpy(localMsg, lcdActionMessage, sizeof(localMsg) - 1);
      localMsg[sizeof(localMsg) - 1] = '\0';
    }
    xSemaphoreGive(lcdMutex);
  }

  uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

  // Show message for 2.5 seconds after being set
  if (pending && (now - timestamp < 2500)) {
    char actionBuffer[48];
    snprintf(actionBuffer, sizeof(actionBuffer), "> %s", localMsg);
    u8g2.drawStr(0, 60, actionBuffer);
  } else if (pending) {
    // Safely auto-expire the flag
    if (lcdMutex != NULL &&
        xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      lcdMessagePending = false;
      xSemaphoreGive(lcdMutex);
    }
  }
}

// ============ Main Draw Function ============

void draw_menu() {
  u8g2.clearBuffer();

  draw_displayTimer();
  draw_buttonStatus();
  u8g2.drawHLine(0, 10, 128);

  draw_encoderStatus();
  u8g2.drawHLine(0, 46, 128);

  draw_actionMessage();

  u8g2.sendBuffer(); // Push to OLED
}