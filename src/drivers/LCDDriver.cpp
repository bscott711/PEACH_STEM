#include "drivers/LCDDriver.h"
#include "messaging.h"
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
 * 3. systemStateMutex  (SystemState - UI-specific states ONLY: mode, enc selections)
 * * Rule: Never hold a "lower" mutex while waiting for a "higher" one.
 * Rule: Keep critical sections short; copy data to local vars before releasing.
 * Note: Motion subsystem state is read lock-free via telemetry queues.
 */

// Instantiation for our LCD screen
U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI u8g2(U8G2_R0, LCD_CS, LCD_DC, LCD_RESET);

// ============ Encapsulated LCD Globals ============
static char lcdActionMessage[32] = "";
static bool lcdMessagePending = false;
static uint32_t lcdMessageTimestamp = 0;
static uint32_t lcdBtnPressTime[4] = {0, 0, 0, 0};

// LCD-specific mutex for thread-safe message buffer access
static SemaphoreHandle_t lcdMutex = NULL;

static void draw_peachLogo() {
  // === Half-peach graphic (far left, centered vertically) ===
  const int cx = 22, cy = 34, r = 22;

  // Draw the full peach body
  u8g2.drawDisc(cx, cy, r);

  // Cut it in half: clear the left side to create the flat cut face
  u8g2.setDrawColor(0);
  u8g2.drawBox(0, cy - r - 1, cx - 4, r * 2 + 2);
  u8g2.setDrawColor(1);

  // Flat cut face (vertical line)
  u8g2.drawVLine(cx - 4, cy - r + 2, r * 2 - 4);

  // Inner flesh ring to give depth
  u8g2.setDrawColor(0);
  u8g2.drawCircle(cx, cy, r - 4);
  u8g2.setDrawColor(1);

  // Pit: concentric circles in the center
  u8g2.drawDisc(cx + 2, cy, 7, U8G2_DRAW_ALL);
  u8g2.setDrawColor(0);
  u8g2.drawDisc(cx + 2, cy, 4, U8G2_DRAW_ALL);
  u8g2.setDrawColor(1);
  // Pit texture
  u8g2.drawPixel(cx + 2, cy);
  u8g2.drawPixel(cx + 3, cy - 1);
  u8g2.drawPixel(cx + 1, cy + 1);

  // Stem at top
  u8g2.drawLine(cx + 3, cy - r + 1, cx + 6, cy - r - 5);
  u8g2.drawLine(cx + 4, cy - r + 1, cx + 7, cy - r - 5);

  // Small leaf
  u8g2.drawEllipse(cx + 10, cy - r - 3, 5, 2);
  u8g2.drawLine(cx + 7, cy - r - 4, cx + 14, cy - r - 3);
}

static void draw_splashScreen() {
  u8g2.clearBuffer();

  draw_peachLogo();

  // === Text (right of peach) ===
  u8g2.setFont(u8g2_font_helvB14_tr);
  u8g2.drawStr(50, 30, "PEACH");

  u8g2.setFont(u8g2_font_helvB18_tr);
  u8g2.drawStr(66, 54, "PIT");

  // Small version tag
  u8g2.setFont(u8g2_font_tiny5_tf);
  u8g2.drawStr(50, 62, "v2.0");

  u8g2.sendBuffer();
}

void draw_wifiStatus(const char* status, const char* ssid, int attempt, bool failed) {
  u8g2.clearBuffer();

  draw_peachLogo();

  // WiFi Connection text on the right
  u8g2.setFont(u8g2_font_helvB08_tr);
  u8g2.drawStr(50, 16, "WiFi Connect");
  u8g2.drawHLine(50, 20, 78);

  u8g2.setFont(u8g2_font_tiny5_tf);
  char ssidBuf[32];
  snprintf(ssidBuf, sizeof(ssidBuf), "SSID: %s", ssid);
  u8g2.drawStr(50, 32, ssidBuf);

  u8g2.drawStr(50, 44, status);

  // Draw loading dots or fail warning
  if (failed) {
    u8g2.drawStr(50, 56, "Rebooting in 5s...");
  } else {
    // Show some simple animation based on attempt
    char anim[16] = "";
    int dotCount = (attempt % 4);
    for (int i = 0; i < dotCount; i++) {
      strcat(anim, ".");
    }
    u8g2.drawStr(50, 56, anim);
  }

  u8g2.sendBuffer();
}

void draw_otaScreen() {
  u8g2.clearBuffer();

  // Premium Header
  u8g2.setFont(u8g2_font_helvB08_tr);
  u8g2.drawStr(16, 10, "FIRMWARE UPDATE");
  u8g2.drawHLine(0, 14, 128);

  // Status message
  u8g2.setFont(u8g2_font_tiny5_tf);
  int statusWidth = strlen(g_otaStatus) * 4;
  u8g2.drawStr((128 - statusWidth) / 2, 26, g_otaStatus);

  // Progress Bar Frame
  u8g2.drawFrame(10, 32, 108, 12);
  
  // Progress Bar Fill
  int fillWidth = map(constrain(g_otaProgress, 0, 100), 0, 100, 0, 106);
  if (fillWidth > 0) {
    u8g2.drawBox(11, 33, fillWidth, 10);
  }

  // Progress text (centered at bottom)
  char progressStr[32];
  snprintf(progressStr, sizeof(progressStr), "%d%% completed", g_otaProgress);
  int textWidth = strlen(progressStr) * 4;
  u8g2.drawStr((128 - textWidth) / 2, 56, progressStr);

  u8g2.sendBuffer();
}

void LCDInit() {
  u8g2.begin();
  u8g2.setBusClock(4000000); // Lower SPI speed to 4MHz to prevent screen tearing
  u8g2.setFont(u8g2_font_tiny5_tf);

  lcdMutex = xSemaphoreCreateMutex();
  if (lcdMutex == NULL) {
    ESP_LOGE("LCD", "Failed to create LCD string mutex");
  }

  // Show splash screen for 2.5 seconds
  draw_splashScreen();
  vTaskDelay(pdMS_TO_TICKS(2500));

  // Restore the small UI font
  u8g2.setFont(u8g2_font_tiny5_tf);
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

static void draw_buttonStatus(const UIData& data) {
  uint32_t localBtnTime[4] = {0};

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

static void draw_encoderStatus(const UIData& data) {
  char statusBuffer[32];

  int armTarget = data.armTarget;
  int armActual = data.armActual;
  int armPosOut = data.armPosOut;
  int armPosIn = data.armPosIn;

  int actuatorTarget = data.actuatorTarget;
  int actuatorActual = data.actuatorActual;

  int motorTarget = data.motorTargetSpeed;
  float currentPos = data.motorPos;

  DeviceMode currentMode = data.currentMode;
  Enc3Menu enc3MenuSelection = data.enc3Menu;
  Enc1Menu enc1MenuSelection = data.enc1Menu;

  // Encoder 0: Arm — Row at y=11, text baseline y=17
  // We don't have access to controller's internal calStep, so we just show Arm data
  snprintf(statusBuffer, sizeof(statusBuffer), "S0:Arm:%03d%%", armTarget);
  u8g2.drawStr(0, 17, statusBuffer);

  // Live sliding dot: track from x=72 to x=124, y centered at 14
  const int trackL = 72, trackR = 124, trackY = 14;
  u8g2.drawHLine(trackL, trackY, trackR - trackL);

  // Only draw track if calibrated
  if (armPosOut != -1 && armPosIn != -1 && armPosIn != armPosOut) {
    int startX = trackL;
    int centerX = trackR;
    
    u8g2.drawVLine(startX, trackY - 2, 5);
    u8g2.drawVLine(centerX, trackY - 2, 5);

    int dotX = map(armActual, armPosOut, armPosIn, trackL, trackR);
    dotX = constrain(dotX, trackL, trackR);
    u8g2.drawDisc(dotX, trackY, 2);
  } else {
    // Uncalibrated: just show a dot bouncing around or center it
    u8g2.drawDisc((trackL+trackR)/2, trackY, 2);
  }

  // Encoder 1: Actuator — Row at y=20, text baseline y=26
  if (enc1MenuSelection == MENU_ACT_MAN) {
    snprintf(statusBuffer, sizeof(statusBuffer), "S1:Man:%03d%%",
             actuatorTarget);
  } else if (enc1MenuSelection == MENU_ACT_GOTO_TOP) {
    snprintf(statusBuffer, sizeof(statusBuffer), "S1:ToTop:%03d%%",
             actuatorTarget);
  } else if (enc1MenuSelection == MENU_ACT_GOTO_MID) {
    snprintf(statusBuffer, sizeof(statusBuffer), "S1:ToMid:%03d%%",
             actuatorTarget);
  } else if (enc1MenuSelection == MENU_ACT_GOTO_BOT) {
    snprintf(statusBuffer, sizeof(statusBuffer), "S1:ToBot:%03d%%",
             actuatorTarget);
  }
  u8g2.drawStr(0, 26, statusBuffer);

  // Fill bar: frame from x=72 to x=124, height 7
  {
    const int barL = 72, barW = 52, barY = 20, barH = 7;
    u8g2.drawFrame(barL, barY, barW, barH);
    int fillW = map(constrain(actuatorActual, 0, 100), 0, 100, 0, barW - 2);
    if (fillW > 0) {
      u8g2.drawBox(barL + 1, barY + 1, fillW, barH - 2);
    }
  }

  // Encoder 2: Motor — Row at y=29, text baseline y=35
  int step = motorTarget / MOTOR_SPEED_SCALE_FACTOR;
  snprintf(statusBuffer, sizeof(statusBuffer), "S2:Mot:%+03d", step);
  u8g2.drawStr(0, 35, statusBuffer);

  // Bidirectional speed bar: frame from x=72 to x=124, height 7, center line
  {
    const int barL = 72, barW = 52, barY = 29, barH = 7;
    int centerX = barL + barW / 2;
    u8g2.drawFrame(barL, barY, barW, barH);
    u8g2.drawVLine(centerX, barY, barH);

    int clicks = constrain(abs(step), 0, 15);
    int fillW = map(clicks, 0, 15, 0, (barW / 2) - 1);

    if (motorTarget > 0) {
      u8g2.drawBox(centerX + 1, barY + 1, fillW, barH - 2);
    } else if (motorTarget < 0) {
      u8g2.drawBox(centerX - fillW, barY + 1, fillW, barH - 2);
    }

    if (motorTarget == 0 && currentMode != IDLE) {
      u8g2.drawStr(centerX + 2, 35, "[P]");
    }
  }

  if (enc3MenuSelection == MENU_AUTO) {
    snprintf(statusBuffer, sizeof(statusBuffer), "S3:Cmd:Auto");
  } else if (enc3MenuSelection == MENU_GOTO_TOP) {
    snprintf(statusBuffer, sizeof(statusBuffer), "S3:Cmd:To Top");
  } else if (enc3MenuSelection == MENU_GOTO_MID) {
    snprintf(statusBuffer, sizeof(statusBuffer), "S3:Cmd:To Mid");
  } else if (enc3MenuSelection == MENU_GOTO_BOT) {
    snprintf(statusBuffer, sizeof(statusBuffer), "S3:Cmd:To Bot");
  }
  u8g2.drawStr(0, 44, statusBuffer);

  bool botSet = data.motorLimitSet[0];
  bool topSet = data.motorLimitSet[2];
  float minLim = data.motorLimits[0];
  float maxLim = data.motorLimits[2];

  if (botSet && topSet && maxLim > minLim) {
    // Sliding dot UI: track from x=72 to x=124, y centered at 41
    const int trackL = 72, trackR = 124, trackY = 41;
    u8g2.drawHLine(trackL, trackY, trackR - trackL);

    float range = maxLim - minLim;
    if (range > 0.01f) {
      for (int i = 0; i < 3; i++) {
        if (data.motorLimitSet[i]) {
          float val = data.motorLimits[i];
          if (val < minLim)
            val = minLim;
          if (val > maxLim)
            val = maxLim;
          int tickX =
              trackL + (int)(((val - minLim) / range) * (trackR - trackL));
          u8g2.drawVLine(tickX, trackY - 2, 5);
        }
      }

      float constrainedPos = currentPos;
      if (constrainedPos < minLim)
        constrainedPos = minLim;
      if (constrainedPos > maxLim)
        constrainedPos = maxLim;

      int dotX = trackL +
                 (int)(((constrainedPos - minLim) / range) * (trackR - trackL));
      u8g2.drawDisc(dotX, trackY, 2);
    }
  }

#if 0
  // Encoder 3: StallGuard — Row at y=38, text baseline y=44
  snprintf(statusBuffer, sizeof(statusBuffer), "S3:SG:%03d", sgThreshold);
  u8g2.drawStr(0, 44, statusBuffer);
  if (isHoming)
    u8g2.drawStr(72, 44, "[H]");
  else if (sgDiagMode)
    u8g2.drawStr(72, 44, "[D]");
  else if (triggerHoming)
    u8g2.drawStr(72, 44, "[*]");
#endif
}

static void draw_actionMessage(const UIData& data) {
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
  bool isAuto = data.isAutoRunning;

  // Show message for 2.5 seconds after being set, or infinitely if in Auto
  // sequence
  if (pending && (isAuto || (now - timestamp < 2500))) {
    char actionBuffer[48];
    snprintf(actionBuffer, sizeof(actionBuffer), "> %s", localMsg);
    u8g2.drawStr(0, 60, actionBuffer);
  } else if (pending && !isAuto) {
    // Safely auto-expire the flag
    if (lcdMutex != NULL &&
        xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      lcdMessagePending = false;
      xSemaphoreGive(lcdMutex);
    }
  }
}

// ============ Main Draw Function ============

void draw_menu(const UIData& data) {
  u8g2.clearBuffer();

  draw_displayTimer();
  draw_buttonStatus(data);
  u8g2.drawHLine(0, 10, 128);

  draw_encoderStatus(data);
  u8g2.drawHLine(0, 46, 128);

  draw_actionMessage(data);

  u8g2.sendBuffer(); // Push to OLED
}