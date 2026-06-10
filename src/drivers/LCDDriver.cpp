#include "drivers/LCDDriver.h"
#include "messaging.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include <cstdio>
#include <cstring>
#include "core/NetworkManager.h"
#include "HardwareConfig.h"
#include "controller.h"
#include <U8g2lib.h>
#include <SPI.h>

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

static int fallingPeachLeafIndex = -1;
static int currentLeaf = 0;
static void drawBranch(int progress, float rootX, float x, float y, float len, float angle, int depth, int pathIndex);

// Deterministic hash function for organic but stable procedural generation
static float hash_fn(int a, int b) {
  float h = sinf(a * 12.9898f + b * 78.233f) * 43758.5453123f;
  return h - floorf(h);
}

static float clamp(float val, float min, float max) {
  if (val < min) return min;
  if (val > max) return max;
  return val;
}

// Recursive function to draw the tree dynamically
static void drawBranch(int progress, float rootX, float x, float y, float len, float angle, int depth, int pathIndex) {
  const int maxDepth = 4; // Max depth 4 for 128x64 screen performance

  // Calculate the timing for this specific depth
  float startP, endP;
  if (depth == 0) {
    startP = 0; endP = 20;
  } else {
    float depthSlice = 30.0f / maxDepth;
    startP = 20 + (depth - 1) * depthSlice;
    endP = startP + depthSlice;
  }

  float growth = clamp((progress - startP) / (endP - startP), 0.0f, 1.0f);

  if (growth > 0) {
    float currentLen = len * growth;
    float endX = x + currentLen * sinf(angle);
    float endY = y - currentLen * cosf(angle);

    // Draw the branch line
    u8g2.drawLine((int)x, (int)y, (int)endX, (int)endY);

    // --- STAGE 2: The Branches (20% - 50%) ---
    if (depth < maxDepth && progress > endP) {
      // Generate procedural but deterministic branch angles and lengths
      float leftLenMod = 0.65f + hash_fn(pathIndex, 1) * 0.15f;
      float rightLenMod = 0.65f + hash_fn(pathIndex, 2) * 0.15f;
      // Increased branch angles to spread them out more
      float leftAngleMod = 0.45f + hash_fn(pathIndex, 3) * 0.4f;
      float rightAngleMod = 0.45f + hash_fn(pathIndex, 4) * 0.4f;

      // Spawn left and right children
      drawBranch(progress, rootX, endX, endY, len * leftLenMod, angle - leftAngleMod, depth + 1, pathIndex * 2);
      drawBranch(progress, rootX, endX, endY, len * rightLenMod, angle + rightAngleMod, depth + 1, pathIndex * 2 + 1);

      // Occasionally spawn a middle branch for organic density
      // Increased probability of middle branch for a fuller tree
      if (hash_fn(pathIndex, 5) > 0.5f) { 
         drawBranch(progress, rootX, endX, endY, len * 0.5f, angle + (hash_fn(pathIndex, 6) * 0.2f - 0.1f), depth + 1, pathIndex * 2 + 2);
      }
    }

    // --- STAGE 3: The Leaves (50% - 80%) ---
    // Increased leaf density for a fuller look (70% chance of leaf)
    if (depth >= 2 && hash_fn(pathIndex, 11) > 0.3f) {
      float leafStart = 50 + hash_fn(pathIndex, 7) * 15;
      float leafEnd = leafStart + 15;
      
      float leafGrowth = clamp((progress - leafStart) / (leafEnd - leafStart), 0.0f, 1.0f);
      
      if (leafGrowth > 0) {
        // Just draw a small circle for leaves on the OLED
        int r = (int)(2.0f * leafGrowth);
        if (r > 0) {
            u8g2.drawDisc((int)endX, (int)endY, r);
        }
      }
    }

    // --- STAGE 4: The Peaches (80% - 100%) ---
    if (depth == maxDepth) {
      currentLeaf++;
      
      // Only spawn exactly three peaches
      if (currentLeaf == 8 || currentLeaf == 25 || currentLeaf == 45) {
        if (progress > 80) {
          float peachGrowth = clamp((progress - 80.0f) / 10.0f, 0.0f, 1.0f); // Grow faster (80-90)
          
          float pY = endY;
          
          // Pick the first peach that is far enough from the center to be the falling peach
          if (fallingPeachLeafIndex == -1 && abs((int)endX - (int)rootX) > 12) {
              fallingPeachLeafIndex = currentLeaf;
          }
          
          // Peach falling animation (90% - 100%)
          if (progress > 90 && currentLeaf == fallingPeachLeafIndex) {
               float fallProgress = clamp((progress - 90.0f) / 10.0f, 0.0f, 1.0f);
               // Quadratic fall: distance = 1/2 * g * t^2
               pY += (62.0f - endY) * fallProgress * fallProgress; 
               // Stop at ground
               if (pY > 60.0f) pY = 60.0f;
          }

          if (peachGrowth > 0) {
            int radius = (int)(4.0f * peachGrowth); // slightly smaller peaches
            if (radius > 0) {
                // Draw small peach body (circle with cleft line)
                u8g2.drawDisc((int)endX, (int)pY, radius);
                u8g2.setDrawColor(0);
                u8g2.drawLine((int)endX, (int)(pY - radius), (int)endX, (int)(pY + radius - 1));
                u8g2.setDrawColor(1);
            }
          }
        }
      }
    }
  }
}



static void draw_splashScreen() {
  u8g2.clearBuffer();

  currentLeaf = 0;
  fallingPeachLeafIndex = -1;
  drawBranch(100, 32.0f, 32.0f, 64.0f, 12.0f, 0.0f, 0, 1);

  // === Text (right of peach) ===
  u8g2.setFont(u8g2_font_helvB14_tr);
  u8g2.drawStr(50, 30, "PEACH");

  u8g2.setFont(u8g2_font_helvB18_tr);
  u8g2.drawStr(50, 54, "STEM");

  // Small version tag
  u8g2.setFont(u8g2_font_profont10_tf); // Small font
  u8g2.drawStr(50, 62, "v1.0dev11");

  u8g2.sendBuffer();
}

void draw_wifiStatus(const char* status, const char* ssid, int attempt, bool failed) {
  u8g2.clearBuffer();

  currentLeaf = 0;
  fallingPeachLeafIndex = -1;
  drawBranch(100, 32.0f, 32.0f, 64.0f, 12.0f, 0.0f, 0, 1);

  // WiFi Connection text on the right
  u8g2.setFont(u8g2_font_helvB08_tr);
  u8g2.drawStr(56, 16, "WiFi Connect");
  u8g2.drawHLine(56, 20, 72);

  u8g2.setFont(u8g2_font_tiny5_tf);
  char ssidBuf[32];
  snprintf(ssidBuf, sizeof(ssidBuf), "SSID: %s", ssid);
  u8g2.drawStr(56, 32, ssidBuf);

  u8g2.drawStr(56, 44, status);

  // Draw loading dots or fail warning
  if (failed) {
    u8g2.drawStr(56, 56, "Rebooting in 5s...");
  } else {
    // Show some simple animation based on attempt
    char anim[16] = "";
    int dotCount = (attempt % 4);
    for (int i = 0; i < dotCount; i++) {
      strcat(anim, ".");
    }
    u8g2.drawStr(56, 56, anim);
  }

  u8g2.sendBuffer();
}

void draw_otaScreen() {
  u8g2.clearBuffer();

  int otaProgress = NetworkManager::getOTAProgress();
  const char* otaStatus = NetworkManager::getOTAStatus();

  // Reset falling peach tracker for this frame
  fallingPeachLeafIndex = -1;
  currentLeaf = 0;

  // Draw the growing tree!
  int startX = 64; // Center
  int startY = 64; // Bottom of screen
  float baseTrunkLength = 12.0f; // Shorter trunk
  
  drawBranch(otaProgress, startX, startX, startY, baseTrunkLength, 0.0f, 0, 1);

  // UI Overlay - Top Left
  u8g2.setFont(u8g2_font_tiny5_tf);
  u8g2.drawStr(0, 10, "PEACH STEM UPDATING");

  u8g2.sendBuffer();
}

void LCDInit() {
  SPI.begin(LCD_SCK, -1, LCD_MOSI, -1); // Prevent SPI from taking over pin 19 (MISO) which we use for CS
  u8g2.begin();
  u8g2.setBusClock(4000000); // Lower SPI speed to 4MHz to prevent screen tearing
  u8g2.sendF("ca", 0xd5, 0xf0); // Maximize internal oscillator freq to fix camera flicker
  u8g2.setFont(u8g2_font_tiny5_tf);

  lcdMutex = xSemaphoreCreateMutex();
  if (lcdMutex == NULL) {
    PEACH_LOGE("LCD", "Failed to create LCD string mutex");
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
    PEACH_LOGW("LCD", "Mutex timeout setting message");
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

static const char* s4Level0Names[] = {"Shutdown", "Arm", "Rot", "Z", "Auto"};
static const char* s4ArmSubNames[] = {"Clear", "Scrape", "JogSpd", "GoSpd", "SGTune", "Back"};
static const char* s4RotSubNames[] = {"JogSpd", "GoSpd", "NumRot", "SGTune", "Back"};
static const char* s4LiftSubNames[] = {"Home", "Tilt", "JogSpd", "GoSpd", "NumMix", "SGTune", "Back"};

static void draw_encoderStatus(const UIData& data) {
  char statusBuffer[32];

  auto dirChar = [](int dir) -> char {
    if (dir > 0) return '>';
    if (dir < 0) return '<';
    return '-';
  };

  // ---- S1: Arm ----
  {
    char dc = dirChar(data.scraperArmJogDir);
    if (data.scraperArmPosClear != -1 && data.scraperArmPosScrape != -1) {
      snprintf(statusBuffer, sizeof(statusBuffer), "S1:Arm:%c SG:%d", dc, data.scraperArmSGResult);
    } else {
      snprintf(statusBuffer, sizeof(statusBuffer), "S1:Arm:%c SG:%d NC", dc, data.scraperArmSGResult);
    }
    u8g2.drawStr(0, 17, statusBuffer);

    const int trackL = 72, trackR = 124, trackY = 14;
    u8g2.drawHLine(trackL, trackY, trackR - trackL);

    if (data.scraperArmPosClear != -1 && data.scraperArmPosScrape != -1 && data.scraperArmPosScrape != data.scraperArmPosClear) {
      u8g2.drawVLine(trackL, trackY - 2, 5);
      u8g2.drawVLine(trackR, trackY - 2, 5);
      int dotX = map((int)data.scraperArmPosition, data.scraperArmPosClear, data.scraperArmPosScrape, trackL, trackR);
      dotX = constrain(dotX, trackL, trackR);
      u8g2.drawDisc(dotX, trackY, 2);
    } else {
      u8g2.drawDisc((trackL + trackR) / 2, trackY, 2);
    }
  }

  // ---- S2: Rotation ----
  {
    char dc = dirChar(data.dishRotationJogDir);
    snprintf(statusBuffer, sizeof(statusBuffer), "S2:Rot:%c SG:%d", dc, data.dishRotationSGResult);
    u8g2.drawStr(0, 26, statusBuffer);
  }

  // ---- S3: Z Motor ----
  {
    char dc = dirChar(data.dishLiftJogDir);
    snprintf(statusBuffer, sizeof(statusBuffer), "S3:Z:%c SG:%d", dc, data.dishLiftSGResult);
    u8g2.drawStr(0, 35, statusBuffer);

    bool botSet = data.dishLiftPosHomeSet;
    bool topSet = data.dishLiftPosTiltSet;
    float minLim = data.dishLiftPosHome;
    float maxLim = data.dishLiftPosTilt;

    const int trackL = 72, trackR = 124, trackY = 32;
    u8g2.drawHLine(trackL, trackY, trackR - trackL);

    if (botSet && topSet && maxLim > minLim) {
      float range = maxLim - minLim;
      if (range > 0.01f) {
        u8g2.drawVLine(trackL, trackY - 2, 5);
        u8g2.drawVLine(trackR, trackY - 2, 5);

        float cPos = data.dishLiftPos;
        if (cPos < minLim) cPos = minLim;
        if (cPos > maxLim) cPos = maxLim;
        int dotX = trackL + (int)(((cPos - minLim) / range) * (trackR - trackL));
        u8g2.drawDisc(dotX, trackY, 2);
      }
    } else {
      u8g2.drawDisc((trackL + trackR) / 2, trackY, 2);
    }
  }

  // ---- S4: Menu ----
  {
    if (!data.s4InSubMenu) {
      snprintf(statusBuffer, sizeof(statusBuffer), "S4:>%s", s4Level0Names[data.s4Menu]);
    } else {
      const char* axisName = s4Level0Names[data.s4Menu];
      
      if (data.s4Menu == S4_SCRAPER) {
        int idx = data.s4SubMenu;
        const char* itemName = s4ArmSubNames[idx];
        if (idx == S4_SCRAPER_BACK) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Back", axisName);
        else if (idx == S4_SCRAPER_CLEAR) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>%s:%s", axisName, itemName, (data.scraperArmPosClear != -1) ? "OK" : "--");
        else if (idx == S4_SCRAPER_SCRAPE) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>%s:%s", axisName, itemName, (data.scraperArmPosScrape != -1) ? "OK" : "--");
        else if (idx == S4_SCRAPER_JOG_SPD) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Jog:%d%s", axisName, data.scraperArmJogSpeed, data.s4InSpeedEdit ? "*" : "");
        else if (idx == S4_SCRAPER_GO_SPD) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Go:%d%s", axisName, data.scraperArmGoSpeed, data.s4InSpeedEdit ? "*" : "");
        else if (idx == S4_SCRAPER_SG_TUNE) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>SG:%d%s R:%d", axisName, data.scraperArmSGThreshold, data.s4InSpeedEdit ? "*" : "", data.scraperArmSGResult);
      } else if (data.s4Menu == S4_ROTATION) {
        int idx = data.s4SubMenu;
        const char* itemName = s4RotSubNames[idx];
        if (idx == S4_ROT_BACK) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Back", axisName);
        else if (idx == S4_ROT_JOG_SPD) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Jog:%d%s", axisName, data.dishRotationJogSpeed, data.s4InSpeedEdit ? "*" : "");
        else if (idx == S4_ROT_GO_SPD) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Go:%d%s", axisName, data.dishRotationGoSpeed, data.s4InSpeedEdit ? "*" : "");
        else if (idx == S4_ROT_NUM_ROTATIONS) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Rot:%d%s", axisName, data.dishRotationNumRotations, data.s4InSpeedEdit ? "*" : "");
        else if (idx == S4_ROT_SG_TUNE) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>SG:%d%s R:%d", axisName, data.dishRotationSGThreshold, data.s4InSpeedEdit ? "*" : "", data.dishRotationSGResult);
      } else if (data.s4Menu == S4_LIFT) {
        int idx = data.s4SubMenu;
        const char* itemName = s4LiftSubNames[idx];
        if (idx == S4_LIFT_BACK) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Back", axisName);
        else if (idx == S4_LIFT_HOME) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>%s:%s", axisName, itemName, data.dishLiftPosHomeSet ? "OK" : "--");
        else if (idx == S4_LIFT_TILT) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>%s:%s", axisName, itemName, data.dishLiftPosTiltSet ? "OK" : "--");
        else if (idx == S4_LIFT_JOG_SPD) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Jog:%d%s", axisName, data.dishLiftJogSpeed, data.s4InSpeedEdit ? "*" : "");
        else if (idx == S4_LIFT_GO_SPD) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Go:%d%s", axisName, data.dishLiftGoSpeed, data.s4InSpeedEdit ? "*" : "");
        else if (idx == S4_LIFT_NUM_MIX) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Mix:%d%s", axisName, data.dishLiftNumMix, data.s4InSpeedEdit ? "*" : "");
        else if (idx == S4_LIFT_SG_TUNE) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>SG:%d%s R:%d", axisName, data.dishLiftSGThreshold, data.s4InSpeedEdit ? "*" : "", data.dishLiftSGResult);
      }
    }
    u8g2.drawStr(0, 44, statusBuffer);
  }
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