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

struct BranchSegment {
    float x0, y0;
    float x1_full, y1_full;
    float startP, endP;
    bool isLeaf;
    int peachIndex;
};
#define MAX_TREE_SEGMENTS 250
static BranchSegment treeSegments[MAX_TREE_SEGMENTS];
static int numTreeSegments = 0;
static int fallingPeachLeafIndex = -1;
static int currentLeaf = 0;

static float hash_fn(int a, int b) {
  float h = sinf(a * 12.9898f + b * 78.233f) * 43758.5453123f;
  return h - floorf(h);
}

static float clamp(float val, float min, float max) {
  if (val < min) return min;
  if (val > max) return max;
  return val;
}

static void precalculateTree(float x, float y, float len, float angle, int depth, int pathIndex) {
  if (numTreeSegments >= MAX_TREE_SEGMENTS) return;
  const int maxDepth = 4;
  
  float startP, endP;
  if (depth == 0) {
    startP = 0; endP = 20;
  } else {
    float depthSlice = 30.0f / maxDepth;
    startP = 20 + (depth - 1) * depthSlice;
    endP = startP + depthSlice;
  }

  float endX_full = x + len * sinf(angle);
  float endY_full = y - len * cosf(angle);

  BranchSegment& seg = treeSegments[numTreeSegments++];
  seg.x0 = x; seg.y0 = y;
  seg.x1_full = endX_full; seg.y1_full = endY_full;
  seg.startP = startP; seg.endP = endP;
  seg.isLeaf = false;
  seg.peachIndex = -1;

  if (depth < maxDepth) {
    float leftLenMod = 0.65f + hash_fn(pathIndex, 1) * 0.15f;
    float rightLenMod = 0.65f + hash_fn(pathIndex, 2) * 0.15f;
    float leftAngleMod = 0.45f + hash_fn(pathIndex, 3) * 0.4f;
    float rightAngleMod = 0.45f + hash_fn(pathIndex, 4) * 0.4f;

    precalculateTree(endX_full, endY_full, len * leftLenMod, angle - leftAngleMod, depth + 1, pathIndex * 2);
    precalculateTree(endX_full, endY_full, len * rightLenMod, angle + rightAngleMod, depth + 1, pathIndex * 2 + 1);

    if (hash_fn(pathIndex, 5) > 0.5f) { 
       precalculateTree(endX_full, endY_full, len * 0.5f, angle + (hash_fn(pathIndex, 6) * 0.2f - 0.1f), depth + 1, pathIndex * 2 + 2);
    }
  }

  if (depth >= 2 && hash_fn(pathIndex, 11) > 0.3f) {
    if (numTreeSegments < MAX_TREE_SEGMENTS) {
        BranchSegment& leafSeg = treeSegments[numTreeSegments++];
        leafSeg.x0 = endX_full; leafSeg.y0 = endY_full;
        leafSeg.startP = 50 + hash_fn(pathIndex, 7) * 15; 
        leafSeg.endP = leafSeg.startP + 15;
        leafSeg.isLeaf = true;
        leafSeg.peachIndex = -1;
    }
  }

  if (depth == maxDepth) {
    currentLeaf++;
    if (currentLeaf == 8 || currentLeaf == 25 || currentLeaf == 45) {
      if (numTreeSegments < MAX_TREE_SEGMENTS) {
          if (fallingPeachLeafIndex == -1 && abs((int)endX_full) > 12) fallingPeachLeafIndex = currentLeaf;
          BranchSegment& peachSeg = treeSegments[numTreeSegments++];
          peachSeg.x0 = endX_full; peachSeg.y0 = endY_full;
          peachSeg.startP = 80.0f; peachSeg.endP = 90.0f;
          peachSeg.isLeaf = false;
          peachSeg.peachIndex = currentLeaf;
      }
    }
  }
}

static void drawTree(int progress, float offsetX) {
    for (int i = 0; i < numTreeSegments; i++) {
        const BranchSegment& seg = treeSegments[i];
        if (seg.isLeaf) {
            float leafGrowth = clamp((progress - seg.startP) / (seg.endP - seg.startP), 0.0f, 1.0f);
            if (leafGrowth > 0) {
                int r = (int)(2.0f * leafGrowth);
                if (r > 0) u8g2.drawDisc((int)(seg.x0 + offsetX), (int)seg.y0, r);
            }
            continue;
        }
        if (seg.peachIndex != -1) {
            if (progress > 80) {
                float peachGrowth = clamp((progress - 80.0f) / 10.0f, 0.0f, 1.0f);
                float pY = seg.y0;
                if (progress > 90 && seg.peachIndex == fallingPeachLeafIndex) {
                    float fallProgress = clamp((progress - 90.0f) / 10.0f, 0.0f, 1.0f);
                    pY += (62.0f - seg.y0) * fallProgress * fallProgress;
                    if (pY > 60.0f) pY = 60.0f;
                }
                if (peachGrowth > 0) {
                    int radius = (int)(4.0f * peachGrowth);
                    if (radius > 0) {
                        int px = (int)(seg.x0 + offsetX);
                        u8g2.drawDisc(px, (int)pY, radius);
                        u8g2.setDrawColor(0);
                        u8g2.drawLine(px, (int)(pY - radius), px, (int)(pY + radius - 1));
                        u8g2.setDrawColor(1);
                    }
                }
            }
            continue;
        }
        float growth = clamp((progress - seg.startP) / (seg.endP - seg.startP), 0.0f, 1.0f);
        if (growth > 0) {
            float currX = seg.x0 + (seg.x1_full - seg.x0) * growth;
            float currY = seg.y0 + (seg.y1_full - seg.y0) * growth;
            u8g2.drawLine((int)(seg.x0 + offsetX), (int)seg.y0, (int)(currX + offsetX), (int)currY);
        }
    }
}

static void draw_splashScreen() {
  u8g2.clearBuffer();

  drawTree(100, 32.0f);

  // === Text (right of peach) ===
  u8g2.setFont(u8g2_font_helvB14_tr);
  u8g2.drawStr(50, 30, "PEACH");

  u8g2.setFont(u8g2_font_helvB18_tr);
  u8g2.drawStr(55, 54, "STEM");

  // Small version tag
  u8g2.setFont(u8g2_font_tiny5_tf);
  u8g2.drawStr(50, 62, "v1.0");

  u8g2.sendBuffer();
}

void draw_wifiStatus(const char* status, const char* ssid, int attempt, bool failed) {
  u8g2.clearBuffer();

  drawTree(100, 32.0f);

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

  drawTree(otaProgress, 64.0f);

  // UI Overlay - Top Left
  u8g2.setFont(u8g2_font_tiny5_tf);
  u8g2.drawStr(0, 10, "PEACH PIT UPDATING");

  u8g2.sendBuffer();
}

void LCDInit() {
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
      snprintf(statusBuffer, sizeof(statusBuffer), "S1:Arm:%c", dc);
    } else {
      snprintf(statusBuffer, sizeof(statusBuffer), "S1:Arm:%c NC", dc);
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
    snprintf(statusBuffer, sizeof(statusBuffer), "S2:Rot:%c", dc);
    u8g2.drawStr(0, 26, statusBuffer);
  }

  // ---- S3: Z Motor ----
  {
    char dc = dirChar(data.dishLiftJogDir);
    snprintf(statusBuffer, sizeof(statusBuffer), "S3:Z:%c", dc);
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
        else if (idx == S4_SCRAPER_SG_TUNE) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>SG:%d%s", axisName, data.scraperArmSGThreshold, data.s4InSpeedEdit ? "*" : "");
      } else if (data.s4Menu == S4_ROTATION) {
        int idx = data.s4SubMenu;
        const char* itemName = s4RotSubNames[idx];
        if (idx == S4_ROT_BACK) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Back", axisName);
        else if (idx == S4_ROT_JOG_SPD) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Jog:%d%s", axisName, data.dishRotationJogSpeed, data.s4InSpeedEdit ? "*" : "");
        else if (idx == S4_ROT_GO_SPD) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Go:%d%s", axisName, data.dishRotationGoSpeed, data.s4InSpeedEdit ? "*" : "");
        else if (idx == S4_ROT_NUM_ROTATIONS) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Rot:%d%s", axisName, data.dishRotationNumRotations, data.s4InSpeedEdit ? "*" : "");
        else if (idx == S4_ROT_SG_TUNE) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>SG:%d%s", axisName, data.dishRotationSGThreshold, data.s4InSpeedEdit ? "*" : "");
      } else if (data.s4Menu == S4_LIFT) {
        int idx = data.s4SubMenu;
        const char* itemName = s4LiftSubNames[idx];
        if (idx == S4_LIFT_BACK) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Back", axisName);
        else if (idx == S4_LIFT_HOME) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>%s:%s", axisName, itemName, data.dishLiftPosHomeSet ? "OK" : "--");
        else if (idx == S4_LIFT_TILT) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>%s:%s", axisName, itemName, data.dishLiftPosTiltSet ? "OK" : "--");
        else if (idx == S4_LIFT_JOG_SPD) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Jog:%d%s", axisName, data.dishLiftJogSpeed, data.s4InSpeedEdit ? "*" : "");
        else if (idx == S4_LIFT_GO_SPD) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Go:%d%s", axisName, data.dishLiftGoSpeed, data.s4InSpeedEdit ? "*" : "");
        else if (idx == S4_LIFT_NUM_MIX) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Mix:%d%s", axisName, data.dishLiftNumMix, data.s4InSpeedEdit ? "*" : "");
        else if (idx == S4_LIFT_SG_TUNE) snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>SG:%d%s", axisName, data.dishLiftSGThreshold, data.s4InSpeedEdit ? "*" : "");
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