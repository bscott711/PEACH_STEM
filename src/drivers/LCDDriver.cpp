#include "LCDDriver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <cstdio>  // for snprintf
#include <cstring> // for strncpy

// Instantiation for our LCD screen
U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, LCD_CS, LCD_DC, LCD_RESET);

uint32_t lcdStartTime = 0;

// ============ Message Buffer Globals ============
char lcdActionMessage[LCD_MSG_LEN] = "";
volatile bool lcdMessagePending = false;
volatile uint32_t lcdMessageTimestamp = 0; // ← Fixed: volatile matches header

// ============ Mutex for Thread-Safe State Access ============
static SemaphoreHandle_t stateMutex = NULL;

void LCDInit() {
  lcdStartTime = millis();
  u8g2.begin();
  u8g2.setFont(u8g2_font_tiny5_tf);

  // Create mutex for safe systemState access
  stateMutex = xSemaphoreCreateMutex();
  if (stateMutex == NULL) {
    Serial.println("ERROR: Failed to create LCD state mutex");
  }
}

void LCD_setMessage(const char *msg) {
  strncpy(lcdActionMessage, msg, LCD_MSG_LEN - 1);
  lcdActionMessage[LCD_MSG_LEN - 1] = '\0';
  lcdMessageTimestamp = millis();
  lcdMessagePending = true;
}

uint32_t buttonPressTime[4] = {0, 0, 0, 0};

void LCD_notifyButtonPress(int index) {
  if (index >= 0 && index < 4) {
    buttonPressTime[index] = millis();
  }
}

// ============ Drawing Helpers ============

void draw_displayTimer() {
  uint32_t t = millis() / 1000;
  uint32_t m = t / 60;
  uint32_t s = t % 60;
  char buf[32];
  // Shorter label saves precious horizontal space
  snprintf(buf, sizeof(buf), "RUN:%02u:%02u", m, s);
  u8g2.drawStr(0, 6, buf); // ← Changed from 78 to 0 (left-aligned)
}

void draw_buttonStatus() {
  // Draw 4 small indicators for encoder buttons 0-3 (top-RIGHT corner)
  for (int i = 0; i < 4; i++) {
    bool pressed = (millis() - buttonPressTime[i] < 200) || g_encoderState.buttonLongPressed[i];
    if (pressed) {
      u8g2.drawDisc(100 + (i * 6), 4, 2); // filled = pressed
    } else {
      u8g2.drawCircle(100 + (i * 6), 4, 2); // outline = unpressed
    }
  }
}

void draw_encoderStatus() {
  char buf[32];

  // Safely copy state values under mutex protection
  int servoTarget = 0, actuatorTarget = 0, motorTarget = 0, sgThreshold = 0;
  DeviceMode currentMode = IDLE;
  bool isHoming = false, sgDiagMode = false, triggerHoming = false;

  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    // Critical section: copy only what we need for display
    servoTarget = systemState.servoTargetPercent;
    actuatorTarget = systemState.actuatorTargetPercent;
    motorTarget = systemState.targetSpeed;
    sgThreshold = systemState.sgThreshold;
    currentMode = systemState.mode;
    isHoming = systemState.isHoming;
    sgDiagMode = systemState.sgDiagMode;
    triggerHoming = systemState.triggerHoming;
    xSemaphoreGive(stateMutex);
  }
  // If mutex fails, we draw with zeros — acceptable for display fallback

  // Encoder 0: Servo (0-100%)
  snprintf(buf, sizeof(buf), "S0:Srv:%03d%%", servoTarget);
  u8g2.drawStr(0, 16, buf);
  if (servoTarget > 50)
    u8g2.drawStr(72, 16, "------->");
  else if (servoTarget < 50)
    u8g2.drawStr(72, 16, "<-------");
  else
    u8g2.drawStr(72, 16, "--- O ---");

  // Encoder 1: Actuator (0-100%)
  snprintf(buf, sizeof(buf), "S1:Act:%03d%%", actuatorTarget);
  u8g2.drawStr(0, 24, buf);
  
  // Single Triangle (filled UP if > 50, filled DOWN if < 50, CIRCLE if == 50)
  if (actuatorTarget > 50) {
    u8g2.drawTriangle(75, 18, 71, 24, 79, 24); // UP
  } else if (actuatorTarget < 50) {
    u8g2.drawTriangle(75, 24, 71, 18, 79, 18); // DOWN
  } else {
    u8g2.drawDisc(75, 21, 2); // Center dot
  }

  // Encoder 2: Motor (display in steps -15 to +15)
  int step = motorTarget / 333;
  snprintf(buf, sizeof(buf), "S2:Mot:%+03d", step);
  u8g2.drawStr(0, 32, buf);
  
  // Custom Motor Speed Bar (Center at x=85)
  u8g2.drawFrame(51, 25, 69, 9); // Frame from x=51 to 119
  u8g2.drawLine(85, 25, 85, 33); // Center zero mark
  
  int clicks = abs(step);
  if (clicks > 15) clicks = 15;
  
  if (motorTarget > 0) {
    u8g2.drawBox(87, 27, clicks * 2, 5);
  } else if (motorTarget < 0) {
    u8g2.drawBox(83 - (clicks * 2), 27, clicks * 2, 5);
  }

  // Show [P] inside the empty right-side of the bar if paused
  if (motorTarget == 0 && currentMode != IDLE) {
    u8g2.drawStr(88, 32, "[P]");
  }

  // Encoder 3: StallGuard threshold + status flags
  snprintf(buf, sizeof(buf), "S3:SG:%03d", sgThreshold);
  u8g2.drawStr(0, 40, buf);
  if (isHoming)
    u8g2.drawStr(72, 40, "[H]");
  else if (sgDiagMode)
    u8g2.drawStr(72, 40, "[D]");
  else if (triggerHoming)
    u8g2.drawStr(72, 40, "[*]");
}

void draw_actionMessage() {
  // Show message for 2.5 seconds after being set
  if (lcdMessagePending && (millis() - lcdMessageTimestamp < 2500)) {
    char buf[LCD_MSG_LEN + 4];
    snprintf(buf, sizeof(buf), "> %s", lcdActionMessage);
    u8g2.drawStr(0, 60, buf);
  } else {
    lcdMessagePending = false; // auto-expire
  }
}

// ============ Main Draw Function ============

void draw_menu() {
  u8g2.clearBuffer();

  // Header row
  draw_displayTimer(); // Runtime top-right
  draw_buttonStatus(); // Button indicators next to it

  u8g2.drawHLine(0, 10, 128); // separator

  // Main content: 4 encoder status lines
  draw_encoderStatus();

  u8g2.drawHLine(0, 46, 128); // separator before footer

  // Footer: transient action message
  draw_actionMessage();

  u8g2.sendBuffer(); // Push to OLED
}