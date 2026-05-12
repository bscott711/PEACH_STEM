#include "drivers/EncoderDriver.h"
#include "controller.h" // Gives access to encoderStateMutex
#include <Adafruit_seesaw.h>
#include <esp_log.h>
#include <Preferences.h>

extern Preferences preferences;

static Adafruit_seesaw ss(&Wire);

EncoderState g_encoderState;

// State tracking for debouncing and edge detection
static bool lastBtnLevel[4];
static TickType_t lastDebounceTime[4] = {0};
static TickType_t btnPressTime[4] = {0};

const TickType_t DEBOUNCE_DELAY_MS = 50;

void init_encoder() {
  if (!ss.begin(ENCODER_I2C_ADDR)) {
    ESP_LOGE("ENCODER", "Failed to detect I2C encoder at 0x%02X",
             ENCODER_I2C_ADDR);
    vTaskDelay(pdMS_TO_TICKS(2000));
    while (true) {
      vTaskDelay(portMAX_DELAY);
    }
  }

  uint32_t mask = 0;
  for (int i = 0; i < 4; i++) {
    ss.pinMode(SEESAW_BUTTON_PINS[i], INPUT_PULLUP);
    mask |= (1UL << SEESAW_BUTTON_PINS[i]);

    bool state = ss.digitalRead(SEESAW_BUTTON_PINS[i]);
    lastBtnLevel[i] = state;
  }

  ss.setGPIOInterrupts(mask, 1);

  for (int i = 0; i < 4; i++) {
    g_encoderState.position[i] = ss.getEncoderPosition(i);
    ss.enableEncoderInterrupt(i);
    g_encoderState.buttonPressed[i] = false;
    g_encoderState.buttonDoublePressed[i] = false;
    g_encoderState.buttonLongPressed[i] = false;
  }

  // Override encoder 0 to match saved servo start position
  int savedStart = preferences.getInt("srvStart", -1);

  if (savedStart != -1) {
    if (xSemaphoreTake(encoderStateMutex, portMAX_DELAY) == pdTRUE) {
      g_encoderState.position[0] = savedStart;
      xSemaphoreGive(encoderStateMutex);
    }
  }
}

void EncoderDriver_Service() {
  TickType_t now = xTaskGetTickCount();

  for (int i = 0; i < 4; i++) {

    // 1. Process Buttons with Lockout Debouncing + Confirmation Read
    bool reading = ss.digitalRead(SEESAW_BUTTON_PINS[i]);
    bool setPressed = false;
    bool setDoublePressed = false;
    bool setLongPressed = false;

    if (reading != lastBtnLevel[i]) {
      // Double-read to filter single-sample I2C glitches
      bool confirm = ss.digitalRead(SEESAW_BUTTON_PINS[i]);
      if (confirm == reading &&
          (now - lastDebounceTime[i]) >= pdMS_TO_TICKS(DEBOUNCE_DELAY_MS)) {
        lastDebounceTime[i] = now;
        lastBtnLevel[i] = reading;

        if (reading == false) {
          // LOW = Pressed
          btnPressTime[i] = now;
        } else {
          // HIGH = Released
          TickType_t duration = (now - btnPressTime[i]) * portTICK_PERIOD_MS;

          if (duration >= 2500) {
            setDoublePressed = true;
            ESP_LOGI("ENCODER", "Button %d VERY LONG pressed (clearing pos)", i);
          } else if (duration >= 800) {
            setLongPressed = true;
            ESP_LOGI("ENCODER", "Button %d LONG pressed", i);
          } else {
            setPressed = true;
            ESP_LOGI("ENCODER", "Button %d pressed", i);
          }
        }
      }
    }

    // 2. Process Encoder Rotation
    // Adafruit lib returns 0 if I2C fails or no delta occurred.
    int32_t d = ss.getEncoderDelta(i);

    // 3. Update Global State (Mutex Protected)
    bool stateChanged = setPressed || setDoublePressed || setLongPressed || (reading != lastBtnLevel[i]) || (d != 0);
    if (stateChanged) {
      if (xSemaphoreTake(encoderStateMutex, portMAX_DELAY) == pdTRUE) {
        if (setPressed) {
          g_encoderState.buttonPressed[i] = true;
        }
        if (setDoublePressed) {
          g_encoderState.buttonDoublePressed[i] = true;
        }
        if (setLongPressed) {
          g_encoderState.buttonLongPressed[i] = true;
        }
        if (d != 0) {
          g_encoderState.position[i] += d;

          // Rate limit the serial output so I2C doesn't get starved
          static TickType_t lastLogTime[4] = {0};
          if ((now - lastLogTime[i]) >= pdMS_TO_TICKS(250)) {
            ESP_LOGI("ENCODER", "Enc %d: %ld", i,
                     (long)g_encoderState.position[i]);
            lastLogTime[i] = now;
          }
        }
        // Update held state dynamically
        g_encoderState.buttonHeld[i] = (lastBtnLevel[i] == false);
        g_encoderState.buttonPressTime[i] = btnPressTime[i];

        xSemaphoreGive(encoderStateMutex);
      }
    }
  }
}