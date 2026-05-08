#include "drivers/EncoderDriver.h"
#include "controller.h"
#include <Adafruit_seesaw.h>
#include <Wire.h>

// Seesaw Object bound to I2C. All encoder communication happens through this.
static Adafruit_seesaw ss(&Wire);

// Global shared struct
EncoderState g_encoderState;

// Stores previous button level and press times for edge/hold detection
static bool lastBtnLevel[4];
static uint32_t btnPressTime[4];

// Initialize the 4-encoder Seesaw board
void init_encoder() {
  if (!ss.begin(ENCODER_I2C_ADDR)) {
    Serial.printf("Failed to detect encoder.");
    while (true) {
      vTaskDelay(portMAX_DELAY);
    }
  }

  ss.pinMode(12, INPUT_PULLUP);
  ss.pinMode(14, INPUT_PULLUP);
  ss.pinMode(17, INPUT_PULLUP);
  ss.pinMode(9, INPUT_PULLUP);

  lastBtnLevel[0] = ss.digitalRead(12);
  lastBtnLevel[1] = ss.digitalRead(14);
  lastBtnLevel[2] = ss.digitalRead(17);
  lastBtnLevel[3] = ss.digitalRead(9);

  uint32_t mask = (1UL << 12) | (1UL << 14) | (1UL << 17) | (1UL << 9);
  ss.setGPIOInterrupts(mask, 1);

  for (int i = 0; i < 4; i++) {
    g_encoderState.position[i] = ss.getEncoderPosition(i);
    ss.enableEncoderInterrupt(i);
    g_encoderState.buttonPressed[i] = false;
    g_encoderState.buttonLongPressed[i] = false;
    btnPressTime[i] = 0;
  }
}

void EncoderDriver_Service() {
  int pins[] = {12, 14, 17, 9};

  for (int i = 0; i < 4; i++) {
    // 1. Process Buttons (Short vs Long Press)
    bool b = ss.digitalRead(pins[i]);

    if (lastBtnLevel[i] == true && b == false) {
      // Falling Edge (Button Pressed Down)
      btnPressTime[i] = millis();
    } else if (lastBtnLevel[i] == false && b == true) {
      // Rising Edge (Button Released)
      if (millis() - btnPressTime[i] >= 1000) {
        g_encoderState.buttonLongPressed[i] = true;
        Serial.printf("Button %d LONG pressed\n", i);
      } else {
        g_encoderState.buttonPressed[i] = true;
        Serial.printf("Button %d pressed\n", i);
      }
    }
    lastBtnLevel[i] = b;

    // 2. Process Encoder Rotation
    int32_t d = ss.getEncoderDelta(i);
    if (d != 0) {
      g_encoderState.position[i] += d;
      Serial.printf("Enc %d: %d\n", i, (int)g_encoderState.position[i]);
    }
  }
}