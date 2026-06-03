#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "HardwareConfig.h"

static const uint8_t SEESAW_BUTTON_PINS[4] = {SEESAW_BTN_ENC0, SEESAW_BTN_ENC1,
                                              SEESAW_BTN_ENC2, SEESAW_BTN_ENC3};

typedef struct {
  int32_t position[4];
  bool buttonPressed[4];
  bool buttonDoublePressed[4];
  bool buttonLongPressed[4];
  bool buttonHeld[4];
  TickType_t buttonPressTime[4];
} EncoderState;

extern EncoderState g_encoderState;

void init_encoder();
void EncoderDriver_Service();