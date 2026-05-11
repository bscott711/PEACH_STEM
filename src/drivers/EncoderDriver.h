#pragma once
#include <Arduino.h>
#include <Wire.h>

#define ENCODER_I2C_ADDR 0x49

// Seesaw GPIO pins for encoder buttons (from Adafruit datasheet)
#define SEESAW_BTN_ENC0 12
#define SEESAW_BTN_ENC1 14
#define SEESAW_BTN_ENC2 17
#define SEESAW_BTN_ENC3 9

static const uint8_t SEESAW_BUTTON_PINS[4] = {SEESAW_BTN_ENC0, SEESAW_BTN_ENC1,
                                              SEESAW_BTN_ENC2, SEESAW_BTN_ENC3};

typedef struct {
  int32_t position[4];
  bool buttonPressed[4];
  bool buttonDoublePressed[4];
  bool buttonLongPressed[4];
} EncoderState;

extern EncoderState g_encoderState;

void init_encoder();
void EncoderDriver_Service();