#pragma once
#include <Arduino.h>
//#include <Adafruit_seesaw.h>
#include <Wire.h>

#define ENCODER_I2C_ADDR  0x49   // your working address

typedef struct
{
    int32_t position[4];
    bool    buttonPressed[4]; // latched "press" event
    bool    buttonLongPressed[4]; // latched "long press" event
} EncoderState;

extern EncoderState g_encoderState;

void init_encoder();
void EncoderDriver_Service();   // call after INT fires