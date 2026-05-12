#pragma once
#include <Arduino.h>

// ---- Hardware Config ----
#define SERVO_PIN 13
#define SERVO_LEDC_CH 0
#define SERVO_PWM_HZ 50
#define SERVO_RES_BITS 16

#define SERVO_MIN_US 500  // 0%
#define SERVO_MAX_US 2500 // 100%

// ---- API ----
void ServoDriver_Init();
void ServoDriver_Disable();
void ServoDriver_WritePercent(float pct);