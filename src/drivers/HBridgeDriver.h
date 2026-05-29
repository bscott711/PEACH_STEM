#pragma once
#include "controller.h" // Fixed include syntax
#include <Arduino.h>

#define HB_IN1 13
#define HB_IN2 14

#define HB_PWM_CH_IN1 1
#define HB_PWM_CH_IN2 2
#define HB_PWM_FREQ 5000
#define HB_PWM_RES 8

void HBridge_Init();
void HBridge_Set(ActuatorDirection dir, uint8_t pwm_val = 255);