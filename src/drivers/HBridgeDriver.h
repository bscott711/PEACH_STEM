#pragma once
#include "controller.h" // Fixed include syntax
#include <Arduino.h>

#define HB_ENA 26
#define HB_IN1 25
#define HB_IN2 33

#define HB_PWM_CH 1
#define HB_PWM_FREQ 5000
#define HB_PWM_RES 8

void HBridge_Init();
void HBridge_Set(ActuatorDirection dir, uint8_t pwm_val = 255);