#pragma once
#include "controller.h" // Fixed include syntax
#include <Arduino.h>
#include "HardwareConfig.h"

void HBridge_Init();
void HBridge_Set(ActuatorDirection dir, uint8_t pwm_val = 255);