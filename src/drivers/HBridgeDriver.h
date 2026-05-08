#pragma once
#include "controller.h" // Fixed include syntax
#include <Arduino.h>

#define HB_ENA 26
#define HB_IN1 25
#define HB_IN2 33

void HBridge_Init();
void HBridge_Set(ActuatorDirection dir);