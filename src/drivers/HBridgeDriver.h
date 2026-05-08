#pragma once
#include <Arduino.h>
#include <controller.h>

#define HB_ENA 26
#define HB_IN1 25
#define HB_IN2 33

void HBridge_Init();
void HBridge_Set(ActuatorDirection dir);

/***********************************************
             Tasks Timing Intervals
***********************************************/
#define TASK_UPDATE_HBRIDGE 100