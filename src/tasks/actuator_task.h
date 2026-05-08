#pragma once
#include "controller.h"
#include "drivers/HBridgeDriver.h"

#define TASK_UPDATE_ACTUATOR 10 // Added dedicated interval definition

void actuator_task(void *pvParameters);