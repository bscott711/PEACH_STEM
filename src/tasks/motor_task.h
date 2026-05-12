#pragma once
#include "MotorDriver.h"
#include "controller.h"
#include <freertos/event_groups.h>

void motor_task(void *parameter);
void motorInit();