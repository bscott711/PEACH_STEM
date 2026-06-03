#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void autonomous_task(void *pvParameters);
void motor_goto_task(void *pvParameters);
