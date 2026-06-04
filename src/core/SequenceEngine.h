#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define SEQ_TARGET_BUFFER 200

void autonomous_task(void *pvParameters);
void motor_goto_task(void *pvParameters);
