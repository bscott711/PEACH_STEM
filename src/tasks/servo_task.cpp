#include "servo_task.h"

extern SystemState systemState;

void servo_task(void *pvParameters)
{
    ServoDriver_Init();

    int interval = 10;  // 10ms smooth update
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1)
    {
        int current = systemState.servoPercent;
        int target  = systemState.servoTargetPercent;

        if (current < target)
            systemState.servoPercent++;
        else if (current > target)
            systemState.servoPercent--;

        ServoDriver_WritePercent(systemState.servoPercent);

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(interval));
    }
}