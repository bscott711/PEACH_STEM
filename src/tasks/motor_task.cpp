#include "motor_task.h"

static motorDriver motor;

void motor_task(void *parameter)
{
    int interval = *(int*)parameter;
    TickType_t lastWakeTime = xTaskGetTickCount();
    //static int lastSpeed = 0;

    // Obtain initial target speed
    int newSpeed = systemState.targetSpeed;
    while(1)
    {
        // If change detected in global state
        if( newSpeed != systemState.targetSpeed )
        {
            // Update temp variable
            newSpeed = systemState.targetSpeed;
            // Call the motor driver to update its speed
            motor.setVelocity(newSpeed);
        }

        // Wait until next interval
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(interval));
    }
}

void motorInit()
{
    // Initialize the motor
    Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
    delay(200);
    motor.begin(Serial1, TMC2209::SERIAL_ADDRESS_0);
}