#include "controller.h"
#include "drivers/EncoderDriver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


// Instantiate Global System State
SystemState systemState =
{
    .mode = IDLE,
    .busy = false,
    .servoAdjustMode = true,
    .servoPercent = 0,
    .servoTargetPercent = 0,
    .actuatorDir = ACT_STOP,
    .actualSpeed = 0,
    .targetSpeed = 0,
    .actuatorTargetPercent = 0
};

// Instantiate Hardware Objects
//static Arm arm;
//static Slide slide;

void controller_task(void *pvParameters)
{
    // Record initial variables
    int32_t d0 = g_encoderState.position[0];  // Servo
    int32_t d1 = g_encoderState.position[1];  // Actuator
    //double stepperPosition = 0;


    while(1)
    {
        // ********************************************************************
        //                     ENCODER 0 - SERVO CONTROL
        // ********************************************************************
        // Encoder 0 - Manual Servo Control
        if (d0 != g_encoderState.position[0])
        {
            // Update d0
            d0 = g_encoderState.position[0];
            // Update servo percent
            systemState.servoTargetPercent = constrain(d0, 0, 100);

            printf((const char*)g_encoderState.position[0]);    //cast may be messy
        }

        // Push Button 0 - Servo Toggle
        if( g_encoderState.buttonPressed[0] )
        {
            g_encoderState.buttonPressed[0] = false;

            if(systemState.servoTargetPercent == 0)
            {
                // Go to center if at start
                g_encoderState.position[0] = 50;
            }
            else if(systemState.servoTargetPercent == 50)
            {
                // Or go to start if at center
                g_encoderState.position[0] = 0;
            }
            else if( systemState.servoTargetPercent < 50 )
            {
                // Otherwise Round down
                g_encoderState.position[0] = 0;
            }
            else
            {
                // Or Round up
                g_encoderState.position[0] = 50;
            }
        }

        // ********************************************************************
        //                   ENCODER 1 - ACTUATOR CONTROL
        // ********************************************************************
        // Linear Actuator Manual Control
        if (d1 != g_encoderState.position[1])
        {
            // Constrain this from 0 to 10 (set to move in 10% intervals)
            if( g_encoderState.position[1] >= 10 )
            {
                 g_encoderState.position[1] = 10;
            }
            if( g_encoderState.position[1] <= 0 )
            {
                g_encoderState.position[1] = 0;
            }
            // Update new percentage 
            d1 = g_encoderState.position[1];

            // Move Actuator
            systemState.actuatorTargetPercent = d1 * 10;
        }

        // Push Button 1 - Actuator Cycle
        if (g_encoderState.buttonPressed[1])
        {
            // Service Flag
            g_encoderState.buttonPressed[1] = false;

            // Get current percent
            int percent = systemState.actuatorTargetPercent;

            if ( percent == 0)
            {
                // Go to center if at start
                systemState.actuatorTargetPercent = 50;
            }
            else if (percent == 50)
            {
                // Go to end if at center
                systemState.actuatorTargetPercent = 100;
            }
            else if (percent == 100)
            {
                // Go to start if at end
                systemState.actuatorTargetPercent = 0;
            }
            else
            {
                // Otherwise go back to 0
                systemState.actuatorTargetPercent = 0;
            }
        }


        // ********************************************************************
        //                   ENCODER 2 - MOTOR CONTROL
        // ********************************************************************
        // Encoder 2 - Stepper Motor Control
        bool motorTestRunning = false;

        if (!motorTestRunning)
        {
            int32_t d2 = g_encoderState.position[2];
        
            if (d2 != 0)
            {
                systemState.targetSpeed = d2 * 10000;
            }
            else
            {
                systemState.targetSpeed = 0;
            }
        }

        if (g_encoderState.buttonPressed[2])
        {
            g_encoderState.buttonPressed[2] = false;

            motorTestRunning = true;

            systemState.targetSpeed = -120000;

            vTaskDelay(pdMS_TO_TICKS(15000));

            systemState.targetSpeed = 0;

            motorTestRunning = false;
        }

        // Push button 2 - Start/Stop Motor
        //if (g_encoderState.buttonPressed[2])
        //{


            // Service Flag
            //g_encoderState.buttonPressed[2] = false;

            // Reset to 0
            //g_encoderState.position[2] = 0;
            //systemState.targetSpeed = 0;
        //}


        // ********************************************************************
        //                   ENCODER 3 - AUTONOMOUS CONTROL
        // ********************************************************************
        // Encoder 3 - Autonomous Code

        if (g_encoderState.buttonPressed[3])
        {

            // Service Flag
            g_encoderState.buttonPressed[3] = false;
            // maybe add while false loop to make sure we can emergency stop..?

            // Start at 0
            systemState.targetSpeed = 0;
            systemState.actuatorTargetPercent = 0;      //make sure we aren't pressing
            //stepperPosition = 0;
            systemState.servoTargetPercent = 0;    //adjusted from 50 for demo (start under pipette)
            vTaskDelay(pdMS_TO_TICKS(2500));


            //gets out of the way to allow for person to add sample
            systemState.targetSpeed = 120000;
            vTaskDelay(pdMS_TO_TICKS(15000));
            systemState.targetSpeed = 0;
            vTaskDelay(pdMS_TO_TICKS(500));

            // press button again to start
            while (g_encoderState.buttonPressed[3] == false)
            {
                printf("waiting for button");
                vTaskDelay(pdMS_TO_TICKS(1000));
                //updates every 5 seconds 
            }
            g_encoderState.buttonPressed[3] = false;


            systemState.servoTargetPercent = 0; // move arm out 
            vTaskDelay(pdMS_TO_TICKS(2500));

            //move halfway back down into tube
            systemState.targetSpeed = -120000;
            vTaskDelay(pdMS_TO_TICKS(15000));
            systemState.targetSpeed = 0;
            vTaskDelay(pdMS_TO_TICKS(500));
            

            // hopefully will do aspiration motion
            systemState.actuatorTargetPercent = 80;
            vTaskDelay(pdMS_TO_TICKS(750));
            systemState.actuatorTargetPercent = 0;
            vTaskDelay(pdMS_TO_TICKS(750));
            systemState.actuatorTargetPercent = 80;
            vTaskDelay(pdMS_TO_TICKS(750));
            systemState.actuatorTargetPercent = 0;
            vTaskDelay(pdMS_TO_TICKS(750));
            systemState.actuatorTargetPercent = 80;
            vTaskDelay(pdMS_TO_TICKS(750));
            systemState.actuatorTargetPercent = 0;
            vTaskDelay(pdMS_TO_TICKS(750));

            // pickup cells
            systemState.actuatorTargetPercent = 100;
            vTaskDelay(pdMS_TO_TICKS(500));
            systemState.actuatorTargetPercent = 0;
            vTaskDelay(pdMS_TO_TICKS(1000));    //needs time before starting to move

            // move back up away from tube
            systemState.targetSpeed = 120000;
            vTaskDelay(pdMS_TO_TICKS(15000));
            systemState.targetSpeed = 0;
            vTaskDelay(pdMS_TO_TICKS(500));

            systemState.servoTargetPercent = 30; //get out of the way
            vTaskDelay(pdMS_TO_TICKS(1000));

            //into microscope
            systemState.targetSpeed = -120000;
            vTaskDelay(pdMS_TO_TICKS(15000));
            systemState.targetSpeed = 0;
            vTaskDelay(pdMS_TO_TICKS(500));

            systemState.actuatorTargetPercent = 100;    // drop cells adjusted from 100 for demo
            vTaskDelay(pdMS_TO_TICKS(1000));


            //out of microscope
            systemState.targetSpeed = 120000;
            vTaskDelay(pdMS_TO_TICKS(15000));
            systemState.targetSpeed = 0;
            vTaskDelay(pdMS_TO_TICKS(500));

            systemState.actuatorTargetPercent = 0;  //back to resting
            vTaskDelay(pdMS_TO_TICKS(500));

        }


        // RTOS Loop
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

double motorDistanceCalculator(double speed, int timeInMS)
{
    //50000Speed at 5000MS = AVG(3.43mm)

    double position = 0.000001372 * speed * timeInMS;

    return position;
}

double motorSpeedCalculator(double position, int timeInMS)
{
    double speed = (position/timeInMS)/0.000001372;

    return speed;

}