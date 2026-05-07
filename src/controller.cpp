#include "controller.h"
#include "drivers/EncoderDriver.h"
#include "drivers/LCDDriver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdio>

// Instantiate Global System State
SystemState systemState = {.mode = IDLE,
                           .busy = false,
                           .servoAdjustMode = true,
                           .servoPercent = 0,
                           .servoTargetPercent = 0,
                           .actuatorDir = ACT_STOP,
                           .actualSpeed = 0,
                           .targetSpeed = 0,
                           .actuatorTargetPercent = 0,
                           .triggerHoming = false,
                           .isHoming = false,
                           .sgDiagMode = false,
                           .sgThreshold = 16,
                           .currentPosition = 0.0,
                           .isHomed = false,
                           .motorEncoderLimit = 16,
                           .collisionDetected = false,
                           .collisionTimestamp = 0};

// Instantiate Hardware Objects
// static Arm arm;
// static Slide slide;

void controller_task(void *pvParameters) {
  // Record initial variables
  int32_t d0 = g_encoderState.position[0]; // Servo
  int32_t d1 = g_encoderState.position[1]; // Actuator

  while (1) {
    // ********************************************************************
    //                     ENCODER 0 - SERVO CONTROL
    // ********************************************************************
    // Encoder 0 - Manual Servo Control
    if (d0 != g_encoderState.position[0]) {

      // Constrain the hardware position so the physical knob doesn't "wind up"
      if (g_encoderState.position[0] > 100)
        g_encoderState.position[0] = 100;
      if (g_encoderState.position[0] < 0)
        g_encoderState.position[0] = 0;

      // Update d0
      d0 = g_encoderState.position[0];

      // Update servo percent
      systemState.servoTargetPercent = d0;

      // Debugging + UI Feedback
      printf("Servo Percent: %d\n", d0);
      LCD_setMessage("Servo Adjusted");
    }

    // Push Button 0 - Servo Toggle
    if (g_encoderState.buttonPressed[0]) {
      g_encoderState.buttonPressed[0] = false;
      LCD_notifyButtonPress(0);

      if (systemState.servoTargetPercent == 0) {
        // Go to center if at start
        g_encoderState.position[0] = 50;
        LCD_setMessage("Servo: Center");
      } else if (systemState.servoTargetPercent == 50) {
        // Or go to start if at center
        g_encoderState.position[0] = 0;
        LCD_setMessage("Servo: Start");
      } else if (systemState.servoTargetPercent < 50) {
        // Otherwise Round down
        g_encoderState.position[0] = 0;
        LCD_setMessage("Servo: Start");
      } else {
        // Or Round up
        g_encoderState.position[0] = 50;
        LCD_setMessage("Servo: Center");
      }

      // Instantly sync the target and d0 so the servo moves immediately
      d0 = g_encoderState.position[0];
      systemState.servoTargetPercent = d0;

      printf("Servo Button Pressed. New Pos: %d\n", d0);
    }

    // ********************************************************************
    //                   ENCODER 1 - ACTUATOR CONTROL
    // ********************************************************************
    // Linear Actuator Manual Control
    if (d1 != g_encoderState.position[1]) {
      // Constrain this from 0 to 10 (set to move in 10% intervals)
      if (g_encoderState.position[1] >= 10) {
        g_encoderState.position[1] = 10;
      }
      if (g_encoderState.position[1] <= 0) {
        g_encoderState.position[1] = 0;
      }
      // Update new percentage
      d1 = g_encoderState.position[1];

      // Move Actuator
      systemState.actuatorTargetPercent = d1 * 10;

      // Optional: Only update message if it changes significantly to avoid
      // flicker or just let the user see "Actuator Moving"
      // LCD_setMessage("Actuator Moving");
    }

    // Push Button 1 - Actuator Cycle
    if (g_encoderState.buttonPressed[1]) {
      // Service Flag
      g_encoderState.buttonPressed[1] = false;
      LCD_notifyButtonPress(1);

      // Get current percent
      int percent = systemState.actuatorTargetPercent;

      if (percent == 0) {
        // Go to center if at start
        systemState.actuatorTargetPercent = 50;
        LCD_setMessage("Actuator: 50%");
      } else if (percent == 50) {
        // Go to end if at center
        systemState.actuatorTargetPercent = 100;
        LCD_setMessage("Actuator: 100%");
      } else if (percent == 100) {
        // Go to start if at end
        systemState.actuatorTargetPercent = 0;
        LCD_setMessage("Actuator: 0%");
      } else {
        // Otherwise go back to 0
        systemState.actuatorTargetPercent = 0;
        LCD_setMessage("Actuator: Reset");
      }

      printf("Actuator Button Pressed. New Target: %d%%\n",
             systemState.actuatorTargetPercent);
    }

    // ********************************************************************
    //                   ENCODER 2 - MOTOR CONTROL
    // ********************************************************************
    // Static variables persist across loop iterations to remember state
    static bool motorPaused = false;
    static int32_t last_d2 = g_encoderState.position[2];

    // Constrain the encoder so it doesn't wind up
    if (g_encoderState.position[2] > systemState.motorEncoderLimit) {
      g_encoderState.position[2] = systemState.motorEncoderLimit;
    }
    if (g_encoderState.position[2] < -systemState.motorEncoderLimit) {
      g_encoderState.position[2] = -systemState.motorEncoderLimit;
    }

    int32_t d2 = g_encoderState.position[2];

    // If the encoder knob is moved, automatically unpause
    if (motorPaused && (d2 != last_d2)) {
      motorPaused = false;
      LCD_setMessage("Motor: Unpaused");
      printf("Motor Unpaused via Encoder Turn\n");
    }
    last_d2 = d2; // Track position for the next loop

    // Push button 2 - Toggle Start/Stop Motor
    if (g_encoderState.buttonPressed[2]) {
      // Service Flag
      g_encoderState.buttonPressed[2] = false;
      LCD_notifyButtonPress(2);

      // Toggle the pause state
      motorPaused = !motorPaused;

      if (motorPaused) {
        LCD_setMessage("Motor: PAUSED");
        printf("Motor PAUSED\n");
      } else {
        LCD_setMessage("Motor: RUNNING");
        printf("Motor RUNNING\n");
      }

      // Reset position to 0
      g_encoderState.position[2] = 0;
      last_d2 = 0;
      d2 = 0;
    }

    // Apply speed based on pause state
    if (!motorPaused) {
      systemState.targetSpeed = d2 * 5000;
    } else {
      // Force stop when paused
      systemState.targetSpeed = 0;
    }

    // ********************************************************************
    //                   ENCODER 3 - AUTONOMOUS CONTROL
    // ********************************************************************

    // Push button 3 - Sensorless Homing (LONG PRESS)
    if (g_encoderState.buttonLongPressed[3]) {
      g_encoderState.buttonLongPressed[3] = false;
      printf("\nTriggering Hardware Sensorless Homing...\n");
      LCD_setMessage("Homing Started...");
      systemState.triggerHoming = true;
    }

    // ENCODER 3 (Live StallGuard Tuning)
    static int32_t last_d3 = g_encoderState.position[3];
    int32_t delta3 = g_encoderState.position[3] - last_d3;
    last_d3 = g_encoderState.position[3];

    if (delta3 != 0) {
      systemState.sgThreshold += delta3;

      // Constrain between 0 and 255 to prevent overflow
      systemState.sgThreshold = constrain(systemState.sgThreshold, 0, 255);

      printf("Enc 3 (SG Threshold): %d\n", systemState.sgThreshold);
      // Optional: Show SG value briefly on LCD if tuning actively
      // char buf[16];
      // snprintf(buf, sizeof(buf), "SG Tune: %d", systemState.sgThreshold);
      // LCD_setMessage(buf);
    }

    // Push button 3 - Autonomous Code (SHORT PRESS)
    if (g_encoderState.buttonPressed[3]) {
      // Service Flag
      g_encoderState.buttonPressed[3] = false;
      LCD_notifyButtonPress(3);

      LCD_setMessage("Auto Sequence Start");
      printf("Starting Autonomous Sequence...\n");

      // Start at 0
      systemState.targetSpeed = 0;
      systemState.actuatorTargetPercent = 0;
      systemState.servoTargetPercent = 0;
      vTaskDelay(pdMS_TO_TICKS(2500));

      // gets out of the way to allow for person to add sample
      LCD_setMessage("Auto: Moving Away");
      systemState.targetSpeed = 120000;
      vTaskDelay(pdMS_TO_TICKS(15000));
      systemState.targetSpeed = 0;
      vTaskDelay(pdMS_TO_TICKS(500));

      // press button again to start
      LCD_setMessage("Press Btn to Continue");
      while (g_encoderState.buttonPressed[3] == false) {
        // Note: Printing inside a tight loop can flood serial buffer.
        // Ideally, print only once or every few seconds.
        // printf("waiting for button");
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
      g_encoderState.buttonPressed[3] = false;
      LCD_setMessage("Auto: Resuming");

      systemState.servoTargetPercent = 0; // move arm out
      vTaskDelay(pdMS_TO_TICKS(2500));

      // move halfway back down into tube
      LCD_setMessage("Auto: Down to Tube");
      systemState.targetSpeed = -120000;
      vTaskDelay(pdMS_TO_TICKS(15000));
      systemState.targetSpeed = 0;
      vTaskDelay(pdMS_TO_TICKS(500));

      // hopefully will do aspiration motion
      LCD_setMessage("Auto: Aspiration");
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
      LCD_setMessage("Auto: Pickup Cells");
      systemState.actuatorTargetPercent = 100;
      vTaskDelay(pdMS_TO_TICKS(500));
      systemState.actuatorTargetPercent = 0;
      vTaskDelay(pdMS_TO_TICKS(1000)); // needs time before starting to move

      // move back up away from tube
      LCD_setMessage("Auto: Moving Up");
      systemState.targetSpeed = 120000;
      vTaskDelay(pdMS_TO_TICKS(15000));
      systemState.targetSpeed = 0;
      vTaskDelay(pdMS_TO_TICKS(500));

      systemState.servoTargetPercent = 30; // get out of the way
      vTaskDelay(pdMS_TO_TICKS(1000));

      // into microscope
      LCD_setMessage("Auto: To Microscope");
      systemState.targetSpeed = -120000;
      vTaskDelay(pdMS_TO_TICKS(15000));
      systemState.targetSpeed = 0;
      vTaskDelay(pdMS_TO_TICKS(500));

      LCD_setMessage("Auto: Dropping Cells");
      systemState.actuatorTargetPercent = 100;
      vTaskDelay(pdMS_TO_TICKS(1000));

      // out of microscope
      LCD_setMessage("Auto: Retreating");
      systemState.targetSpeed = 120000;
      vTaskDelay(pdMS_TO_TICKS(15000));
      systemState.targetSpeed = 0;
      vTaskDelay(pdMS_TO_TICKS(500));

      systemState.actuatorTargetPercent = 0; // back to resting
      LCD_setMessage("Auto: Complete");
      vTaskDelay(pdMS_TO_TICKS(500));
    }

    // RTOS Loop
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

double motorDistanceCalculator(double speed, int timeInMS) {
  // 50000Speed at 5000MS = AVG(3.43mm)

  double position = 0.000001372 * speed * timeInMS;

  return position;
}

double motorSpeedCalculator(double position, int timeInMS) {
  double speed = (position / timeInMS) / 0.000001372;

  return speed;
}