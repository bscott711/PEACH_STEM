#include "motor_task.h"

static motorDriver motor;

void motor_task(void *parameter) {
  int interval = *(int *)parameter;
  TickType_t lastWakeTime = xTaskGetTickCount();

  int newSpeed = systemState.targetSpeed;

  while (1) {
    // 1. Check if homing was requested by the controller
    if (systemState.triggerHoming) {
      systemState.triggerHoming = false;
      systemState.isHoming = true;
      motor.homeSensorless();
      systemState.isHoming = false;
      systemState.targetSpeed = 0;
      newSpeed = 0;
    }

    // 2. Normal speed control
    if (newSpeed != systemState.targetSpeed) {
      newSpeed = systemState.targetSpeed;
      motor.setVelocity(newSpeed);
    }

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(interval));
  }
}

void motorInit() {
  // Initialize the DIAG pin to listen for the hardware trigger
  pinMode(DIAG_PIN, INPUT_PULLDOWN);

  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
  delay(200);
  motor.begin(Serial1, TMC2209::SERIAL_ADDRESS_0);
}