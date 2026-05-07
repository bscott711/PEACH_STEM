#include "controller.h"
#include "drivers/EncoderDriver.h"
#include "drivers/motorDriver.h"
#include "tasks/LCD_task.h"
#include "tasks/actuator_task.h"
#include "tasks/encoder_task.h"
#include "tasks/motor_task.h"
#include "tasks/servo_task.h"

void setup() {
  // Begin USB serial for debugging/monitoring
  Serial.begin(115200);

  // Start I2C Line (Used by encoder)
  Wire.begin(21, 22);

  // Inits
  encoderInit();
  motorInit();
  LCDInit();

  // Task Update Intervals
  int task_update_motor = TASK_UPDATE_MOTOR;
  int servo_interval = TASK_UPDATE_SERVO;
  int lcd_interval = TASK_REFRESH_LCD;

  // Create Tasks
  xTaskCreate(motor_task, "Update Motor", 4096, &task_update_motor, 1, NULL);
  xTaskCreate(encoderTask, "EncoderTask", 4096, NULL, 2, NULL);
  xTaskCreate(controller_task, "Controller", 4096, NULL, 2, NULL);
  xTaskCreate(servo_task, "Servo", 8192, &servo_interval, 2, NULL);
  xTaskCreate(actuator_task, "Actuator", 4096, NULL, 2, NULL);
  xTaskCreate(LCD_task, "LCD", 4096, &lcd_interval, 2, NULL);
}

void loop() {
  // Main loop never used
  // Delay prevents watchdog trigger
  vTaskDelay(pdMS_TO_TICKS(1000));
}