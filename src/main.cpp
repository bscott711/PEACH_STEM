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

  // Initialize System State from NVS
  initSystemState();

  // Start I2C Line (Used by encoder)
  Wire.begin(21, 22);

  // Inits
  encoderInit();
  motorInit();
  LCDInit();

  // Task Update Intervals
  static int task_update_motor = TASK_UPDATE_MOTOR;
  static int servo_interval = TASK_UPDATE_SERVO;
  static int lcd_interval = TASK_REFRESH_LCD;

  // Create Tasks
  // Priorities Rebalanced: Higher number = Higher Priority
  xTaskCreate(encoderTask, "EncoderTask", 4096, NULL, 3, NULL);
  xTaskCreate(controller_task, "Controller", 4096, NULL, 3, NULL);

  xTaskCreate(motor_task, "Update Motor", 4096, &task_update_motor, 2, NULL);
  xTaskCreate(actuator_task, "Actuator", 4096, NULL, 2, NULL);
  xTaskCreate(servo_task, "Servo", 4096, &servo_interval, 2, NULL);

  xTaskCreate(LCD_task, "LCD", 8192, &lcd_interval, 1, NULL);
}

void loop() {
  // Delete the default Arduino loop task to reclaim its memory stack
  vTaskDelete(NULL);
}