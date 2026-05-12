#include "controller.h"
#include "messaging.h"
#include "tasks/ServoNode.h"
#include "tasks/ActuatorNode.h"
#include "tasks/MotorNode.h"
#include "drivers/EncoderDriver.h"
#include "tasks/LCD_task.h"
#include "tasks/encoder_task.h"

// Global Node instances (extern in controller.cpp)
ServoNode g_servoNode;
ActuatorNode g_actuatorNode;
MotorNode g_motorNode;

void setup() {
  // Begin USB serial for debugging/monitoring
  Serial.begin(115200);

  // Initialize System State from NVS
  initSystemState();

  // Start I2C Line (Used by encoder)
  Wire.begin(21, 22);

  // Inits
  encoderInit();
  LCDInit();

  // Task Update Intervals
  static int lcd_interval = TASK_REFRESH_LCD;

  // Create Tasks
  // Priorities Rebalanced: Higher number = Higher Priority
  xTaskCreate(encoderTask, "EncoderTask", 4096, NULL, 3, NULL);
  xTaskCreate(controller_task, "Controller", 4096, NULL, 3, NULL);

  // Start Active Motion Nodes (they create their own tasks internally)
  // Motor Node - priority 2
  if (!g_motorNode.start("MotorNode", 4096, 2)) {
    ESP_LOGE("MAIN", "Failed to start MotorNode");
  }
  
  // Actuator Node - priority 2
  if (!g_actuatorNode.start("ActuatorNode", 4096, 2)) {
    ESP_LOGE("MAIN", "Failed to start ActuatorNode");
  }
  
  // Servo Node - priority 2
  if (!g_servoNode.start("ServoNode", 4096, 2)) {
    ESP_LOGE("MAIN", "Failed to start ServoNode");
  }

  xTaskCreate(LCD_task, "LCD", 8192, &lcd_interval, 2, NULL);
}

void loop() {
  // Delete the default Arduino loop task to reclaim its memory stack
  vTaskDelete(NULL);
}