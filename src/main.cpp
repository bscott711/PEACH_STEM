#include "controller.h"
#include "drivers/EncoderDriver.h"
#include "messaging.h"
#include "tasks/ActuatorNode.h"
#include "tasks/LCD_task.h"
#include "tasks/MotorNode.h"
#include "tasks/ServoNode.h"
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

  // --- NEW: THE PRIORITY SHIELD ---
  // Elevate setup() to Priority 5 (higher than our tasks) so it cannot be
  // preempted before it finishes linking the global queues.
  vTaskPrioritySet(NULL, 5);

  // 1. Start Active Motion Nodes
  if (!g_motorNode.start("MotorNode", 4096, 2))
    ESP_LOGE("MAIN", "Failed MotorNode");
  if (!g_actuatorNode.start("ActuatorNode", 4096, 2))
    ESP_LOGE("MAIN", "Failed ActuatorNode");
  if (!g_servoNode.start("ServoNode", 4096, 2))
    ESP_LOGE("MAIN", "Failed ServoNode");

  // 2. Link the global messaging queues
  servoCmdQueue = g_servoNode.getCmdQueue();
  servoTelQueue = g_servoNode.getTelQueue();
  actuatorCmdQueue = g_actuatorNode.getCmdQueue();
  actuatorTelQueue = g_actuatorNode.getTelQueue();
  motorCmdQueue = g_motorNode.getCmdQueue();
  motorTelQueue = g_motorNode.getTelQueue();

  // 3. Create Dependent Tasks
  xTaskCreate(encoderTask, "EncoderTask", 4096, NULL, 3, NULL);
  xTaskCreate(controller_task, "Controller", 4096, NULL, 3, NULL);
  xTaskCreate(LCD_task, "LCD", 8192, &lcd_interval, 2, NULL);

  // --- NEW: LOWER SHIELD ---
  // Restore setup() to Priority 1, allowing the RTOS scheduler to take over
  vTaskPrioritySet(NULL, 1);
}

void loop() {
  // Delete the default Arduino loop task to reclaim its memory stack
  vTaskDelete(NULL);
}