#include "controller.h"
#include "drivers/EncoderDriver.h"
#include "messaging.h"
#include "tasks/ActuatorNode.h"
#include "tasks/LCD_task.h"
#include "tasks/MotorNode.h"
#include "tasks/ArmNode.h"
#include "tasks/encoder_task.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "HardwareConfig.h"
#include "core/NetworkManager.h"

// Global Node instances (extern in controller.cpp)
ArmNode g_armNode;
ActuatorNode g_actuatorNode;
MotorNode g_motorNode;

// Removed initWiFiAndOTA

void setup() {
  // Begin USB serial for debugging/monitoring
  Serial.begin(115200);

  // Initialize System State from NVS
  initSystemState();

  // Start I2C Line (Used by encoder)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Initialize shared UART for Steppers (Address 0 & 1)
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);

  // Inits
  encoderInit();
  LCDInit();

  // Connect WiFi and setup OTA
  NetworkManager::init();

  // Task Update Intervals
  static int lcd_interval = TASK_REFRESH_LCD;

  // --- NEW: THE PRIORITY SHIELD ---
  // Elevate setup() to Priority 5 (higher than our tasks) so it cannot be
  // preempted before it finishes linking the global queues.
  vTaskPrioritySet(NULL, 5);

  // 1. Start Active Motion Nodes
  if (!g_motorNode.start("MotorNode", 4096, 2))
    PEACH_LOGE("MAIN", "Failed MotorNode");
  if (!g_actuatorNode.start("ActuatorNode", 4096, 2))
    PEACH_LOGE("MAIN", "Failed ActuatorNode");
  if (!g_armNode.start("ArmNode", 4096, 2))
    PEACH_LOGE("MAIN", "Failed ArmNode");

  // 2. Link the global messaging queues
  armCmdQueue = g_armNode.getCmdQueue();
  armTelQueue = g_armNode.getTelQueue();
  actuatorCmdQueue = g_actuatorNode.getCmdQueue();
  actuatorTelQueue = g_actuatorNode.getTelQueue();
  motorCmdQueue = g_motorNode.getCmdQueue();
  motorTelQueue = g_motorNode.getTelQueue();
  lcdDataQueue = xQueueCreate(1, sizeof(UIData));

  // 3. Create Dependent Tasks
  xTaskCreate(encoderTask, "EncoderTask", 4096, NULL, 3, NULL);
  xTaskCreate(controller_task, "Controller", 4096, NULL, 3, NULL);
  xTaskCreate(LCD_task, "LCD", 8192, &lcd_interval, 2, NULL);

  // --- NEW: LOWER SHIELD ---
  // Restore setup() to Priority 1, allowing the RTOS scheduler to take over
  vTaskPrioritySet(NULL, 1);
}

void loop() {
  NetworkManager::handle();
  vTaskDelay(pdMS_TO_TICKS(50));
}