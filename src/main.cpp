#include "controller.h"
#include "drivers/EncoderDriver.h"
#include "messaging.h"
#include "tasks/DishRotationNode.h"
#include "tasks/LCD_task.h"
#include "tasks/DishLiftNode.h"
#include "tasks/ScraperArmNode.h"
#include "tasks/encoder_task.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "HardwareConfig.h"
#include "core/NetworkManager.h"

// Global Node instances (extern in controller.cpp)
ScraperArmNode g_scraperArmNode;
DishRotationNode g_dishRotationNode;
DishLiftNode g_dishLiftNode;

// Removed initWiFiAndOTA

void setup() {
  // Begin USB serial for debugging/monitoring
  Serial.begin(115200);

  // Initialize System State from NVS
  initSystemState();

  // Steppers cannot be disabled via EN_PIN because it doesn't exist

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
  if (!g_dishLiftNode.start("DishLiftNode", 4096, 2))
    PEACH_LOGE("MAIN", "Failed DishLiftNode");
  if (!g_dishRotationNode.start("DishRotationNode", 4096, 2))
    PEACH_LOGE("MAIN", "Failed DishRotationNode");
  if (!g_scraperArmNode.start("ScraperArmNode", 4096, 2))
    PEACH_LOGE("MAIN", "Failed ScraperArmNode");

  // 2. Link the global messaging queues
  scraperArmCmdQueue = g_scraperArmNode.getCmdQueue();
  scraperArmTelQueue = g_scraperArmNode.getTelQueue();
  dishRotationCmdQueue = g_dishRotationNode.getCmdQueue();
  dishRotationTelQueue = g_dishRotationNode.getTelQueue();
  dishLiftCmdQueue = g_dishLiftNode.getCmdQueue();
  dishLiftTelQueue = g_dishLiftNode.getTelQueue();
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