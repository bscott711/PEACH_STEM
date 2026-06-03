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

// Global Node instances (extern in controller.cpp)
ArmNode g_armNode;
ActuatorNode g_actuatorNode;
MotorNode g_motorNode;

const char* ssid = "sdsmtopn";

void initWiFiAndOTA() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false); // Disable power save for reliable OTA (UDP/mDNS)
  WiFi.begin(ssid);

  int attempt = 0;
  Serial.println("Connecting to WiFi...");
  
  while (true) {
    wl_status_t status = WiFi.status();
    if (status == WL_CONNECTED) {
      break;
    }
    
    if (attempt >= 30) { // 15 seconds timeout
      Serial.println("Connection Failed! Rebooting...");
      draw_wifiStatus("Failed! Timeout", ssid, attempt, true);
      delay(5000);
      ESP.restart();
    }
    
    draw_wifiStatus("Connecting", ssid, attempt, false);
    delay(500);
    attempt++;
  }

  // Once connected
  Serial.println("WiFi Connected!");
  draw_wifiStatus("peach-pit-esp32.local", ssid, 0, false);
  delay(2000); // Show name for 2 seconds

  // Initialize ArduinoOTA
  ArduinoOTA.setHostname("peach-pit-esp32");

  ArduinoOTA
    .onStart([]() {
      g_otaActive = true;
      g_otaProgress = 0;
      
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
        g_otaStatus = "Updating Sketch";
      } else { // U_SPIFFS
        type = "filesystem";
        g_otaStatus = "Updating Filesystem";
      }
      Serial.println("Start updating " + type);
      
      // Safety Interlocks
      if (motorCmdQueue != NULL) {
        MotorCommand stopMotor = { MotorCmdAction::SET_SPEED, 0.0f };
        xQueueSend(motorCmdQueue, &stopMotor, 0);
      }
      if (actuatorCmdQueue != NULL) {
        ActuatorCommand stopAct = { ActuatorCmdAction::SET_TARGET, 0 };
        ActuatorTelemetry actTel;
        if (actuatorTelQueue != NULL && xQueuePeek(actuatorTelQueue, &actTel, 0) == pdPASS) {
          stopAct.value = actTel.currentPercent;
        }
        xQueueSend(actuatorCmdQueue, &stopAct, 0);
      }
      if (armCmdQueue != NULL) {
        ArmCommand stopArm = { ArmCmdAction::SET_SPEED, 0.0f };
        xQueueSend(armCmdQueue, &stopArm, 0);
      }
    })
    .onEnd([]() {
      g_otaProgress = 100;
      g_otaStatus = "Success! Rebooting";
      Serial.println("\nEnd");
      g_otaActive = true;
      // Removed draw_otaScreen() to prevent thread collision with LCD_task
      delay(1000);
      g_otaActive = false;
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      g_otaProgress = (progress * 100) / total;
      Serial.printf("Progress: %u%%\r", g_otaProgress);
    })
    .onError([](ota_error_t error) {
      g_otaActive = false;
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) g_otaStatus = "Auth Failed";
      else if (error == OTA_BEGIN_ERROR) g_otaStatus = "Begin Failed";
      else if (error == OTA_CONNECT_ERROR) g_otaStatus = "Connect Failed";
      else if (error == OTA_RECEIVE_ERROR) g_otaStatus = "Receive Failed";
      else if (error == OTA_END_ERROR) g_otaStatus = "End Failed";
    });

  if (WiFi.status() == WL_CONNECTED) {
    if (MDNS.begin("peach-pit-esp32")) {
      Serial.println("mDNS responder started: peach-pit-esp32.local");
    }
    ArduinoOTA.setHostname("peach-pit-esp32");
    ArduinoOTA.begin();
    Serial.println("OTA Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
}

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
  initWiFiAndOTA();

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
  if (!g_armNode.start("ArmNode", 4096, 2))
    ESP_LOGE("MAIN", "Failed ArmNode");

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
  ArduinoOTA.handle();
  vTaskDelay(pdMS_TO_TICKS(50));
}