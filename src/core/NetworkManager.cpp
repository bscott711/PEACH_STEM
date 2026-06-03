#include "core/NetworkManager.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "HardwareConfig.h"
#include "messaging.h"
#include "drivers/LCDDriver.h" // For draw_wifiStatus

static volatile bool g_otaActive = false;
static volatile int g_otaProgress = 0;
static const char* g_otaStatus = "";

static const char* ssid = "sdsmtopn"; // From main.cpp

void NetworkManager::init() {
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

void NetworkManager::handle() {
    ArduinoOTA.handle();
}

bool NetworkManager::isOTAActive() {
    return g_otaActive;
}

int NetworkManager::getOTAProgress() {
    return g_otaProgress;
}

const char* NetworkManager::getOTAStatus() {
    return g_otaStatus;
}
