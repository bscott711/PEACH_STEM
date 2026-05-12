#include "tasks/ServoNode.h"
#include "drivers/ServoDriver.h"
#include <esp_log.h>

static const char* TAG = "SERVO_NODE";

ServoNode::ServoNode()
    : currentPercent(0.0f)
    , targetPercent(50.0f)  // Default to center
    , isActive(false)
    , calStart(-1)
    , calCenter(-1) {
}

ServoNode::~ServoNode() {
    preferences.end();
}

void ServoNode::hwInit() {
    // Initialize PWM hardware for servo
    ServoDriver_Init();
    
    // Open NVS namespace for calibration storage
    if (!preferences.begin("peach", false)) {
        ESP_LOGE(TAG, "Failed to open NVS namespace");
    } else {
        // Load calibration data from NVS
        calStart = preferences.getInt("srvStart", -1);
        calCenter = preferences.getInt("srvCenter", -1);
        
        ESP_LOGI(TAG, "Loaded calibration: start=%d, center=%d", calStart, calCenter);
        
        // Initialize position to calibrated center if available
        if (calCenter != -1) {
            currentPercent = (float)calCenter;
            targetPercent = (float)calCenter;
        }
    }
}

void ServoNode::processCommand(const ServoCommand& cmd) {
    switch (cmd.action) {
        case ServoCmdAction::SET_TARGET:
            targetPercent = constrain(cmd.value, 0.0f, 100.0f);
            isActive = true;
            ESP_LOGD(TAG, "Set target: %.2f%%", targetPercent);
            break;
            
        case ServoCmdAction::ACTIVATE:
            isActive = true;
            ESP_LOGD(TAG, "Servo activated");
            break;
            
        case ServoCmdAction::DEACTIVATE:
            isActive = false;
            ESP_LOGD(TAG, "Servo deactivated (limp)");
            break;
            
        case ServoCmdAction::SET_CAL_START:
            calStart = (int)cmd.value;
            ESP_LOGI(TAG, "Calibration START set to %d", calStart);
            break;
            
        case ServoCmdAction::SET_CAL_CENTER:
            calCenter = (int)cmd.value;
            ESP_LOGI(TAG, "Calibration CENTER set to %d", calCenter);
            break;
            
        case ServoCmdAction::GET_CAL_DATA:
            // Telemetry will include calibration data automatically
            break;
    }
}

void ServoNode::hwUpdate() {
    // Smooth ramp toward target using high-resolution float math
    if (currentPercent < targetPercent) {
        currentPercent += STEP_SIZE;
        if (currentPercent > targetPercent) {
            currentPercent = targetPercent;
        }
    } else if (currentPercent > targetPercent) {
        currentPercent -= STEP_SIZE;
        if (currentPercent < targetPercent) {
            currentPercent = targetPercent;
        }
    }
    
    // Command hardware with exact float position
    if (isActive) {
        ServoDriver_WritePercent(currentPercent);
    } else {
        ServoDriver_Disable();  // Leave servo limp
    }
}

ServoTelemetry ServoNode::generateTelemetry() {
    ServoTelemetry tel;
    tel.currentPercent = currentPercent;
    tel.targetPercent = targetPercent;
    tel.isActive = isActive;
    tel.calStart = calStart;
    tel.calCenter = calCenter;
    return tel;
}

bool ServoNode::setTarget(float percent) {
    ServoCommand cmd;
    cmd.action = ServoCmdAction::SET_TARGET;
    cmd.value = percent;
    return sendCommand(cmd);
}

bool ServoNode::activate() {
    ServoCommand cmd;
    cmd.action = ServoCmdAction::ACTIVATE;
    cmd.value = 0;
    return sendCommand(cmd);
}

bool ServoNode::deactivate() {
    ServoCommand cmd;
    cmd.action = ServoCmdAction::DEACTIVATE;
    cmd.value = 0;
    return sendCommand(cmd);
}

bool ServoNode::setCalStart(int value) {
    ServoCommand cmd;
    cmd.action = ServoCmdAction::SET_CAL_START;
    cmd.value = (float)value;
    bool result = sendCommand(cmd);
    
    if (result && preferences.begin("peach", false)) {
        preferences.putInt("srvStart", calStart);
        preferences.end();
        ESP_LOGI(TAG, "Saved calibration START to NVS: %d", calStart);
    }
    
    return result;
}

bool ServoNode::setCalCenter(int value) {
    ServoCommand cmd;
    cmd.action = ServoCmdAction::SET_CAL_CENTER;
    cmd.value = (float)value;
    bool result = sendCommand(cmd);
    
    if (result && preferences.begin("peach", false)) {
        preferences.putInt("srvCenter", calCenter);
        preferences.end();
        ESP_LOGI(TAG, "Saved calibration CENTER to NVS: %d", calCenter);
    }
    
    return result;
}
