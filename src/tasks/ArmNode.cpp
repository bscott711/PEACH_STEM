#include "tasks/ArmNode.h"
#include <esp_log.h>

static const char* TAG = "ARM_NODE";

ArmNode::ArmNode()
    : currentPosition(0.0f)
    , targetSpeed(0)
    , calStart(-1)
    , calCenter(-1)
    , isTrackingTarget(false)
    , targetTrackingAbsSteps(0.0f) {
}

ArmNode::~ArmNode() {
    preferences.end();
}

void ArmNode::hwInit() {
    // Initialize TMC2209 driver for Arm on Address 1
    // Note: Serial1 is shared and initialized globally in main.cpp!
    vTaskDelay(pdMS_TO_TICKS(200));
    driver.begin(Serial1, TMC2209::SERIAL_ADDRESS_1);
    
    currentPosition = 0.0f;
    
    // Open NVS namespace
    if (!preferences.begin("peach", false)) {
        ESP_LOGE(TAG, "Failed to open NVS namespace");
    } else {
        calStart = preferences.getInt("armCalS", -1);
        calCenter = preferences.getInt("armCalC", -1);
        
        ESP_LOGI(TAG, "Loaded Arm cal: Start=%d, Center=%d", calStart, calCenter);
    }
}

void ArmNode::processCommand(const ArmCommand& cmd) {
    switch (cmd.action) {
        case ArmCmdAction::SET_SPEED:
            targetSpeed = (int)cmd.value;
            isTrackingTarget = false;
            ESP_LOGD(TAG, "Arm set speed: %d", targetSpeed);
            break;
            
        case ArmCmdAction::SET_TARGET:
            if (calStart != -1 && calCenter != -1) {
                targetTrackingAbsSteps = calStart + (cmd.value / 100.0f) * (calCenter - calStart);
                isTrackingTarget = true;
                ESP_LOGI(TAG, "Arm tracking target: %.2f%% -> %.2f steps", cmd.value, targetTrackingAbsSteps);
            } else {
                ESP_LOGW(TAG, "Arm SET_TARGET ignored: Uncalibrated!");
            }
            break;
            
        case ArmCmdAction::SET_CAL_START:
            calStart = (cmd.value < 0) ? -1 : (int)currentPosition;
            if (preferences.begin("peach", false)) {
                preferences.putInt("armCalS", calStart);
                preferences.end();
            }
            ESP_LOGI(TAG, "Arm CalStart set to %d", calStart);
            break;
            
        case ArmCmdAction::SET_CAL_CENTER:
            calCenter = (cmd.value < 0) ? -1 : (int)currentPosition;
            if (preferences.begin("peach", false)) {
                preferences.putInt("armCalC", calCenter);
                preferences.end();
            }
            ESP_LOGI(TAG, "Arm CalCenter set to %d", calCenter);
            break;
            
        case ArmCmdAction::GET_CAL_DATA:
            break;
    }
}

void ArmNode::hwUpdate() {
    if (isTrackingTarget) {
        float error = targetTrackingAbsSteps - currentPosition;
        
        // P-Controller
        int pControlSpeed = (int)(error * 10.0f); // Gain
        int maxTrackingSpeed = 20000;
        
        targetSpeed = constrain(pControlSpeed, -maxTrackingSpeed, maxTrackingSpeed);
        
        // Deadband
        if (abs(error) < 5.0f) {
            targetSpeed = 0;
            isTrackingTarget = false;
        }
    }

    // Basic velocity control and position integration
    if (targetSpeed != 0) {
        float deltaPos = (1.372e-6f * (float)targetSpeed * (float)TASK_UPDATE_INTERVAL_MS);
        currentPosition += deltaPos;
    }
    
    driver.setVelocity(targetSpeed);
}

ArmTelemetry ArmNode::generateTelemetry() {
    ArmTelemetry tel;
    tel.currentPosition = currentPosition;
    tel.targetPosition = isTrackingTarget ? targetTrackingAbsSteps : currentPosition;
    tel.calStart = calStart;
    tel.calCenter = calCenter;
    return tel;
}

bool ArmNode::setSpeed(int speed) {
    ArmCommand cmd;
    cmd.action = ArmCmdAction::SET_SPEED;
    cmd.value = (float)speed;
    return sendCommand(cmd);
}

bool ArmNode::setTarget(float percent) {
    ArmCommand cmd;
    cmd.action = ArmCmdAction::SET_TARGET;
    cmd.value = percent;
    return sendCommand(cmd);
}

bool ArmNode::setCalStart(int value) {
    ArmCommand cmd;
    cmd.action = ArmCmdAction::SET_CAL_START;
    cmd.value = (float)value;
    return sendCommand(cmd);
}

bool ArmNode::setCalCenter(int value) {
    ArmCommand cmd;
    cmd.action = ArmCmdAction::SET_CAL_CENTER;
    cmd.value = (float)value;
    return sendCommand(cmd);
}
