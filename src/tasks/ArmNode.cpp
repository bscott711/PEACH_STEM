#include "tasks/ArmNode.h"
#include <esp_log.h>

static const char* TAG = "ARM_NODE";

ArmNode::ArmNode()
    : currentPosition(0.0f)
    , targetSpeed(0)
    , posOut(-1)
    , posIn(-1)
    , isTrackingTarget(false)
    , targetTrackingAbsSteps(0.0f)
    , lastSavedPosition(-999.0f) {
}

ArmNode::~ArmNode() {
    preferences.end();
}

void ArmNode::hwInit() {
    // Initialize TMC2209 driver for Arm on Address 1
    // Note: Serial1 is shared and initialized globally in main.cpp!
    vTaskDelay(pdMS_TO_TICKS(200));
    driver.begin(Serial1, TMC2209::SERIAL_ADDRESS_1);
    
    // Open NVS namespace
    if (!preferences.begin("peach", false)) {
        ESP_LOGE(TAG, "Failed to open NVS namespace");
    } else {
        // Load calibration positions
        posOut = preferences.getInt("armPosO", -1);
        posIn = preferences.getInt("armPosI", -1);
        
        // Load last known position (restored on boot)
        float lastPos = preferences.getFloat("armPos", 0.0f);
        currentPosition = lastPos;
        lastSavedPosition = lastPos;
        
        // Close NVS after loading — reopen per-write
        preferences.end();
        
        ESP_LOGI(TAG, "Loaded Arm: posOut=%d, posIn=%d, lastPos=%.2f", posOut, posIn, lastPos);
    }
}

void ArmNode::processCommand(const ArmCommand& cmd) {
    switch (cmd.action) {
        case ArmCmdAction::SET_SPEED:
            targetSpeed = (int)cmd.value;
            isTrackingTarget = false;
            ESP_LOGD(TAG, "Arm set speed: %d", targetSpeed);
            break;
            
        case ArmCmdAction::STOP:
            targetSpeed = 0;
            // Don't clear isTrackingTarget — if we were tracking, the 
            // P-controller in hwUpdate will resume. STOP is for jog mode only.
            if (!isTrackingTarget) {
                ESP_LOGD(TAG, "Arm stopped");
            }
            break;
            
        case ArmCmdAction::SET_TARGET:
            if (posOut != -1 && posIn != -1) {
                targetTrackingAbsSteps = posOut + (cmd.value / 100.0f) * (posIn - posOut);
                isTrackingTarget = true;
                ESP_LOGI(TAG, "Arm tracking target: %.2f%% -> %.2f steps", cmd.value, targetTrackingAbsSteps);
            } else {
                ESP_LOGW(TAG, "Arm SET_TARGET ignored: Uncalibrated!");
            }
            break;
            
        case ArmCmdAction::SET_POS_OUT:
            posOut = (int)currentPosition;
            if (preferences.begin("peach", false)) {
                preferences.putInt("armPosO", posOut);
                preferences.end();
            }
            ESP_LOGI(TAG, "Arm posOut set to %d", posOut);
            break;
            
        case ArmCmdAction::SET_POS_IN:
            posIn = (int)currentPosition;
            if (preferences.begin("peach", false)) {
                preferences.putInt("armPosI", posIn);
                preferences.end();
            }
            ESP_LOGI(TAG, "Arm posIn set to %d", posIn);
            break;
            
        case ArmCmdAction::CLEAR_CAL:
            posOut = -1;
            posIn = -1;
            if (preferences.begin("peach", false)) {
                preferences.putInt("armPosO", -1);
                preferences.putInt("armPosI", -1);
                preferences.end();
            }
            ESP_LOGI(TAG, "Arm calibration cleared");
            break;
    }
}

void ArmNode::hwUpdate() {
    if (isTrackingTarget) {
        float error = targetTrackingAbsSteps - currentPosition;
        
        // P-Controller with proportional gain
        int pControlSpeed = (int)(error * 10.0f);
        int maxTrackingSpeed = 20000;
        
        targetSpeed = constrain(pControlSpeed, -maxTrackingSpeed, maxTrackingSpeed);
        
        // Deadband — close enough to target, stop tracking
        if (abs(error) < 5.0f) {
            targetSpeed = 0;
            isTrackingTarget = false;
        }
    }

    // Velocity control and position integration
    if (targetSpeed != 0) {
        float deltaPos = (1.372e-6f * (float)targetSpeed * (float)TASK_UPDATE_INTERVAL_MS);
        currentPosition += deltaPos;
    }
    
    driver.setVelocity(targetSpeed);
    
    // Save position to NVS when stopped and position has changed
    if (targetSpeed == 0 && abs(currentPosition - lastSavedPosition) > 0.1f) {
        if (preferences.begin("peach", false)) {
            preferences.putFloat("armPos", currentPosition);
            preferences.end();
            lastSavedPosition = currentPosition;
            ESP_LOGI(TAG, "Saved arm position: %.2f", currentPosition);
        }
    }
}

ArmTelemetry ArmNode::generateTelemetry() {
    ArmTelemetry tel;
    tel.currentPosition = currentPosition;
    tel.targetPosition = isTrackingTarget ? targetTrackingAbsSteps : currentPosition;
    tel.posOut = posOut;
    tel.posIn = posIn;
    tel.isMoving = (targetSpeed != 0);
    return tel;
}

bool ArmNode::setSpeed(int speed) {
    ArmCommand cmd;
    cmd.action = ArmCmdAction::SET_SPEED;
    cmd.value = (float)speed;
    return sendCommand(cmd);
}

bool ArmNode::stop() {
    ArmCommand cmd;
    cmd.action = ArmCmdAction::STOP;
    cmd.value = 0.0f;
    return sendCommand(cmd);
}

bool ArmNode::setTarget(float percent) {
    ArmCommand cmd;
    cmd.action = ArmCmdAction::SET_TARGET;
    cmd.value = percent;
    return sendCommand(cmd);
}

bool ArmNode::setPosOut() {
    ArmCommand cmd;
    cmd.action = ArmCmdAction::SET_POS_OUT;
    cmd.value = 0.0f;
    return sendCommand(cmd);
}

bool ArmNode::setPosIn() {
    ArmCommand cmd;
    cmd.action = ArmCmdAction::SET_POS_IN;
    cmd.value = 0.0f;
    return sendCommand(cmd);
}

bool ArmNode::clearCal() {
    ArmCommand cmd;
    cmd.action = ArmCmdAction::CLEAR_CAL;
    cmd.value = 0.0f;
    return sendCommand(cmd);
}
