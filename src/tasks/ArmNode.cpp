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
}

void ArmNode::hwInit() {
    // Initialize TMC2209 driver for Arm on Address 1
    // Note: Serial1 is shared and initialized globally in main.cpp!
    vTaskDelay(pdMS_TO_TICKS(200));
    driver.begin(Serial1, TMC2209::SERIAL_ADDRESS_1);
    
    // Open NVS namespace
    StorageManager::loadArmCalibration(posOut, posIn);
    posBuffer = StorageManager::loadArmPosBuffer();
    float lastPos = StorageManager::loadArmPosition();
    
    // Set stepper to last known position immediately
    currentPosition = lastPos;
    lastSavedPosition = lastPos;
    
    ESP_LOGI(TAG, "Loaded calibration: Out=%d, Buf=%d, In=%d, Pos=%.1f", posOut, posBuffer, posIn, lastPos);
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
                targetTrackingSpeed = cmd.targetSpeed;
                isTrackingTarget = true;
                ESP_LOGI(TAG, "Arm tracking target: %.2f%% -> %.2f steps at speed %d", cmd.value, targetTrackingAbsSteps, targetTrackingSpeed);
            } else {
                ESP_LOGW(TAG, "Arm SET_TARGET ignored: Uncalibrated!");
            }
            break;
            
        case ArmCmdAction::JOG:
            if (!isTrackingTarget) {
                targetTrackingAbsSteps = currentPosition;
                isTrackingTarget = true;
            }
            targetTrackingAbsSteps += cmd.value;
            break;
            
        case ArmCmdAction::SET_POS_OUT:
            posOut = (int)currentPosition;
            StorageManager::saveArmPosOut(posOut);
            ESP_LOGI(TAG, "Arm posOut set to %d", posOut);
            break;
            
        case ArmCmdAction::SET_POS_BUFFER:
            posBuffer = (int)currentPosition;
            StorageManager::saveArmPosBuffer(posBuffer);
            ESP_LOGI(TAG, "Arm posBuffer set to %d", posBuffer);
            break;
            
        case ArmCmdAction::SET_POS_IN:
            posIn = (int)currentPosition;
            StorageManager::saveArmPosIn(posIn);
            ESP_LOGI(TAG, "Arm posIn set to %d", posIn);
            break;
            
        case ArmCmdAction::CLEAR_CAL:
            posOut = -1;
            posBuffer = -1;
            posIn = -1;
            StorageManager::saveArmPosOut(-1);
            StorageManager::saveArmPosBuffer(-1);
            StorageManager::saveArmPosIn(-1);
            ESP_LOGI(TAG, "Arm calibration cleared");
            break;
    }
}

void ArmNode::hwUpdate() {
    if (isTrackingTarget) {
        float error = targetTrackingAbsSteps - currentPosition;
        
        // Constant velocity tracking
        if (abs(error) < 5.0f) {
            targetSpeed = 0;
            isTrackingTarget = false;
        } else {
            targetSpeed = (error > 0) ? targetTrackingSpeed : -targetTrackingSpeed;
        }
    }

    // Velocity control and position integration
    if (targetSpeed != 0) {
        // VACTUAL to steps/sec factor is ~0.715
        float stepsPerSec = (float)targetSpeed * 0.715f;
        float deltaPos = stepsPerSec * ((float)TASK_UPDATE_INTERVAL_MS / 1000.0f);
        currentPosition += deltaPos;
    }
    
    driver.setVelocity(targetSpeed);
    
    // Save position to NVS when stopped and position has changed
    if (targetSpeed == 0 && abs(currentPosition - lastSavedPosition) > 0.1f) {
        StorageManager::saveArmPosition(currentPosition);
        lastSavedPosition = currentPosition;
        ESP_LOGI(TAG, "Saved arm position: %.2f", currentPosition);
    }
}

ArmTelemetry ArmNode::generateTelemetry() {
    ArmTelemetry tel;
    tel.currentPosition = currentPosition;
    tel.targetPosition = isTrackingTarget ? targetTrackingAbsSteps : currentPosition;
    tel.posOut = posOut;
    tel.posBuffer = posBuffer;
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

bool ArmNode::jog(float relativeSteps) {
    ArmCommand cmd;
    cmd.action = ArmCmdAction::JOG;
    cmd.value = relativeSteps;
    return sendCommand(cmd);
}

bool ArmNode::setTarget(float percent, int targetSpeed) {
    ArmCommand cmd;
    cmd.action = ArmCmdAction::SET_TARGET;
    cmd.value = percent;
    cmd.targetSpeed = targetSpeed;
    return sendCommand(cmd);
}

bool ArmNode::setPosOut() {
    ArmCommand cmd;
    cmd.action = ArmCmdAction::SET_POS_OUT;
    cmd.value = 0.0f;
    return sendCommand(cmd);
}

bool ArmNode::setPosBuffer() {
    ArmCommand cmd;
    cmd.action = ArmCmdAction::SET_POS_BUFFER;
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
