#include "tasks/ActuatorNode.h"
#include "drivers/HBridgeDriver.h"
#include "controller.h"
#include "core/NetworkManager.h"
#include <cmath>

static const char* TAG = "ACTUATOR_NODE";

ActuatorNode::ActuatorNode()
    : currentPercent(0.0f)
    , targetPercent(0)
    , targetSpeedPWM(255)
    , lastSavedPercent(-1.0f)
    , wasMoving(false) {
    limits[0] = 0; limits[1] = 0; limits[2] = 0;
    limitSet[0] = false; limitSet[1] = false; limitSet[2] = false;
}

ActuatorNode::~ActuatorNode() {
}

void ActuatorNode::hwInit() {
    // Initialize H-bridge hardware
    HBridge_Init();
    
    // Open NVS namespace for limit and position storage
    StorageManager::loadActuatorLimits(limits, limitSet);
    float lastPos = StorageManager::loadActuatorPosition();
    
    currentPercent = lastPos;
    targetPercent = (int)currentPercent;
    lastSavedPercent = lastPos;
    
    PEACH_LOGI(TAG, "Loaded limits: Bot=%d(%s), Mid=%d(%s), Top=%d(%s), Pos=%.1f",
             limits[0], limitSet[0] ? "Y" : "N",
             limits[1], limitSet[1] ? "Y" : "N",
             limits[2], limitSet[2] ? "Y" : "N",
             lastPos);
}

void ActuatorNode::processCommand(const ActuatorCommand& cmd) {
    switch (cmd.action) {
        case ActuatorCmdAction::SET_TARGET:
            targetPercent = constrain(cmd.value, 0, 100);
            targetSpeedPWM = constrain(cmd.pwmSpeed, 0, 255);
            wasMoving = true; // Ensure event triggers even if already at target
            PEACH_LOGD(TAG, "Set target: %d%%, spd: %d", targetPercent, targetSpeedPWM);
            break;
            
        case ActuatorCmdAction::SET_LIMIT_BOT:
            limits[0] = cmd.value;
            limitSet[0] = true;
            StorageManager::saveActuatorLimit(StorageManager::LIMIT_BOT, limits[0], true);
            PEACH_LOGI(TAG, "Bottom limit set to %d%%", limits[0]);
            break;
            
        case ActuatorCmdAction::SET_LIMIT_MID:
            limits[1] = cmd.value;
            limitSet[1] = true;
            StorageManager::saveActuatorLimit(StorageManager::LIMIT_MID, limits[1], true);
            PEACH_LOGI(TAG, "Middle limit set to %d%%", limits[1]);
            break;
            
        case ActuatorCmdAction::SET_LIMIT_TOP:
            limits[2] = cmd.value;
            limitSet[2] = true;
            StorageManager::saveActuatorLimit(StorageManager::LIMIT_TOP, limits[2], true);
            PEACH_LOGI(TAG, "Top limit set to %d%%", limits[2]);
            break;
            
        case ActuatorCmdAction::CLEAR_LIMIT_BOT:
            limitSet[0] = false;
            StorageManager::saveActuatorLimit(StorageManager::LIMIT_BOT, limits[0], false);
            PEACH_LOGI(TAG, "Bottom limit cleared");
            break;
            
        case ActuatorCmdAction::CLEAR_LIMIT_MID:
            limitSet[1] = false;
            StorageManager::saveActuatorLimit(StorageManager::LIMIT_MID, limits[1], false);
            PEACH_LOGI(TAG, "Middle limit cleared");
            break;
            
        case ActuatorCmdAction::CLEAR_LIMIT_TOP:
            limitSet[2] = false;
            StorageManager::saveActuatorLimit(StorageManager::LIMIT_TOP, limits[2], false);
            PEACH_LOGI(TAG, "Top limit cleared");
            break;
            
        case ActuatorCmdAction::GET_LIMITS:
            // Telemetry will include limit data automatically
            break;
    }
}

// Helper function for empirical piecewise linear interpolation
static float interpolateTime(int pwm, const int pwms[], const float times[], int size) {
    if (pwm <= pwms[0]) return times[0];
    if (pwm >= pwms[size - 1]) return times[size - 1];
    
    for (int i = 0; i < size - 1; i++) {
        if (pwm >= pwms[i] && pwm <= pwms[i+1]) {
            float t = (float)(pwm - pwms[i]) / (float)(pwms[i+1] - pwms[i]);
            return times[i] + t * (times[i+1] - times[i]);
        }
    }
    return times[size - 1];
}

void ActuatorNode::hwUpdate() {
    float timeMs = 3000.0f; // Default safe fallback

    if (currentPercent < targetPercent) {
        wasMoving = true;
        // ==========================
        // EXTENDING (Forward)
        // Measured empirical data
        // ==========================
        const int pwms[] = {155, 165, 175, 205, 255};
        const float times[] = {6000.0f, 4000.0f, 3000.0f, 1800.0f, 800.0f};
        timeMs = interpolateTime(targetSpeedPWM, pwms, times, 5);
        
        float dynamicPctPerTick = (100.0f * (float)TASK_UPDATE_INTERVAL_MS) / timeMs;
        currentPercent += dynamicPctPerTick;
        
        if (currentPercent > targetPercent) {
            currentPercent = targetPercent;  // Clamp exact arrival
        }
        HBridge_Set(ACT_FORWARD, targetSpeedPWM);
        
    } else if (currentPercent > targetPercent) {
        wasMoving = true;
        // ==========================
        // RETRACTING (Reverse)
        // Measured empirical data
        // ==========================
        const int pwms[] = {155, 175, 205, 255};
        const float times[] = {3000.0f, 3000.0f, 1500.0f, 800.0f};
        timeMs = interpolateTime(targetSpeedPWM, pwms, times, 4);
        
        float dynamicPctPerTick = (100.0f * (float)TASK_UPDATE_INTERVAL_MS) / timeMs;
        currentPercent -= dynamicPctPerTick;
        
        if (currentPercent < targetPercent) {
            currentPercent = targetPercent;  // Clamp exact arrival
        }
        HBridge_Set(ACT_REVERSE, targetSpeedPWM);
        
    } else {
        HBridge_Set(ACT_STOP);
        
        if (wasMoving) {
            // Save position to NVS if it has changed since last save
            if (std::abs(currentPercent - lastSavedPercent) > 0.1f) {
                StorageManager::saveActuatorPosition(currentPercent);    
                lastSavedPercent = currentPercent;
                PEACH_LOGI(TAG, "Saved actuator position: %.2f%%", currentPercent);
            }
            xEventGroupSetBits(controlEvents, BIT_POS_REACHED_ACT);
            wasMoving = false;
        }
    }
}

ActuatorTelemetry ActuatorNode::generateTelemetry() {
    ActuatorTelemetry tel;
    tel.currentPercent = currentPercent;
    tel.targetPercent = targetPercent;
    tel.limits[0] = limits[0];
    tel.limits[1] = limits[1];
    tel.limits[2] = limits[2];
    tel.limitSet[0] = limitSet[0];
    tel.limitSet[1] = limitSet[1];
    tel.limitSet[2] = limitSet[2];
    return tel;
}

bool ActuatorNode::setTarget(int percent, int pwmSpeed) {
    ActuatorCommand cmd;
    cmd.action = ActuatorCmdAction::SET_TARGET;
    cmd.value = percent;
    cmd.pwmSpeed = pwmSpeed;
    return sendCommand(cmd);
}

bool ActuatorNode::setLimitBot(int percent) {
    ActuatorCommand cmd;
    cmd.action = ActuatorCmdAction::SET_LIMIT_BOT;
    cmd.value = percent;
    return sendCommand(cmd);
}

bool ActuatorNode::setLimitMid(int percent) {
    ActuatorCommand cmd;
    cmd.action = ActuatorCmdAction::SET_LIMIT_MID;
    cmd.value = percent;
    return sendCommand(cmd);
}

bool ActuatorNode::setLimitTop(int percent) {
    ActuatorCommand cmd;
    cmd.action = ActuatorCmdAction::SET_LIMIT_TOP;
    cmd.value = percent;
    return sendCommand(cmd);
}

bool ActuatorNode::clearLimitBot() {
    ActuatorCommand cmd;
    cmd.action = ActuatorCmdAction::CLEAR_LIMIT_BOT;
    cmd.value = 0;
    return sendCommand(cmd);
}

bool ActuatorNode::clearLimitMid() {
    ActuatorCommand cmd;
    cmd.action = ActuatorCmdAction::CLEAR_LIMIT_MID;
    cmd.value = 0;
    return sendCommand(cmd);
}

bool ActuatorNode::clearLimitTop() {
    ActuatorCommand cmd;
    cmd.action = ActuatorCmdAction::CLEAR_LIMIT_TOP;
    cmd.value = 0;
    return sendCommand(cmd);
}
