#include "tasks/ActuatorNode.h"
#include "drivers/HBridgeDriver.h"
#include <esp_log.h>

static const char* TAG = "ACTUATOR_NODE";

ActuatorNode::ActuatorNode()
    : currentPercent(0.0f)
    , targetPercent(0)
    , lastSavedPercent(-1.0f) {
    limits[0] = 0; limits[1] = 0; limits[2] = 0;
    limitSet[0] = false; limitSet[1] = false; limitSet[2] = false;
}

ActuatorNode::~ActuatorNode() {
    preferences.end();
}

void ActuatorNode::hwInit() {
    // Initialize H-bridge hardware
    HBridge_Init();
    
    // Open NVS namespace for limit and position storage
    if (!preferences.begin("peach", false)) {
        ESP_LOGE(TAG, "Failed to open NVS namespace");
    } else {
        // Load last known position
        float lastPos = preferences.getFloat("actPos", 0.0f);
        currentPercent = lastPos;
        targetPercent = (int)lastPos;
        lastSavedPercent = lastPos;
        ESP_LOGI(TAG, "Resuming actuator at last position: %.2f%%", lastPos);

        // Load limit data from NVS
        limits[0] = preferences.getInt("actB", 0);
        limits[1] = preferences.getInt("actM", 0);
        limits[2] = preferences.getInt("actT", 0);
        limitSet[0] = preferences.getBool("actS_B", false);
        limitSet[1] = preferences.getBool("actS_M", false);
        limitSet[2] = preferences.getBool("actS_T", false);
        
        // Close NVS after loading — reopen per-write to avoid stale handles
        preferences.end();
        
        ESP_LOGI(TAG, "Loaded limits: Bot=%d(%s), Mid=%d(%s), Top=%d(%s)",
                 limits[0], limitSet[0] ? "Y" : "N",
                 limits[1], limitSet[1] ? "Y" : "N",
                 limits[2], limitSet[2] ? "Y" : "N");
    }
}

void ActuatorNode::processCommand(const ActuatorCommand& cmd) {
    switch (cmd.action) {
        case ActuatorCmdAction::SET_TARGET:
            targetPercent = constrain(cmd.value, 0, 100);
            ESP_LOGD(TAG, "Set target: %d%%", targetPercent);
            break;
            
        case ActuatorCmdAction::SET_LIMIT_BOT:
            limits[0] = cmd.value;
            limitSet[0] = true;
            if (preferences.begin("peach", false)) {
                preferences.putInt("actB", limits[0]);
                preferences.putBool("actS_B", true);
                preferences.end();
            }
            ESP_LOGI(TAG, "Bottom limit set to %d%%", limits[0]);
            break;
            
        case ActuatorCmdAction::SET_LIMIT_MID:
            limits[1] = cmd.value;
            limitSet[1] = true;
            if (preferences.begin("peach", false)) {
                preferences.putInt("actM", limits[1]);
                preferences.putBool("actS_M", true);
                preferences.end();
            }
            ESP_LOGI(TAG, "Middle limit set to %d%%", limits[1]);
            break;
            
        case ActuatorCmdAction::SET_LIMIT_TOP:
            limits[2] = cmd.value;
            limitSet[2] = true;
            if (preferences.begin("peach", false)) {
                preferences.putInt("actT", limits[2]);
                preferences.putBool("actS_T", true);
                preferences.end();
            }
            ESP_LOGI(TAG, "Top limit set to %d%%", limits[2]);
            break;
            
        case ActuatorCmdAction::CLEAR_LIMIT_BOT:
            limitSet[0] = false;
            if (preferences.begin("peach", false)) {
                preferences.putBool("actS_B", false);
                preferences.end();
            }
            ESP_LOGI(TAG, "Bottom limit cleared");
            break;
            
        case ActuatorCmdAction::CLEAR_LIMIT_MID:
            limitSet[1] = false;
            if (preferences.begin("peach", false)) {
                preferences.putBool("actS_M", false);
                preferences.end();
            }
            ESP_LOGI(TAG, "Middle limit cleared");
            break;
            
        case ActuatorCmdAction::CLEAR_LIMIT_TOP:
            limitSet[2] = false;
            if (preferences.begin("peach", false)) {
                preferences.putBool("actS_T", false);
                preferences.end();
            }
            ESP_LOGI(TAG, "Top limit cleared");
            break;
            
        case ActuatorCmdAction::GET_LIMITS:
            // Telemetry will include limit data automatically
            break;
    }
}

void ActuatorNode::hwUpdate() {
    float dynamicPctPerTick = (100.0f * (float)TASK_UPDATE_INTERVAL_MS) / (float)FULL_EXTEND_TIME_MS;

    // Non-blocking movement evaluation with float-based ramping
    if (currentPercent < targetPercent) {
        currentPercent += dynamicPctPerTick;
        if (currentPercent > targetPercent) {
            currentPercent = targetPercent;  // Clamp exact arrival
        }
        HBridge_Set(ACT_FORWARD, 255);
    } else if (currentPercent > targetPercent) {
        currentPercent -= dynamicPctPerTick;
        if (currentPercent < targetPercent) {
            currentPercent = targetPercent;  // Clamp exact arrival
        }
        HBridge_Set(ACT_REVERSE, 255);
    } else {
        HBridge_Set(ACT_STOP);
        
        // Save position to NVS if it has changed since last save
        if (abs(currentPercent - lastSavedPercent) > 0.1f) {
            if (preferences.begin("peach", false)) {
                preferences.putFloat("actPos", currentPercent);
                preferences.end();
                lastSavedPercent = currentPercent;
                ESP_LOGI(TAG, "Saved actuator position: %.2f%%", currentPercent);
            }
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

bool ActuatorNode::setTarget(int percent) {
    ActuatorCommand cmd;
    cmd.action = ActuatorCmdAction::SET_TARGET;
    cmd.value = percent;
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
