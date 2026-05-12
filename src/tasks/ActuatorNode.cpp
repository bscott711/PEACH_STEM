#include "tasks/ActuatorNode.h"
#include "drivers/HBridgeDriver.h"
#include <esp_log.h>

static const char* TAG = "ACTUATOR_NODE";

ActuatorNode::ActuatorNode()
    : currentPercent(0.0f)
    , targetPercent(0)
    , pctPerTick(0.0f) {
    limits[0] = 0; limits[1] = 0; limits[2] = 0;
    limitSet[0] = false; limitSet[1] = false; limitSet[2] = false;
}

ActuatorNode::~ActuatorNode() {
    preferences.end();
}

void ActuatorNode::hwInit() {
    // Initialize H-bridge hardware
    HBridge_Init();
    
    // Home (retract) actuator on boot
    // Safe to block here since RTOS scheduler is just starting
    HBridge_Set(ACT_REVERSE);
    vTaskDelay(pdMS_TO_TICKS(FULL_EXTEND_TIME_MS));
    HBridge_Set(ACT_STOP);
    
    // Calculate percentage traveled per task tick
    // e.g., (100.0 * 10ms) / 1000ms = 1.0% per tick
    pctPerTick = (100.0f * (float)TASK_UPDATE_INTERVAL_MS) / (float)FULL_EXTEND_TIME_MS;
    
    // Open NVS namespace for limit storage
    if (!preferences.begin("peach", false)) {
        ESP_LOGE(TAG, "Failed to open NVS namespace");
    } else {
        // Load limit data from NVS
        limits[0] = preferences.getInt("actB", 0);
        limits[1] = preferences.getInt("actM", 0);
        limits[2] = preferences.getInt("actT", 0);
        limitSet[0] = preferences.getBool("actS_B", false);
        limitSet[1] = preferences.getBool("actS_M", false);
        limitSet[2] = preferences.getBool("actS_T", false);
        
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
            ESP_LOGI(TAG, "Bottom limit set to %d%%", limits[0]);
            break;
            
        case ActuatorCmdAction::SET_LIMIT_MID:
            limits[1] = cmd.value;
            limitSet[1] = true;
            ESP_LOGI(TAG, "Middle limit set to %d%%", limits[1]);
            break;
            
        case ActuatorCmdAction::SET_LIMIT_TOP:
            limits[2] = cmd.value;
            limitSet[2] = true;
            ESP_LOGI(TAG, "Top limit set to %d%%", limits[2]);
            break;
            
        case ActuatorCmdAction::CLEAR_LIMIT_BOT:
            limitSet[0] = false;
            ESP_LOGI(TAG, "Bottom limit cleared");
            break;
            
        case ActuatorCmdAction::CLEAR_LIMIT_MID:
            limitSet[1] = false;
            ESP_LOGI(TAG, "Middle limit cleared");
            break;
            
        case ActuatorCmdAction::CLEAR_LIMIT_TOP:
            limitSet[2] = false;
            ESP_LOGI(TAG, "Top limit cleared");
            break;
            
        case ActuatorCmdAction::GET_LIMITS:
            // Telemetry will include limit data automatically
            break;
    }
}

void ActuatorNode::hwUpdate() {
    // Non-blocking movement evaluation with float-based ramping
    if (currentPercent < targetPercent) {
        currentPercent += pctPerTick;
        if (currentPercent > targetPercent) {
            currentPercent = targetPercent;  // Clamp exact arrival
        }
        HBridge_Set(ACT_FORWARD);
    } else if (currentPercent > targetPercent) {
        currentPercent -= pctPerTick;
        if (currentPercent < targetPercent) {
            currentPercent = targetPercent;  // Clamp exact arrival
        }
        HBridge_Set(ACT_REVERSE);
    } else {
        HBridge_Set(ACT_STOP);
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
    bool result = sendCommand(cmd);
    
    if (result && preferences.begin("peach", false)) {
        preferences.putInt("actB", limits[0]);
        preferences.putBool("actS_B", limitSet[0]);
        preferences.end();
        ESP_LOGI(TAG, "Saved bottom limit to NVS: %d", limits[0]);
    }
    
    return result;
}

bool ActuatorNode::setLimitMid(int percent) {
    ActuatorCommand cmd;
    cmd.action = ActuatorCmdAction::SET_LIMIT_MID;
    cmd.value = percent;
    bool result = sendCommand(cmd);
    
    if (result && preferences.begin("peach", false)) {
        preferences.putInt("actM", limits[1]);
        preferences.putBool("actS_M", limitSet[1]);
        preferences.end();
        ESP_LOGI(TAG, "Saved middle limit to NVS: %d", limits[1]);
    }
    
    return result;
}

bool ActuatorNode::setLimitTop(int percent) {
    ActuatorCommand cmd;
    cmd.action = ActuatorCmdAction::SET_LIMIT_TOP;
    cmd.value = percent;
    bool result = sendCommand(cmd);
    
    if (result && preferences.begin("peach", false)) {
        preferences.putInt("actT", limits[2]);
        preferences.putBool("actS_T", limitSet[2]);
        preferences.end();
        ESP_LOGI(TAG, "Saved top limit to NVS: %d", limits[2]);
    }
    
    return result;
}

bool ActuatorNode::clearLimitBot() {
    ActuatorCommand cmd;
    cmd.action = ActuatorCmdAction::CLEAR_LIMIT_BOT;
    cmd.value = 0;
    bool result = sendCommand(cmd);
    
    if (result && preferences.begin("peach", false)) {
        preferences.putBool("actS_B", false);
        preferences.end();
        ESP_LOGI(TAG, "Cleared bottom limit in NVS");
    }
    
    return result;
}

bool ActuatorNode::clearLimitMid() {
    ActuatorCommand cmd;
    cmd.action = ActuatorCmdAction::CLEAR_LIMIT_MID;
    cmd.value = 0;
    bool result = sendCommand(cmd);
    
    if (result && preferences.begin("peach", false)) {
        preferences.putBool("actS_M", false);
        preferences.end();
        ESP_LOGI(TAG, "Cleared middle limit in NVS");
    }
    
    return result;
}

bool ActuatorNode::clearLimitTop() {
    ActuatorCommand cmd;
    cmd.action = ActuatorCmdAction::CLEAR_LIMIT_TOP;
    cmd.value = 0;
    bool result = sendCommand(cmd);
    
    if (result && preferences.begin("peach", false)) {
        preferences.putBool("actS_T", false);
        preferences.end();
        ESP_LOGI(TAG, "Cleared top limit in NVS");
    }
    
    return result;
}
