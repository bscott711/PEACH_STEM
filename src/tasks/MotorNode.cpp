#include "tasks/MotorNode.h"
#include "drivers/LCDDriver.h"
#include <esp_log.h>

static const char* TAG = "MOTOR_NODE";

// External queue handles for reading servo telemetry (for interlock)
extern QueueHandle_t servoTelQueue;

MotorNode::MotorNode()
    : currentPosition(0.0f)
    , targetSpeed(0)
    , isHomed(false)
    , isHoming(false)
    , collisionDetected(false)
    , motorLocked(false)
    , sgThreshold(16)
    , homingState(H_IDLE)
    , homingStartTime(0)
    , servoPercent(0)
    , servoCalStart(-1) {
    limits[0] = 0.0f; limits[1] = 0.0f; limits[2] = 0.0f;
    limitSet[0] = false; limitSet[1] = false; limitSet[2] = false;
}

MotorNode::~MotorNode() {
    preferences.end();
}

void MotorNode::hwInit() {
    // Initialize hardware pins and TMC2209 driver
    pinMode(DIAG_PIN, INPUT_PULLDOWN);
    Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
    vTaskDelay(pdMS_TO_TICKS(200));
    driver.begin(Serial1, TMC2209::SERIAL_ADDRESS_0);
    
    // Always require re-homing on boot (clears stale NVS homing data)
    isHomed = false;
    currentPosition = 0.0f;
    
    // Open NVS namespace for limit storage
    if (!preferences.begin("peach", false)) {
        ESP_LOGE(TAG, "Failed to open NVS namespace");
    } else {
        // Load limit data from NVS
        limits[0] = preferences.getFloat("limB", 0.0f);
        limits[1] = preferences.getFloat("limM", 0.0f);
        limits[2] = preferences.getFloat("limT", 0.0f);
        limitSet[0] = preferences.getBool("limS_B", false);
        limitSet[1] = preferences.getBool("limS_M", false);
        limitSet[2] = preferences.getBool("limS_T", false);
        
        ESP_LOGI(TAG, "Loaded limits: Bot=%.2f(%s), Mid=%.2f(%s), Top=%.2f(%s)",
                 limits[0], limitSet[0] ? "Y" : "N",
                 limits[1], limitSet[1] ? "Y" : "N",
                 limits[2], limitSet[2] ? "Y" : "N");
    }
}

void MotorNode::processCommand(const MotorCommand& cmd) {
    switch (cmd.action) {
        case MotorCmdAction::SET_SPEED:
            targetSpeed = (int)cmd.value;
            ESP_LOGD(TAG, "Set speed: %d", targetSpeed);
            break;
            
        case MotorCmdAction::START_HOMING:
            if (homingState == H_IDLE && !motorLocked) {
                homingState = H_MOVING;
                isHoming = true;
                ESP_LOGI(TAG, "Homing sequence initiated");
            }
            break;
            
        case MotorCmdAction::SET_LIMIT_BOT:
            limits[0] = cmd.value;
            limitSet[0] = true;
            ESP_LOGI(TAG, "Bottom limit set to %.2f", limits[0]);
            break;
            
        case MotorCmdAction::SET_LIMIT_MID:
            limits[1] = cmd.value;
            limitSet[1] = true;
            ESP_LOGI(TAG, "Middle limit set to %.2f", limits[1]);
            break;
            
        case MotorCmdAction::SET_LIMIT_TOP:
            limits[2] = cmd.value;
            limitSet[2] = true;
            ESP_LOGI(TAG, "Top limit set to %.2f", limits[2]);
            break;
            
        case MotorCmdAction::CLEAR_LIMIT_BOT:
            limitSet[0] = false;
            ESP_LOGI(TAG, "Bottom limit cleared");
            break;
            
        case MotorCmdAction::CLEAR_LIMIT_MID:
            limitSet[1] = false;
            ESP_LOGI(TAG, "Middle limit cleared");
            break;
            
        case MotorCmdAction::CLEAR_LIMIT_TOP:
            limitSet[2] = false;
            ESP_LOGI(TAG, "Top limit cleared");
            break;
            
        case MotorCmdAction::SET_SG_THRESHOLD:
            sgThreshold = (int)cmd.value;
            driver.updateSGThreshold(sgThreshold);
            ESP_LOGI(TAG, "SG threshold updated to %d", sgThreshold);
            break;
            
        case MotorCmdAction::GET_STATE:
            // Telemetry will include state automatically
            break;
    }
}

void MotorNode::hwUpdate() {
    // Read servo telemetry for interlock logic
    ServoTelemetry servoTel;
    if (xQueuePeek(servoTelQueue, &servoTel, 0) == pdPASS) {
        servoPercent = (int)servoTel.currentPercent;
        servoCalStart = servoTel.calStart;
    }
    
    // Unlock motor if collision was cleared
    if (motorLocked && targetSpeed == 0) {
        motorLocked = false;
        collisionDetected = false;
        LCD_setMessage("MOTOR UNLOCKED");
        ESP_LOGI(TAG, "Motor unlocked after collision clear");
    }
    
    // --- HOMING STATE MACHINE ---
    if (homingState != H_IDLE) {
        switch (homingState) {
            case H_MOVING:
                driver.setupHoming();
                driver.setVelocity(-20000);
                homingStartTime = xTaskGetTickCount();
                homingState = H_BLIND_WAIT;
                break;
                
            case H_BLIND_WAIT:
                // Wait 1 second before listening to avoid static friction spike
                if (xTaskGetTickCount() - homingStartTime >= pdMS_TO_TICKS(1000)) {
                    ESP_LOGI(TAG, "Listening to DIAG pin");
                    homingState = H_POLLING;
                }
                break;
                
            case H_POLLING:
                // Check for collision OR timeout (5 seconds)
                if (digitalRead(DIAG_PIN) == HIGH) {
                    driver.setVelocity(0);
                    ESP_LOGI(TAG, "Homing complete!");
                    
                    driver.finishHoming(sgThreshold);
                    
                    currentPosition = 0.0f;
                    isHomed = true;
                    isHoming = false;
                    targetSpeed = 0;
                    homingState = H_IDLE;
                    
                    // Save homing state to NVS
                    if (preferences.begin("peach", false)) {
                        preferences.putBool("isHomed", true);
                        preferences.putFloat("pos", 0.0f);
                        preferences.end();
                    }
                } else if (xTaskGetTickCount() - homingStartTime > pdMS_TO_TICKS(5000)) {
                    ESP_LOGE(TAG, "Homing timeout - aborting");
                    LCD_setMessage("Homing: TIMEOUT");
                    
                    driver.setVelocity(0);
                    driver.finishHoming(sgThreshold);
                    
                    isHoming = false;
                    targetSpeed = 0;
                    homingState = H_IDLE;
                }
                break;
                
            default:
                break;
        }
        return;  // Skip normal operation during homing
    }
    
    // --- LIVE POSITION TRACKING & LIMITS ---
    if (!motorLocked && targetSpeed != 0) {
        // Update position based on velocity
        float deltaPos = (1.372e-6f * (float)targetSpeed * (float)TASK_UPDATE_INTERVAL_MS);
        currentPosition += deltaPos;
        
        // Calculate effective bottom limit (interlock with servo position)
        bool effectiveBotSet = limitSet[0];
        float effectiveLimBot = limits[0];
        bool swungOut = (servoCalStart != -1) && (abs(servoPercent - servoCalStart) > 5);
        
        if (swungOut) {
            if (limitSet[1]) {
                effectiveBotSet = true;
                effectiveLimBot = limits[1];
            } else if (targetSpeed < 0) {
                // Block ALL downward movement if swung out and Mid isn't set
                targetSpeed = 0;
                LCD_setMessage("Servo Out: Mid Not Set!");
                ESP_LOGW(TAG, "Blocked downward motion: servo out, mid limit not set");
            }
        }
        
        // Bottom limit check with deceleration zone
        if (effectiveBotSet && targetSpeed < 0) {
            float distToBot = currentPosition - effectiveLimBot;
            if (distToBot <= 0.0f) {
                targetSpeed = 0;
                LCD_setMessage("Bottom Reached");
            } else if (distToBot < 5.0f) {
                // Deceleration zone: taper speed linearly
                int minSpeed = 1000;
                int maxSpeed = abs(targetSpeed);
                if (maxSpeed > minSpeed) {
                    int scaledSpeed = minSpeed + (int)((maxSpeed - minSpeed) * (distToBot / 5.0f));
                    targetSpeed = -scaledSpeed;
                }
            }
        }
        
        // Top limit check
        if (limitSet[2] && currentPosition >= limits[2] && targetSpeed > 0) {
            targetSpeed = 0;
            LCD_setMessage("Top Reached");
        }
        
        // Home position hard stop
        if (isHomed && currentPosition <= 0.0f && targetSpeed < 0) {
            targetSpeed = 0;
        }
    }
    
    // Apply speed command to driver
    if (motorLocked) {
        driver.stop();
    } else {
        driver.setVelocity(targetSpeed);
    }
    
    // Save state when stopped and homed
    if (targetSpeed == 0 && isHomed) {
        if (preferences.begin("peach", false)) {
            preferences.putBool("isHomed", isHomed);
            preferences.putFloat("pos", currentPosition);
            preferences.end();
        }
    }
}

MotorTelemetry MotorNode::generateTelemetry() {
    MotorTelemetry tel;
    tel.currentPosition = currentPosition;
    tel.targetSpeed = targetSpeed;
    tel.isHomed = isHomed;
    tel.isHoming = isHoming;
    tel.limits[0] = limits[0];
    tel.limits[1] = limits[1];
    tel.limits[2] = limits[2];
    tel.limitSet[0] = limitSet[0];
    tel.limitSet[1] = limitSet[1];
    tel.limitSet[2] = limitSet[2];
    tel.sgThreshold = sgThreshold;
    tel.collisionDetected = collisionDetected || motorLocked;
    return tel;
}

bool MotorNode::setSpeed(int speed) {
    MotorCommand cmd;
    cmd.action = MotorCmdAction::SET_SPEED;
    cmd.value = (float)speed;
    return sendCommand(cmd);
}

bool MotorNode::startHoming() {
    MotorCommand cmd;
    cmd.action = MotorCmdAction::START_HOMING;
    cmd.value = 0;
    return sendCommand(cmd);
}

bool MotorNode::setLimitBot(float position) {
    MotorCommand cmd;
    cmd.action = MotorCmdAction::SET_LIMIT_BOT;
    cmd.value = position;
    bool result = sendCommand(cmd);
    
    if (result && preferences.begin("peach", false)) {
        preferences.putFloat("limB", limits[0]);
        preferences.putBool("limS_B", limitSet[0]);
        preferences.end();
        ESP_LOGI(TAG, "Saved bottom limit to NVS: %.2f", limits[0]);
    }
    
    return result;
}

bool MotorNode::setLimitMid(float position) {
    MotorCommand cmd;
    cmd.action = MotorCmdAction::SET_LIMIT_MID;
    cmd.value = position;
    bool result = sendCommand(cmd);
    
    if (result && preferences.begin("peach", false)) {
        preferences.putFloat("limM", limits[1]);
        preferences.putBool("limS_M", limitSet[1]);
        preferences.end();
        ESP_LOGI(TAG, "Saved middle limit to NVS: %.2f", limits[1]);
    }
    
    return result;
}

bool MotorNode::setLimitTop(float position) {
    MotorCommand cmd;
    cmd.action = MotorCmdAction::SET_LIMIT_TOP;
    cmd.value = position;
    bool result = sendCommand(cmd);
    
    if (result && preferences.begin("peach", false)) {
        preferences.putFloat("limT", limits[2]);
        preferences.putBool("limS_T", limitSet[2]);
        preferences.end();
        ESP_LOGI(TAG, "Saved top limit to NVS: %.2f", limits[2]);
    }
    
    return result;
}

bool MotorNode::clearLimitBot() {
    MotorCommand cmd;
    cmd.action = MotorCmdAction::CLEAR_LIMIT_BOT;
    cmd.value = 0;
    bool result = sendCommand(cmd);
    
    if (result && preferences.begin("peach", false)) {
        preferences.putBool("limS_B", false);
        preferences.end();
        ESP_LOGI(TAG, "Cleared bottom limit in NVS");
    }
    
    return result;
}

bool MotorNode::clearLimitMid() {
    MotorCommand cmd;
    cmd.action = MotorCmdAction::CLEAR_LIMIT_MID;
    cmd.value = 0;
    bool result = sendCommand(cmd);
    
    if (result && preferences.begin("peach", false)) {
        preferences.putBool("limS_M", false);
        preferences.end();
        ESP_LOGI(TAG, "Cleared middle limit in NVS");
    }
    
    return result;
}

bool MotorNode::clearLimitTop() {
    MotorCommand cmd;
    cmd.action = MotorCmdAction::CLEAR_LIMIT_TOP;
    cmd.value = 0;
    bool result = sendCommand(cmd);
    
    if (result && preferences.begin("peach", false)) {
        preferences.putBool("limS_T", false);
        preferences.end();
        ESP_LOGI(TAG, "Cleared top limit in NVS");
    }
    
    return result;
}

bool MotorNode::setSGThreshold(int threshold) {
    MotorCommand cmd;
    cmd.action = MotorCmdAction::SET_SG_THRESHOLD;
    cmd.value = (float)threshold;
    return sendCommand(cmd);
}
