#include "tasks/MotorNode.h"
#include "drivers/LCDDriver.h"
#include "controller.h"
#include <esp_log.h>
#include <cmath>

static const char* TAG = "MOTOR_NODE";

// External queue handles for reading arm telemetry (for interlock)
extern QueueHandle_t armTelQueue;

MotorNode::MotorNode()
    : currentPosition(0.0f)
    , targetSpeed(0)
    , previousTargetSpeed(0)
    , isHomed(false)
    , isHoming(false)
    , motorLocked(false)
    , topEndstopTriggered(false)
    , botEndstopTriggered(false)
    , homingState(H_IDLE)
    , homingStartTime(0)
    , armStepPos(0)
    , armCalStart(-1) {
    limits[0] = 0.0f; limits[1] = 0.0f; limits[2] = 0.0f;
    limitSet[0] = false; limitSet[1] = false; limitSet[2] = false;
}

MotorNode::~MotorNode() {
}

void MotorNode::hwInit() {
    // Initialize hardware pins
#if ENABLE_OPTICAL_ENDSTOPS
    pinMode(TOP_ENDSTOP_PIN, INPUT);
    pinMode(BOT_ENDSTOP_PIN, INPUT);
#endif

    // Initialize TMC2209 driver for Z-Axis on Address 0
    vTaskDelay(pdMS_TO_TICKS(200));
    driver.begin(Serial1, TMC2209::SERIAL_ADDRESS_0);
    
    // Always require re-homing on boot (clears stale NVS homing data)
    isHomed = false;
    currentPosition = 0.0f;
    
    StorageManager::loadMotorLimits(limits, limitSet);
    StorageManager::loadMotorState(isHomed, currentPosition);
    
    ESP_LOGI(TAG, "Loaded limits: Bot=%.2f(%s), Mid=%.2f(%s), Top=%.2f(%s)",
             limits[0], limitSet[0] ? "Y" : "N",
             limits[1], limitSet[1] ? "Y" : "N",
             limits[2], limitSet[2] ? "Y" : "N");
}

void MotorNode::processCommand(const MotorCommand& cmd) {
    switch (cmd.action) {
        case MotorCmdAction::SET_SPEED:
            targetSpeed = (int)cmd.value;
            ESP_LOGD(TAG, "Set speed: %d", targetSpeed);
            break;
            
        case MotorCmdAction::START_HOMING:
            if (homingState == H_IDLE && !motorLocked) {
                homingState = H_MOVING_TOP;
                isHoming = true;
                ESP_LOGI(TAG, "Homing sequence initiated");
            }
            break;
            
        case MotorCmdAction::SET_LIMIT_BOT:
            limits[0] = cmd.value;
            limitSet[0] = true;
            StorageManager::saveMotorLimit(StorageManager::LIMIT_BOT, limits[0], true);
            ESP_LOGI(TAG, "Bottom limit set to %.2f", limits[0]);
            break;
            
        case MotorCmdAction::SET_LIMIT_MID:
            limits[1] = cmd.value;
            limitSet[1] = true;
            StorageManager::saveMotorLimit(StorageManager::LIMIT_MID, limits[1], true);
            ESP_LOGI(TAG, "Middle limit set to %.2f", limits[1]);
            break;
            
        case MotorCmdAction::SET_LIMIT_TOP:
            currentPosition = 0.0f;
            limits[2] = 0.0f;
            limitSet[2] = true;
            StorageManager::saveMotorLimit(StorageManager::LIMIT_TOP, limits[2], true);
            ESP_LOGI(TAG, "Top limit set to 0 and position zeroed");
            break;
            
        case MotorCmdAction::CLEAR_LIMIT_BOT:
            limitSet[0] = false;
            StorageManager::saveMotorLimit(StorageManager::LIMIT_BOT, limits[0], false);
            ESP_LOGI(TAG, "Bottom limit cleared");
            break;
            
        case MotorCmdAction::CLEAR_LIMIT_MID:
            limitSet[1] = false;
            StorageManager::saveMotorLimit(StorageManager::LIMIT_MID, limits[1], false);
            ESP_LOGI(TAG, "Middle limit cleared");
            break;
            
        case MotorCmdAction::CLEAR_LIMIT_TOP:
            limitSet[2] = false;
            StorageManager::saveMotorLimit(StorageManager::LIMIT_TOP, limits[2], false);
            ESP_LOGI(TAG, "Top limit cleared");
            break;
            
        case MotorCmdAction::GET_STATE:
            // Telemetry will include state automatically
            break;
    }
}

void MotorNode::hwUpdate() {
    // Read arm telemetry for interlock logic
    ArmTelemetry armTel;
    if (armTelQueue != NULL && xQueuePeek(armTelQueue, &armTel, 0) == pdPASS) {
        armStepPos = (int)armTel.currentPosition;
        armCalStart = armTel.posOut;  // Use posOut as the "start" position for interlock
        armBufferPos = armTel.posBuffer;
        armInPos = armTel.posIn;
    }
    
    // Unlock motor if collision was cleared (not applicable anymore but kept for safety)
    if (motorLocked && targetSpeed == 0) {
        motorLocked = false;
        LCD_setMessage("MOTOR UNLOCKED");
        ESP_LOGI(TAG, "Motor unlocked");
    }

#if ENABLE_OPTICAL_ENDSTOPS
    topEndstopTriggered = (digitalRead(TOP_ENDSTOP_PIN) == HIGH);
    botEndstopTriggered = (digitalRead(BOT_ENDSTOP_PIN) == HIGH);
#else
    topEndstopTriggered = false;
    botEndstopTriggered = false;
#endif

    // --- OPTICAL HOMING STATE MACHINE ---
    if (homingState != H_IDLE) {
        switch (homingState) {
            case H_MOVING_TOP:
                // Move towards TOP endstop (positive velocity)
                driver.setVelocity(20000);
                homingStartTime = xTaskGetTickCount();
                homingState = H_BACKOFF;
                break;
                
            case H_BACKOFF:
                if (topEndstopTriggered) {
                    driver.setVelocity(0);
                    ESP_LOGI(TAG, "Homing complete (Top triggered)!");
                    
                    currentPosition = 0.0f; // Top is 0 or Max Limit depending on configuration.
                    // Assuming Top is 0 for clearance.
                    isHomed = true;
                    isHoming = false;
                    targetSpeed = 0;
                    homingState = H_IDLE;
                    
                    StorageManager::saveMotorState(true, 0.0f);
                } else if (xTaskGetTickCount() - homingStartTime > pdMS_TO_TICKS(15000)) {
                    ESP_LOGE(TAG, "Homing timeout");
                    LCD_setMessage("Homing: TIMEOUT");
                    driver.setVelocity(0);
                    isHoming = false;
                    targetSpeed = 0;
                    homingState = H_IDLE;
                }
                break;
                
            default:
                break;
        }
#if ENABLE_OPTICAL_ENDSTOPS
        return; // Skip normal operation during homing
#else
        // If endstops are disabled, simulate instant homing
        driver.setVelocity(0);
        isHomed = true;
        isHoming = false;
        targetSpeed = 0;
        homingState = H_IDLE;
#endif
    }
    
    // --- LIVE POSITION TRACKING & LIMITS ---
    if (!motorLocked && targetSpeed != 0) {
        // Update position based on velocity
        float deltaPos = (1.372e-6f * (float)targetSpeed * (float)TASK_UPDATE_INTERVAL_MS);
        currentPosition += deltaPos;
        
        // Calculate effective bottom limit
        bool effectiveBotSet = limitSet[0];
        float effectiveLimBot = limits[0];
        
        // Interlock Option A: If Arm is between Buffer and Tip, block going below Mid.
        bool swungOut = false;
        if (armCalStart != -1 && armBufferPos != -1) {
            int distToCurrent = std::abs(armStepPos - armCalStart);
            int distToBuffer = std::abs(armBufferPos - armCalStart);
            if (distToCurrent > distToBuffer + 5) {
                swungOut = true;
            }
        } else {
            // Fallback if Buffer is not set, use old 500 step rule
            swungOut = (armCalStart != -1) && (std::abs(armStepPos - armCalStart) > 500); 
        } 
        bool usingMidAsBot = false;
        
        if (swungOut) {
            if (limitSet[1]) {
                effectiveBotSet = true;
                effectiveLimBot = limits[1];
                usingMidAsBot = true;
            } else if (targetSpeed < 0) {
                // Block ALL downward movement if swung out and Mid isn't set
                targetSpeed = 0;
                LCD_setMessage("Arm Out: Mid Not Set");
                ESP_LOGW(TAG, "Blocked downward motion: arm out, mid limit not set");
            }
        }
        
        // Bottom limit check with deceleration zone
        if (effectiveBotSet && targetSpeed < 0) {
            float distToBot = currentPosition - effectiveLimBot;
            if (distToBot <= 0.0f) {
                targetSpeed = 0;
                if (usingMidAsBot) {
                    LCD_setMessage("Mid Reached(ArmOut)");
                } else {
                    LCD_setMessage("Bottom Reached");
                }
            } else if (distToBot < 5.0f) {
                // Deceleration zone: taper speed linearly
                int minSpeed = 1000;
                int maxSpeed = std::abs(targetSpeed);
                if (maxSpeed > minSpeed) {
                    int scaledSpeed = minSpeed + (int)((maxSpeed - minSpeed) * (distToBot / 5.0f));
                    targetSpeed = -scaledSpeed;
                }
            }
        }
        
        // Top limit check with deceleration zone
        if (limitSet[2] && targetSpeed > 0) {
            float distToTop = limits[2] - currentPosition;
            if (distToTop <= 0.0f) {
                targetSpeed = 0;
                LCD_setMessage("Top Reached");
            } else if (distToTop < 5.0f) {
                int minSpeed = 1000;
                int maxSpeed = std::abs(targetSpeed);
                if (maxSpeed > minSpeed) {
                    int scaledSpeed = minSpeed + (int)((maxSpeed - minSpeed) * (distToTop / 5.0f));
                    targetSpeed = scaledSpeed;
                }
            }
        }
        
        // Home position hard stop checks removed as we now rely on optical limit switches
    }
    
    // Hardware Endstop Overrides
    if (topEndstopTriggered && targetSpeed > 0) {
        targetSpeed = 0;
        LCD_setMessage("TOP ENDSTOP!");
    }
    if (botEndstopTriggered && targetSpeed < 0) {
        targetSpeed = 0;
        LCD_setMessage("BOT ENDSTOP!");
    }
    
    // Apply speed command to driver
    if (motorLocked) {
        driver.stop();
    } else {
        driver.setVelocity(targetSpeed);
    }
    
    // Save state when stopped and homed
    if (targetSpeed == 0 && previousTargetSpeed != 0) {
        if (isHomed) {
            StorageManager::saveMotorState(isHomed, currentPosition);
        }
        xEventGroupSetBits(controlEvents, BIT_POS_REACHED_Z);
    }
    
    previousTargetSpeed = targetSpeed;
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
    tel.topEndstopTriggered = topEndstopTriggered;
    tel.botEndstopTriggered = botEndstopTriggered;
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
    return sendCommand(cmd);
}

bool MotorNode::setLimitMid(float position) {
    MotorCommand cmd;
    cmd.action = MotorCmdAction::SET_LIMIT_MID;
    cmd.value = position;
    return sendCommand(cmd);
}

bool MotorNode::setLimitTop(float position) {
    MotorCommand cmd;
    cmd.action = MotorCmdAction::SET_LIMIT_TOP;
    cmd.value = position;
    return sendCommand(cmd);
}

bool MotorNode::clearLimitBot() {
    MotorCommand cmd;
    cmd.action = MotorCmdAction::CLEAR_LIMIT_BOT;
    cmd.value = 0;
    return sendCommand(cmd);
}

bool MotorNode::clearLimitMid() {
    MotorCommand cmd;
    cmd.action = MotorCmdAction::CLEAR_LIMIT_MID;
    cmd.value = 0;
    return sendCommand(cmd);
}

bool MotorNode::clearLimitTop() {
    MotorCommand cmd;
    cmd.action = MotorCmdAction::CLEAR_LIMIT_TOP;
    cmd.value = 0;
    return sendCommand(cmd);
}
