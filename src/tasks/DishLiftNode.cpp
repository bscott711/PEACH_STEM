#include "DishLiftNode.h"
#include "drivers/LCDDriver.h"
#include "controller.h"
#include "core/NetworkManager.h"
#include <cmath>

static const char* TAG = "LIFT_NODE";

// External queue handles for reading arm telemetry (for interlock)
extern QueueHandle_t scraperArmTelQueue;

DishLiftNode::DishLiftNode()
    : currentPosition(0.0f)
    , targetSpeed(0)
    , previousTargetSpeed(0)
    , isHomed(false)
    , isHoming(false)
    , motorLocked(false)
    , homingState(H_IDLE)
    , homingStartTime(0)
    , armStepPos(0)
    , armCalStart(-1)
    , trackingTarget(0.0f)
    , isTrackingTarget(false) {
    limits[0] = 0.0f; limits[1] = 0.0f; limits[2] = 0.0f;
    limitSet[0] = false; limitSet[1] = false; limitSet[2] = false;
}

DishLiftNode::~DishLiftNode() {
}

void DishLiftNode::hwInit() {
    // Initialize TMC2209 driver for Z-Axis on Address 0
    vTaskDelay(pdMS_TO_TICKS(200));
    driver.begin(Serial1, TMC2209::SERIAL_ADDRESS_0);
    
    // Always require re-homing on boot (clears stale NVS homing data)
    isHomed = false;
    currentPosition = 0.0f;
    
    StorageManager::loadDishLiftLimits(limits, limitSet);
    StorageManager::loadDishLiftState(isHomed, currentPosition);
    
    PEACH_LOGI(TAG, "Loaded limits: Bot=%.2f(%s), Mid=%.2f(%s), Top=%.2f(%s)",
             limits[0], limitSet[0] ? "Y" : "N",
             limits[1], limitSet[1] ? "Y" : "N",
             limits[2], limitSet[2] ? "Y" : "N");
}

void DishLiftNode::processCommand(const DishLiftCommand& cmd) {
    switch (cmd.action) {
        case DishLiftCmdAction::SET_TARGET:
            trackingTarget = cmd.value;
            // Determine direction based on current position
            if (trackingTarget > currentPosition) {
                targetSpeed = cmd.targetSpeed;
            } else {
                targetSpeed = -cmd.targetSpeed;
            }
            isTrackingTarget = true;
            PEACH_LOGI(TAG, "GOTO target: %.2f at speed %d", trackingTarget, cmd.targetSpeed);
            break;
            
        case DishLiftCmdAction::SET_SPEED:
            targetSpeed = (int)cmd.value;
            isTrackingTarget = false;
            PEACH_LOGD(TAG, "Set speed: %d", targetSpeed);
            break;
            
        case DishLiftCmdAction::START_HOMING:
            if (homingState == H_IDLE && !motorLocked) {
                homingState = H_MOVING_TOP;
                isHoming = true;
                PEACH_LOGI(TAG, "Homing sequence initiated");
            }
            break;
            
        case DishLiftCmdAction::SET_LIMIT_BOT:
            limits[0] = cmd.value;
            limitSet[0] = true;
            StorageManager::saveDishLiftLimit(StorageManager::LIMIT_BOT, limits[0], true);
            PEACH_LOGI(TAG, "Bottom limit set to %.2f", limits[0]);
            break;
            
        case DishLiftCmdAction::SET_LIMIT_MID:
            limits[1] = cmd.value;
            limitSet[1] = true;
            StorageManager::saveDishLiftLimit(StorageManager::LIMIT_MID, limits[1], true);
            PEACH_LOGI(TAG, "Middle limit set to %.2f", limits[1]);
            break;
            
        case DishLiftCmdAction::SET_LIMIT_TOP:
            currentPosition = 0.0f;
            limits[2] = 0.0f;
            limitSet[2] = true;
            StorageManager::saveDishLiftLimit(StorageManager::LIMIT_TOP, limits[2], true);
            PEACH_LOGI(TAG, "Top limit set to 0 and position zeroed");
            break;
            
        case DishLiftCmdAction::CLEAR_LIMIT_BOT:
            limitSet[0] = false;
            StorageManager::saveDishLiftLimit(StorageManager::LIMIT_BOT, limits[0], false);
            PEACH_LOGI(TAG, "Bottom limit cleared");
            break;
            
        case DishLiftCmdAction::CLEAR_LIMIT_MID:
            limitSet[1] = false;
            StorageManager::saveDishLiftLimit(StorageManager::LIMIT_MID, limits[1], false);
            PEACH_LOGI(TAG, "Middle limit cleared");
            break;
            
        case DishLiftCmdAction::CLEAR_LIMIT_TOP:
            limitSet[2] = false;
            StorageManager::saveDishLiftLimit(StorageManager::LIMIT_TOP, limits[2], false);
            PEACH_LOGI(TAG, "Top limit cleared");
            break;
            
        case DishLiftCmdAction::GET_STATE:
            // Telemetry will include state automatically
            break;
    }
}

void DishLiftNode::hwUpdate() {
    // Read arm telemetry for interlock logic
    ScraperArmTelemetry armTel;
    if (scraperArmTelQueue != NULL && xQueuePeek(scraperArmTelQueue, &armTel, 0) == pdPASS) {
        armStepPos = (int)armTel.currentPosition;
        armCalStart = armTel.posOut;  // Use posOut as the "start" position for interlock
        armBufferPos = armTel.posBuffer;
        armInPos = armTel.posIn;
    }
    
    // Unlock motor if collision was cleared (not applicable anymore but kept for safety)
    if (motorLocked && targetSpeed == 0) {
        motorLocked = false;
        LCD_setMessage("MOTOR UNLOCKED");
        PEACH_LOGI(TAG, "Motor unlocked");
    }

    // --- HOMING STATE MACHINE ---
    if (homingState != H_IDLE) {
        // Endstops removed. Simulate instant homing for now until StallGuard is implemented.
        driver.setVelocity(0);
        isHomed = true;
        isHoming = false;
        targetSpeed = 0;
        homingState = H_IDLE;
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
                PEACH_LOGW(TAG, "Blocked downward motion: arm out, mid limit not set");
            }
        }
        
        // Interlock Option B: Don't allow swing in if Z is below Buffer
        if (targetSpeed > 0 && !swungOut) {
            // Nothing to interlock going UP, let it move
        }

        // Tracking Target logic (Overrides limits if tracking)
        if (isTrackingTarget) {
            if (targetSpeed < 0) { // Moving DOWN
                float dist = currentPosition - trackingTarget;
                if (dist <= 0.0f) {
                    targetSpeed = 0;
                    isTrackingTarget = false;
                    LCD_setMessage("Target Reached");
                } else if (dist < 5.0f) {
                    // Decel
                    int minSpeed = 1000;
                    int maxSpeed = std::abs(targetSpeed);
                    if (maxSpeed > minSpeed) {
                        targetSpeed = -(minSpeed + (int)((maxSpeed - minSpeed) * (dist / 5.0f)));
                    }
                }
            } else if (targetSpeed > 0) { // Moving UP
                float dist = trackingTarget - currentPosition;
                if (dist <= 0.0f) {
                    targetSpeed = 0;
                    isTrackingTarget = false;
                    LCD_setMessage("Target Reached");
                } else if (dist < 5.0f) {
                    // Decel
                    int minSpeed = 1000;
                    int maxSpeed = std::abs(targetSpeed);
                    if (maxSpeed > minSpeed) {
                        targetSpeed = (minSpeed + (int)((maxSpeed - minSpeed) * (dist / 5.0f)));
                    }
                }
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
        
        // Hard stop checks removed
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
            StorageManager::saveDishLiftState(isHomed, currentPosition);
        }
        xEventGroupSetBits(controlEvents, BIT_POS_REACHED_Z);
    }
    
    previousTargetSpeed = targetSpeed;
}

DishLiftTelemetry DishLiftNode::generateTelemetry() {
    DishLiftTelemetry tel;
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
    return tel;
}

bool DishLiftNode::setSpeed(int speed) {
    DishLiftCommand cmd;
    cmd.action = DishLiftCmdAction::SET_SPEED;
    cmd.value = (float)speed;
    return sendCommand(cmd);
}

bool DishLiftNode::startHoming() {
    DishLiftCommand cmd;
    cmd.action = DishLiftCmdAction::START_HOMING;
    cmd.value = 0;
    return sendCommand(cmd);
}

bool DishLiftNode::setLimitBot(float position) {
    DishLiftCommand cmd;
    cmd.action = DishLiftCmdAction::SET_LIMIT_BOT;
    cmd.value = position;
    return sendCommand(cmd);
}

bool DishLiftNode::setLimitMid(float position) {
    DishLiftCommand cmd;
    cmd.action = DishLiftCmdAction::SET_LIMIT_MID;
    cmd.value = position;
    return sendCommand(cmd);
}

bool DishLiftNode::setLimitTop(float position) {
    DishLiftCommand cmd;
    cmd.action = DishLiftCmdAction::SET_LIMIT_TOP;
    cmd.value = position;
    return sendCommand(cmd);
}

bool DishLiftNode::setTarget(float position, int speed) {
    DishLiftCommand cmd;
    cmd.action = DishLiftCmdAction::SET_TARGET;
    cmd.value = position;
    cmd.targetSpeed = speed;
    return sendCommand(cmd);
}

bool DishLiftNode::clearLimitBot() {
    DishLiftCommand cmd;
    cmd.action = DishLiftCmdAction::CLEAR_LIMIT_BOT;
    cmd.value = 0;
    return sendCommand(cmd);
}

bool DishLiftNode::clearLimitMid() {
    DishLiftCommand cmd;
    cmd.action = DishLiftCmdAction::CLEAR_LIMIT_MID;
    cmd.value = 0;
    return sendCommand(cmd);
}

bool DishLiftNode::clearLimitTop() {
    DishLiftCommand cmd;
    cmd.action = DishLiftCmdAction::CLEAR_LIMIT_TOP;
    cmd.value = 0;
    return sendCommand(cmd);
}
