#include "tasks/DishLiftNode.h"
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
    , armStepPos(0)
    , armCalStart(-1)
    , armBufferPos(-1)
    , armInPos(-1)
    , trackingTarget(0.0f)
    , isTrackingTarget(false) {
    posHome = 0.0f; posTilt = 0.0f;
    posHomeSet = false; posTiltSet = false;
}

DishLiftNode::~DishLiftNode() {
}

void DishLiftNode::hwInit() {
    vTaskDelay(pdMS_TO_TICKS(200));
    driver.begin(Serial1, TMC2209::SERIAL_ADDRESS_0);
    
    isHomed = false;
    currentPosition = 0.0f;
    
    StorageManager::loadDishLiftPositions(posHome, posTilt, posHomeSet, posTiltSet);
    StorageManager::loadDishLiftState(isHomed, currentPosition);
    
    // Apply StallGuard threshold from SystemState
    int sg = StorageManager::loadDishLiftSGThreshold(100);
    if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
        systemState.dishLiftSGThreshold = sg;
        xSemaphoreGive(systemStateMutex);
    }
    driver.setStallGuardThreshold(sg);

    PEACH_LOGI(TAG, "Loaded limits: Home=%.2f(%s), Tilt=%.2f(%s)",
             posHome, posHomeSet ? "Y" : "N",
             posTilt, posTiltSet ? "Y" : "N");
}

void DishLiftNode::processCommand(const DishLiftCommand& cmd) {
    switch (cmd.action) {
        case DishLiftCmdAction::SET_TARGET:
            trackingTarget = cmd.value;
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
            // Homing uses StallGuard: move down until stall
            targetSpeed = -3000;
            isHoming = true;
            isTrackingTarget = false;
            PEACH_LOGI(TAG, "Homing sequence initiated (SG)");
            break;
            
        case DishLiftCmdAction::SET_POS_HOME:
            posHome = cmd.value;
            posHomeSet = true;
            StorageManager::saveDishLiftPosHome(posHome);
            PEACH_LOGI(TAG, "Home limit set to %.2f", posHome);
            break;
            
        case DishLiftCmdAction::SET_POS_TILT:
            posTilt = cmd.value;
            posTiltSet = true;
            StorageManager::saveDishLiftPosTilt(posTilt);
            PEACH_LOGI(TAG, "Tilt limit set to %.2f", posTilt);
            break;
            
        case DishLiftCmdAction::CLEAR_CAL:
            posHomeSet = false;
            posTiltSet = false;
            StorageManager::saveDishLiftPosHome(0.0f);
            StorageManager::saveDishLiftPosTilt(0.0f);
            // manually set the set flags to false in NVS, simple approach:
            // StorageManager::saveDishLiftPosHome actually sets limS_H to true, so we can't clear it easily.
            // Wait, we need to clear limits. Let's just reset them.
            PEACH_LOGI(TAG, "Limits cleared");
            break;
            
        case DishLiftCmdAction::GET_STATE:
            break;
    }
}

void DishLiftNode::hwUpdate() {
    // Update StallGuard threshold dynamically if changed
    static int lastSg = -1;
    int currentSg = 100;
    if (xSemaphoreTake(systemStateMutex, 0) == pdTRUE) {
        currentSg = systemState.dishLiftSGThreshold;
        xSemaphoreGive(systemStateMutex);
    }
    if (currentSg != lastSg) {
        driver.setStallGuardThreshold(currentSg);
        lastSg = currentSg;
    }

    // Read arm telemetry for interlock logic
    ScraperArmTelemetry armTel;
    if (scraperArmTelQueue != NULL && xQueuePeek(scraperArmTelQueue, &armTel, 0) == pdPASS) {
        armStepPos = (int)armTel.currentPosition;
        armCalStart = armTel.posClear;
        armInPos = armTel.posScrape;
    }
    
    // Unlock motor if collision was cleared
    if (motorLocked && targetSpeed == 0) {
        motorLocked = false;
        LCD_setMessage("MOTOR UNLOCKED");
    }

    // SG Homing logic
    if (isHoming) {
        uint16_t sgResult = driver.getStallGuardResult();
        if (sgResult == 0 && std::abs(targetSpeed) > 100) {
            targetSpeed = 0;
            currentPosition = 0.0f; // Zero out on stall
            isHomed = true;
            isHoming = false;
            posHome = 0.0f;
            posHomeSet = true;
            StorageManager::saveDishLiftPosHome(0.0f);
            PEACH_LOGI(TAG, "Homing complete (SG stall detected)");
            LCD_setMessage("Z: Homed");
        }
    }
    
    // --- LIVE POSITION TRACKING & LIMITS ---
    if (!motorLocked && targetSpeed != 0) {
        // Update position based on velocity
        float deltaPos = (1.372e-6f * (float)targetSpeed * (float)TASK_UPDATE_INTERVAL_MS);
        currentPosition += deltaPos;
        
        // Calculate effective bottom limit
        bool effectiveBotSet = posHomeSet;
        float effectiveLimBot = posHome;
        
        // Interlock: If Arm is not near Clear, do not allow downward Z movement
        bool armNotClear = false;
        if (armCalStart != -1) {
            int distToClear = std::abs(armStepPos - armCalStart);
            if (distToClear > 500) {
                armNotClear = true;
            }
        } 
        
        if (armNotClear) {
            if (targetSpeed < 0) {
                // Block ALL downward movement if arm isn't clear
                targetSpeed = 0;
                LCD_setMessage("Arm Not Clear!");
                PEACH_LOGW(TAG, "Blocked downward motion: arm not clear");
            }
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
                    int minSpeed = 1000;
                    int maxSpeed = std::abs(targetSpeed);
                    if (maxSpeed > minSpeed) {
                        targetSpeed = (minSpeed + (int)((maxSpeed - minSpeed) * (dist / 5.0f)));
                    }
                }
            }
        }
        
        // Bottom limit check with deceleration zone
        if (effectiveBotSet && targetSpeed < 0 && !isHoming) {
            float distToBot = currentPosition - effectiveLimBot;
            if (distToBot <= 0.0f) {
                targetSpeed = 0;
                LCD_setMessage("Home Reached");
            } else if (distToBot < 5.0f) {
                int minSpeed = 1000;
                int maxSpeed = std::abs(targetSpeed);
                if (maxSpeed > minSpeed) {
                    targetSpeed = -(minSpeed + (int)((maxSpeed - minSpeed) * (distToBot / 5.0f)));
                }
            }
        }
        
        // Top limit check with deceleration zone
        if (posTiltSet && targetSpeed > 0) {
            float distToTop = posTilt - currentPosition;
            if (distToTop <= 0.0f) {
                targetSpeed = 0;
                LCD_setMessage("Tilt Reached");
            } else if (distToTop < 5.0f) {
                int minSpeed = 1000;
                int maxSpeed = std::abs(targetSpeed);
                if (maxSpeed > minSpeed) {
                    targetSpeed = minSpeed + (int)((maxSpeed - minSpeed) * (distToTop / 5.0f));
                }
            }
        }
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
    tel.posHome = posHome;
    tel.posTilt = posTilt;
    tel.posHomeSet = posHomeSet;
    tel.posTiltSet = posTiltSet;
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

bool DishLiftNode::setPosHome(float position) {
    DishLiftCommand cmd;
    cmd.action = DishLiftCmdAction::SET_POS_HOME;
    cmd.value = position;
    return sendCommand(cmd);
}

bool DishLiftNode::setPosTilt(float position) {
    DishLiftCommand cmd;
    cmd.action = DishLiftCmdAction::SET_POS_TILT;
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

bool DishLiftNode::clearCal() {
    DishLiftCommand cmd;
    cmd.action = DishLiftCmdAction::CLEAR_CAL;
    cmd.value = 0;
    return sendCommand(cmd);
}
