#include "tasks/ScraperArmNode.h"
#include "controller.h"
#include "drivers/LCDDriver.h"
#include "core/NetworkManager.h"
#include <esp_log.h>
#include <cmath>

static const char* TAG = "SCRAPER_NODE";

ScraperArmNode::ScraperArmNode()
    : currentPosition(0.0f)
    , targetSpeed(0)
    , previousTargetSpeed(0)
    , posOut(-1)
    , posIn(-1)
    , isTrackingTarget(false)
    , targetTrackingAbsSteps(0.0f)
    , lastSavedPosition(-999.0f) {
}

ScraperArmNode::~ScraperArmNode() {
}

void ScraperArmNode::hwInit() {
    // Initialize TMC2209 driver for Arm on Address 1
    // Note: Serial1 is shared and initialized globally in main.cpp!
    vTaskDelay(pdMS_TO_TICKS(200));
    driver.begin(Serial1, TMC2209::SERIAL_ADDRESS_1);
    
    // Open NVS namespace
    StorageManager::loadScraperArmCalibration(posOut, posIn);
    posBuffer = StorageManager::loadScraperArmPosBuffer();
    float lastPos = StorageManager::loadScraperArmPosition();
    
    // Set stepper to last known position immediately
    currentPosition = lastPos;
    lastSavedPosition = lastPos;
    
    PEACH_LOGI(TAG, "Loaded calibration: Out=%d, Buf=%d, In=%d, Pos=%.1f", posOut, posBuffer, posIn, lastPos);
}

void ScraperArmNode::processCommand(const ScraperArmCommand& cmd) {
    switch (cmd.action) {
        case ScraperArmCmdAction::SET_SPEED:
            targetSpeed = (int)cmd.value;
            isTrackingTarget = false;
            PEACH_LOGD(TAG, "Arm set speed: %d", targetSpeed);
            break;
            
        case ScraperArmCmdAction::STOP:
            targetSpeed = 0;
            // Don't clear isTrackingTarget — if we were tracking, the 
            // P-controller in hwUpdate will resume. STOP is for jog mode only.
            if (!isTrackingTarget) {
                PEACH_LOGD(TAG, "Arm stopped");
            }
            break;
            
        case ScraperArmCmdAction::SET_TARGET:
            if (posOut != -1 && posIn != -1) {
                if (cmd.value == 200.0f && posBuffer != -1) {
                    targetTrackingAbsSteps = posBuffer;
                    PEACH_LOGI(TAG, "Arm tracking target: Buffer -> %.2f steps at speed %d", targetTrackingAbsSteps, cmd.targetSpeed);
                } else {
                    targetTrackingAbsSteps = posOut + (cmd.value / 100.0f) * (posIn - posOut);
                    PEACH_LOGI(TAG, "Arm tracking target: %.2f%% -> %.2f steps at speed %d", cmd.value, targetTrackingAbsSteps, cmd.targetSpeed);
                }
                targetTrackingSpeed = cmd.targetSpeed;
                isTrackingTarget = true;
            } else {
                PEACH_LOGW(TAG, "Arm SET_TARGET ignored: Uncalibrated!");
            }
            break;
            
        case ScraperArmCmdAction::JOG:
            if (!isTrackingTarget) {
                targetTrackingAbsSteps = currentPosition;
                isTrackingTarget = true;
            }
            targetTrackingAbsSteps += cmd.value;
            break;
            
        case ScraperArmCmdAction::SET_POS_OUT:
            currentPosition = 0.0f;
            posOut = 0;
            StorageManager::saveScraperArmPosOut(posOut);
            PEACH_LOGI(TAG, "Arm posOut set to 0 and position zeroed");
            break;
            
        case ScraperArmCmdAction::SET_POS_BUFFER:
            posBuffer = (int)currentPosition;
            StorageManager::saveScraperArmPosBuffer(posBuffer);
            PEACH_LOGI(TAG, "Arm posBuffer set to %d", posBuffer);
            break;
            
        case ScraperArmCmdAction::SET_POS_IN:
            posIn = (int)currentPosition;
            StorageManager::saveScraperArmPosIn(posIn);
            PEACH_LOGI(TAG, "Arm posIn set to %d", posIn);
            break;
            
        case ScraperArmCmdAction::CLEAR_CAL:
            posOut = -1;
            posBuffer = -1;
            posIn = -1;
            StorageManager::saveScraperArmPosOut(-1);
            StorageManager::saveScraperArmPosBuffer(-1);
            StorageManager::saveScraperArmPosIn(-1);
            PEACH_LOGI(TAG, "Arm calibration cleared");
            break;
    }
}

void ScraperArmNode::hwUpdate() {
    if (isTrackingTarget) {
        float error = targetTrackingAbsSteps - currentPosition;
        float Kp = 5.0f; // Proportional gain for deceleration ease-out
        float desiredSpeedFloat = error * Kp;
        int desiredSpeed = (int)constrain(desiredSpeedFloat, -targetTrackingSpeed, targetTrackingSpeed);
        
        // Slew-rate limiter for acceleration ease-in (1 second to reach full speed)
        int maxAccelPerTick = targetTrackingSpeed / 100;
        if (maxAccelPerTick < 10) maxAccelPerTick = 10;
        
        if (desiredSpeed > targetSpeed + maxAccelPerTick) {
            targetSpeed += maxAccelPerTick;
        } else if (desiredSpeed < targetSpeed - maxAccelPerTick) {
            targetSpeed -= maxAccelPerTick;
        } else {
            targetSpeed = desiredSpeed;
        }
        
        // Stop condition when firmly at target
        if (std::abs(error) < 2.0f && std::abs(targetSpeed) <= 10) {
            currentPosition = targetTrackingAbsSteps;
            targetSpeed = 0;
            isTrackingTarget = false;
        }
    }

    // Velocity control and position integration
    if (targetSpeed != 0) {
        // VACTUAL to steps/sec factor is ~0.715
        float stepsPerSec = (float)targetSpeed * 0.715f;
        float deltaPos = stepsPerSec * ((float)TASK_UPDATE_INTERVAL_MS / 1000.0f);
        currentPosition += deltaPos;
    }
    
    // Read Z motor telemetry for interlock logic
    DishLiftTelemetry motorTel;
    bool zAtTop = false;
    if (dishLiftTelQueue != NULL && xQueuePeek(dishLiftTelQueue, &motorTel, 0) == pdPASS) {
        if (motorTel.limitSet[2] && motorTel.currentPosition >= motorTel.limits[2] - 5.0f) {
            zAtTop = true;
        }
    }
    
    // Check if arm is in or trying to enter the collision zone (between buffer and tip)
    bool inCollisionZone = false;
    if (posIn != -1 && posBuffer != -1) {
        float minZone = min((float)posBuffer, (float)posIn);
        float maxZone = max((float)posBuffer, (float)posIn);
        if (currentPosition >= minZone && currentPosition <= maxZone) {
            inCollisionZone = true;
        }
    }
    
    // Hard Endstops
    if (posOut != -1 && posIn != -1 && targetSpeed != 0) {
        float minLim = min((float)posOut, (float)posIn);
        float maxLim = max((float)posOut, (float)posIn);
        
        if (currentPosition >= maxLim && targetSpeed > 0) {
            targetSpeed = 0;
            currentPosition = maxLim;
            isTrackingTarget = false;
            LCD_setMessage((posIn > posOut) ? "Arm Tip Reached" : "Arm Clear Reached");
        } else if (currentPosition <= minLim && targetSpeed < 0) {
            targetSpeed = 0;
            currentPosition = minLim;
            isTrackingTarget = false;
            LCD_setMessage((posIn < posOut) ? "Arm Tip Reached" : "Arm Clear Reached");
        }
    }
    
    // Interlock Check
    if (inCollisionZone && !zAtTop && targetSpeed != 0) {
        targetSpeed = 0;
        isTrackingTarget = false;
        LCD_setMessage("Interlock: Z Not Clear!");
    }
    
    driver.setVelocity(targetSpeed);
    
    // Save position to NVS when stopped and position has changed
    if (targetSpeed == 0 && previousTargetSpeed != 0) {
        if (std::abs(currentPosition - lastSavedPosition) > 0.1f) {
            StorageManager::saveScraperArmPosition(currentPosition);
            lastSavedPosition = currentPosition;
            PEACH_LOGI(TAG, "Saved arm position: %.2f", currentPosition);
        }
        xEventGroupSetBits(controlEvents, BIT_POS_REACHED_ARM);
    }
    previousTargetSpeed = targetSpeed;
}

ScraperArmTelemetry ScraperArmNode::generateTelemetry() {
    ScraperArmTelemetry tel;
    tel.currentPosition = currentPosition;
    tel.targetPosition = isTrackingTarget ? targetTrackingAbsSteps : currentPosition;
    tel.posOut = posOut;
    tel.posBuffer = posBuffer;
    tel.posIn = posIn;
    tel.isMoving = (targetSpeed != 0);
    return tel;
}

bool ScraperArmNode::setSpeed(int speed) {
    ScraperArmCommand cmd;
    cmd.action = ScraperArmCmdAction::SET_SPEED;
    cmd.value = (float)speed;
    return sendCommand(cmd);
}

bool ScraperArmNode::stop() {
    ScraperArmCommand cmd;
    cmd.action = ScraperArmCmdAction::STOP;
    cmd.value = 0.0f;
    return sendCommand(cmd);
}

bool ScraperArmNode::jog(float relativeSteps) {
    ScraperArmCommand cmd;
    cmd.action = ScraperArmCmdAction::JOG;
    cmd.value = relativeSteps;
    return sendCommand(cmd);
}

bool ScraperArmNode::setTarget(float percent, int targetSpeed) {
    ScraperArmCommand cmd;
    cmd.action = ScraperArmCmdAction::SET_TARGET;
    cmd.value = percent;
    cmd.targetSpeed = targetSpeed;
    return sendCommand(cmd);
}

bool ScraperArmNode::setPosOut() {
    ScraperArmCommand cmd;
    cmd.action = ScraperArmCmdAction::SET_POS_OUT;
    cmd.value = 0.0f;
    return sendCommand(cmd);
}

bool ScraperArmNode::setPosBuffer() {
    ScraperArmCommand cmd;
    cmd.action = ScraperArmCmdAction::SET_POS_BUFFER;
    cmd.value = 0.0f;
    return sendCommand(cmd);
}

bool ScraperArmNode::setPosIn() {
    ScraperArmCommand cmd;
    cmd.action = ScraperArmCmdAction::SET_POS_IN;
    cmd.value = 0.0f;
    return sendCommand(cmd);
}

bool ScraperArmNode::clearCal() {
    ScraperArmCommand cmd;
    cmd.action = ScraperArmCmdAction::CLEAR_CAL;
    cmd.value = 0.0f;
    return sendCommand(cmd);
}
