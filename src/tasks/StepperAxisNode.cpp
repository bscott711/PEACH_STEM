#include "tasks/StepperAxisNode.h"
#include "controller.h"
#include "drivers/LCDDriver.h"
#include <cmath>
#include <esp_log.h>

StepperAxisNode::StepperAxisNode(const StepperAxisConfig& cfg)
    : config(cfg), currentPosition(0.0f), targetSpeed(0), previousTargetSpeed(0),
      isTrackingTarget(false), trackingTarget(0.0f), targetTrackingSpeed(0),
      lastSavedPosition(-999.0f), limitA(0.0f), limitB(0.0f), limitASet(false),
      limitBSet(false), isHoming(false), isHomed(false), motorLocked(false) {}

StepperAxisNode::~StepperAxisNode() {}

void StepperAxisNode::hwInit() {
    vTaskDelay(pdMS_TO_TICKS(200));
    driver.begin(*(config.serialPort), config.serialAddress);

    // Initialize state
    isHomed = false;
    currentPosition = 0.0f;

    if (config.loadPositionFn) {
        currentPosition = config.loadPositionFn();
    }
    lastSavedPosition = currentPosition;

    if (config.hasLimits && config.loadLimitsFn) {
        config.loadLimitsFn(limitA, limitB, limitASet, limitBSet);
    }

    int sg = 100;
    if (config.getSGThresholdFn) {
        sg = config.getSGThresholdFn();
    }
    driver.setStallGuardThreshold(sg);

    ESP_LOGI(config.axisName, "Loaded Limits: A=%.2f(%s) B=%.2f(%s) Pos=%.1f",
             limitA, limitASet ? "Y" : "N",
             limitB, limitBSet ? "Y" : "N",
             currentPosition);
}

void StepperAxisNode::processCommand(const AxisCommand& cmd) {
    switch (cmd.action) {
    case AxisCmdAction::SET_SPEED:
        targetSpeed = (int)cmd.value;
        isTrackingTarget = false;
        ESP_LOGD(config.axisName, "Set speed: %d", targetSpeed);
        break;

    case AxisCmdAction::SET_TARGET:
        trackingTarget = cmd.value;
        targetTrackingSpeed = cmd.targetSpeed;
        isTrackingTarget = true;
        ESP_LOGI(config.axisName, "GOTO target: %.2f at speed %d", trackingTarget, cmd.targetSpeed);
        break;

    case AxisCmdAction::JOG:
        if (!isTrackingTarget) {
            trackingTarget = currentPosition;
            isTrackingTarget = true;
        }
        trackingTarget += cmd.value;
        // JOG assumes we already set targetSpeed via SET_SPEED or we default to a safe speed.
        targetTrackingSpeed = std::abs(targetSpeed) > 0 ? std::abs(targetSpeed) : 5000;
        break;

    case AxisCmdAction::START_HOMING:
        targetSpeed = -3000;
        isHoming = true;
        isTrackingTarget = false;
        ESP_LOGI(config.axisName, "Homing sequence initiated (SG)");
        break;

    case AxisCmdAction::SET_POS_A:
        limitA = cmd.value;
        limitASet = true;
        if (config.saveLimitAFn) config.saveLimitAFn(limitA);
        ESP_LOGI(config.axisName, "Limit A set to %.2f", limitA);
        break;

    case AxisCmdAction::SET_POS_B:
        limitB = cmd.value;
        limitBSet = true;
        if (config.saveLimitBFn) config.saveLimitBFn(limitB);
        ESP_LOGI(config.axisName, "Limit B set to %.2f", limitB);
        break;

    case AxisCmdAction::CLEAR_CAL:
        limitASet = false;
        limitBSet = false;
        if (config.saveLimitAFn) config.saveLimitAFn(0.0f);
        if (config.saveLimitBFn) config.saveLimitBFn(0.0f);
        ESP_LOGI(config.axisName, "Limits cleared");
        break;

    case AxisCmdAction::ZERO_POS:
        currentPosition = 0.0f;
        if (config.savePositionFn) config.savePositionFn(0.0f);
        ESP_LOGI(config.axisName, "Position zeroed");
        break;
    }
}

void StepperAxisNode::hwUpdate() {
    // 1. Update StallGuard dynamically
    int currentSg = 100;
    if (config.getSGThresholdFn) {
        currentSg = config.getSGThresholdFn();
    }
    driver.setStallGuardThreshold(currentSg);

    // 2. Interlock check
    if (checkInterlock(targetSpeed)) {
        targetSpeed = 0;
        isTrackingTarget = false;
    }

    // 3. Unlock motor if collision was cleared
    if (motorLocked && targetSpeed == 0) {
        motorLocked = false;
    }

    // 4. SG Homing logic
    if (isHoming) {
        uint16_t sgResult = driver.getStallGuardResult();
        if (sgResult == 0 && std::abs(targetSpeed) > 100) {
            targetSpeed = 0;
            currentPosition = 0.0f; // Zero out on stall
            isHomed = true;
            isHoming = false;
            limitA = 0.0f;
            limitASet = true;
            if (config.saveLimitAFn) config.saveLimitAFn(0.0f);
            ESP_LOGI(config.axisName, "Homing complete (SG stall detected)");
            LCD_setMessage("Homed");
        }
    }

    // 5. Tracking target logic & Limits
    if (!motorLocked && targetSpeed != 0 || isTrackingTarget) {
        // Calculate proportional control for tracking target
        if (isTrackingTarget) {
            float error = trackingTarget - currentPosition;
            float Kp = 5.0f;
            float desiredSpeedFloat = error * Kp;
            int desiredSpeed = (int)constrain(desiredSpeedFloat, -targetTrackingSpeed, targetTrackingSpeed);

            int maxAccelPerTick = targetTrackingSpeed / 100;
            if (maxAccelPerTick < 10) maxAccelPerTick = 10;

            if (desiredSpeed > targetSpeed + maxAccelPerTick) {
                targetSpeed += maxAccelPerTick;
            } else if (desiredSpeed < targetSpeed - maxAccelPerTick) {
                targetSpeed -= maxAccelPerTick;
            } else {
                targetSpeed = desiredSpeed;
            }

            if (std::abs(error) < 2.0f && std::abs(targetSpeed) <= 10) {
                currentPosition = trackingTarget;
                targetSpeed = 0;
                isTrackingTarget = false;
                LCD_setMessage("Target Reached");
            }
        }

        // Apply Limits (Soft Endstops) if enabled
        if (config.hasLimits && !isHoming && targetSpeed != 0) {
            float minLim = min(limitA, limitB);
            float maxLim = max(limitA, limitB);
            bool minSet = (limitA <= limitB) ? limitASet : limitBSet;
            bool maxSet = (limitA >= limitB) ? limitASet : limitBSet;

            if (targetSpeed < 0 && minSet) {
                float distToBot = currentPosition - minLim;
                if (distToBot <= 0.0f) {
                    targetSpeed = 0;
                    currentPosition = minLim;
                    isTrackingTarget = false;
                    LCD_setMessage("Min Limit Reached");
                } else if (distToBot < 5.0f) {
                    int minSpeed = 1000;
                    int maxSpeed = std::abs(targetSpeed);
                    if (maxSpeed > minSpeed) {
                        targetSpeed = -(minSpeed + (int)((maxSpeed - minSpeed) * (distToBot / 5.0f)));
                    }
                }
            } else if (targetSpeed > 0 && maxSet) {
                float distToTop = maxLim - currentPosition;
                if (distToTop <= 0.0f) {
                    targetSpeed = 0;
                    currentPosition = maxLim;
                    isTrackingTarget = false;
                    LCD_setMessage("Max Limit Reached");
                } else if (distToTop < 5.0f) {
                    int minSpeed = 1000;
                    int maxSpeed = std::abs(targetSpeed);
                    if (maxSpeed > minSpeed) {
                        targetSpeed = minSpeed + (int)((maxSpeed - minSpeed) * (distToTop / 5.0f));
                    }
                }
            }
        }
    }

    // 6. SG Crash Detection (only if not homing)
    if (!isHoming && targetSpeed != 0) {
        uint16_t sgResult = driver.getStallGuardResult();
        if (sgResult == 0 && std::abs(targetSpeed) > 100) {
            ESP_LOGW(config.axisName, "Stall detected!");
            targetSpeed = 0;
            isTrackingTarget = false;
            LCD_setMessage("STALL!");
        }
    }

    // 7. Update Position
    if (targetSpeed != 0) {
        float deltaPos = config.velocityMultiplier * (float)targetSpeed * ((float)TASK_UPDATE_INTERVAL_MS / 1000.0f);
        currentPosition += deltaPos;
    }

    // 8. Command Hardware
    if (motorLocked) {
        driver.stop();
    } else {
        driver.setVelocity(targetSpeed);
    }

    // 9. Save Position on Stop
    if (targetSpeed == 0 && previousTargetSpeed != 0) {
        if (std::abs(currentPosition - lastSavedPosition) > 0.1f) {
            if (config.savePositionFn) config.savePositionFn(currentPosition);
            lastSavedPosition = currentPosition;
            ESP_LOGI(config.axisName, "Saved position: %.2f", currentPosition);
        }
        xEventGroupSetBits(controlEvents, BIT_POS_REACHED_Z | BIT_POS_REACHED_ARM | BIT_POS_REACHED_ACT);
    }

    previousTargetSpeed = targetSpeed;
}

AxisTelemetry StepperAxisNode::generateTelemetry() {
    AxisTelemetry tel;
    tel.currentPosition = currentPosition;
    tel.targetPosition = isTrackingTarget ? trackingTarget : currentPosition;
    tel.targetSpeed = targetSpeed;
    tel.posA = limitA;
    tel.posB = limitB;
    tel.posASet = limitASet;
    tel.posBSet = limitBSet;
    tel.isMoving = (targetSpeed != 0);
    tel.isHoming = isHoming;
    tel.isHomed = isHomed;
    return tel;
}

bool StepperAxisNode::setSpeed(int speed) {
    AxisCommand cmd;
    cmd.action = AxisCmdAction::SET_SPEED;
    cmd.value = (float)speed;
    return sendCommand(cmd);
}

bool StepperAxisNode::setTarget(float position, int speed) {
    AxisCommand cmd;
    cmd.action = AxisCmdAction::SET_TARGET;
    cmd.value = position;
    cmd.targetSpeed = speed;
    return sendCommand(cmd);
}

bool StepperAxisNode::jog(float relativeSteps) {
    AxisCommand cmd;
    cmd.action = AxisCmdAction::JOG;
    cmd.value = relativeSteps;
    return sendCommand(cmd);
}

bool StepperAxisNode::stop() {
    AxisCommand cmd;
    cmd.action = AxisCmdAction::SET_SPEED; // using set speed to 0 instead of STOP
    cmd.value = 0.0f;
    return sendCommand(cmd);
}

bool StepperAxisNode::setLimitA(float position) {
    AxisCommand cmd;
    cmd.action = AxisCmdAction::SET_POS_A;
    cmd.value = position;
    return sendCommand(cmd);
}

bool StepperAxisNode::setLimitB(float position) {
    AxisCommand cmd;
    cmd.action = AxisCmdAction::SET_POS_B;
    cmd.value = position;
    return sendCommand(cmd);
}

bool StepperAxisNode::clearLimits() {
    AxisCommand cmd;
    cmd.action = AxisCmdAction::CLEAR_CAL;
    cmd.value = 0.0f;
    return sendCommand(cmd);
}

bool StepperAxisNode::zeroPosition() {
    AxisCommand cmd;
    cmd.action = AxisCmdAction::ZERO_POS;
    cmd.value = 0.0f;
    return sendCommand(cmd);
}

bool StepperAxisNode::startHoming() {
    AxisCommand cmd;
    cmd.action = AxisCmdAction::START_HOMING;
    cmd.value = 0.0f;
    return sendCommand(cmd);
}
