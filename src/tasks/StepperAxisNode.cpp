#include "tasks/StepperAxisNode.h"
#include "controller.h"
#include "drivers/LCDDriver.h"
#include <cmath>
#include <esp_log.h>

StepperAxisNode::StepperAxisNode(const StepperAxisConfig& cfg)
    : config(cfg), currentPosition(0.0f), targetSpeed(0), previousTargetSpeed(0),
      isTrackingTarget(false), trackingTarget(0.0f), targetTrackingSpeed(0),
      lastSavedPosition(-999.0f), limitA(0.0f), limitB(0.0f), limitASet(false),
      limitBSet(false), isHoming(false), isHomed(false), motorLocked(false), currentSgThreshold(cfg.initialSgThreshold), lastPushedSg(-1), filteredSgResult(0), movementStartTime(0) {}

StepperAxisNode::~StepperAxisNode() {}

void StepperAxisNode::updateVelocityGate(int speed) {
    if (speed == 0) return;
    currentGateVelocity = (int)(std::abs((float)speed) * config.sgVelocityGatePercent);
    if (currentGateVelocity < 10) currentGateVelocity = 10;
    uint32_t tstep = 16777216 / currentGateVelocity;
    driver.setCoolStepDurationThreshold(tstep);
}

void StepperAxisNode::hwInit() {
    vTaskDelay(pdMS_TO_TICKS(200));
    driver.begin(*(config.serialPort), config.serialAddress);

    if (config.diagPin >= 0) {
        pinMode(config.diagPin, INPUT_PULLDOWN);
    }

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

    driver.setStallGuardThreshold(currentSgThreshold);

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
        updateVelocityGate(targetSpeed);
        ESP_LOGD(config.axisName, "Set speed: %d", targetSpeed);
        break;

    case AxisCmdAction::SET_TARGET:
        trackingTarget = cmd.value;
        targetTrackingSpeed = cmd.targetSpeed;
        isTrackingTarget = true;
        updateVelocityGate(targetTrackingSpeed);
        ESP_LOGI(config.axisName, "GOTO target: %.2f at speed %d", trackingTarget, cmd.targetSpeed);
        break;

    case AxisCmdAction::JOG:
        if (!isTrackingTarget) {
            trackingTarget = currentPosition;
            isTrackingTarget = true;
        }
        trackingTarget += cmd.value;
        // JOG assumes we already set targetSpeed via SET_SPEED or we default to a safe speed.
        targetTrackingSpeed = std::abs(targetSpeed) > 0 ? std::abs(targetSpeed) : MOTOR_DEFAULT_JOG_SPEED;
        updateVelocityGate(targetTrackingSpeed);
        break;

    case AxisCmdAction::START_HOMING:
        targetSpeed = -3000;
        isHoming = true;
        isTrackingTarget = false;
        updateVelocityGate(targetSpeed);
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
        if (config.saveLimitAFn) config.saveLimitAFn(-1000000.0f);
        if (config.saveLimitBFn) config.saveLimitBFn(-1000000.0f);
        ESP_LOGI(config.axisName, "Limits cleared");
        break;

    case AxisCmdAction::ZERO_POS:
        currentPosition = 0.0f;
        if (config.savePositionFn) config.savePositionFn(0.0f);
        ESP_LOGI(config.axisName, "Position zeroed");
        break;

    case AxisCmdAction::SET_SG_THRESHOLD:
        currentSgThreshold = (int)cmd.value;
        ESP_LOGI(config.axisName, "SG Threshold updated to %d", currentSgThreshold);
        break;

    case AxisCmdAction::SET_CURRENT:
        driver.setCurrent(cmd.currentPercent);
        ESP_LOGI(config.axisName, "Motor current set to %d%%", cmd.currentPercent);
        break;
    }
}

void StepperAxisNode::hwUpdate() {
    // 1. Update StallGuard dynamically
    if (currentSgThreshold != lastPushedSg) {
        driver.setStallGuardThreshold(currentSgThreshold);
        lastPushedSg = currentSgThreshold;
    }

    // 2. Interlock check
    if (checkInterlock(targetSpeed)) {
        targetSpeed = 0;
        isTrackingTarget = false;
    }

    // 3. Unlock motor if collision was cleared
    if (motorLocked && targetSpeed == 0) {
        motorLocked = false;
    }

    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (targetSpeed != 0 && previousTargetSpeed == 0) {
        movementStartTime = now;
    }

    // 4. SG Homing logic
    bool isMovingStable = std::abs(targetSpeed) > currentGateVelocity;

    if (isHoming && isMovingStable && (now - movementStartTime) > 300) {
        bool stall = false;
        if (config.diagPin >= 0) {
            stall = (digitalRead(config.diagPin) == LOW);
        } else {
            uint16_t sgResult = driver.getStallGuardResult();
            // TMC2209: SG_RESULT increases with load. Stall when it exceeds threshold.
            stall = (currentSgThreshold > 0 && sgResult >= (uint16_t)(currentSgThreshold * 2));
        }

        if (stall) {
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
            if (std::abs(error) <= MOTOR_TARGET_TOLERANCE) {
                currentPosition = trackingTarget;
                targetSpeed = 0;
                isTrackingTarget = false;
                LCD_setMessage("Target Reached");
            } else {
                targetSpeed = (error > 0) ? targetTrackingSpeed : -targetTrackingSpeed;
            }
        }

        // Apply Limits (Soft Endstops) if enabled
        if (config.hasLimits && limitASet && limitBSet && !isHoming && targetSpeed != 0) {
            float minLim = std::min(limitA, limitB);
            float maxLim = std::max(limitA, limitB);
            bool minSet = (limitA <= limitB) ? limitASet : limitBSet;
            bool maxSet = (limitA >= limitB) ? limitASet : limitBSet;

            if (targetSpeed < 0 && minSet) {
                float distToBot = currentPosition - minLim;
                if (distToBot <= 0.0f) {
                    targetSpeed = 0;
                    currentPosition = minLim;
                    isTrackingTarget = false;
                    LCD_setMessage("Min Limit Reached");
                }
            } else if (targetSpeed > 0 && maxSet) {
                float distToTop = maxLim - currentPosition;
                if (distToTop <= 0.0f) {
                    targetSpeed = 0;
                    currentPosition = maxLim;
                    isTrackingTarget = false;
                    LCD_setMessage("Max Limit Reached");
                }
            }
        }
    }

    // 6. SG Crash Detection (Disabled for calibration)
    if (!isHoming && targetSpeed != 0 && (now - movementStartTime) > 600) {
        // Just read the value for telemetry so it shows up on the LCD
        uint16_t sgRaw = driver.getStallGuardResult();
        
        if (filteredSgResult == 0) {
            filteredSgResult = sgRaw;
        } else {
            filteredSgResult = (filteredSgResult * 3 + sgRaw) / 4;
        }

        // Stall Abort Logic
        // TMC2209: SG_RESULT increases with load. Stall when it exceeds threshold.
        bool stall = false;
        if (currentSgThreshold > 0) {
            if (config.diagPin >= 0) {
                stall = (digitalRead(config.diagPin) == LOW);
            } else {
                stall = (sgRaw >= (uint16_t)(currentSgThreshold * 2));
            }
        }

        if (stall && std::abs(targetSpeed) > 100) {
            ESP_LOGW(config.axisName, "Stall detected!");
            targetSpeed = 0;
            isTrackingTarget = false;
            LCD_setMessage("STALL!");
        }
    } else if (targetSpeed == 0) {
        filteredSgResult = 0; // Reset filter when stopped
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
    tel.sgResult = filteredSgResult;
    tel.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
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

bool StepperAxisNode::setSGThreshold(int threshold) {
    AxisCommand cmd;
    cmd.action = AxisCmdAction::SET_SG_THRESHOLD;
    cmd.value = (float)threshold;
    return sendCommand(cmd);
}

bool StepperAxisNode::setCurrent(uint8_t currentPercent) {
    AxisCommand cmd;
    cmd.action = AxisCmdAction::SET_CURRENT;
    cmd.currentPercent = currentPercent;
    return sendCommand(cmd);
}
