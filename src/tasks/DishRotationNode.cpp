#include "tasks/DishRotationNode.h"
#include "controller.h"
#include "core/NetworkManager.h"
#include "drivers/LCDDriver.h"
#include <cmath>
#include <esp_log.h>

static const char *TAG = "ROTATION_NODE";

DishRotationNode::DishRotationNode()
    : currentPosition(0.0f), targetSpeed(0), previousTargetSpeed(0),
      isTrackingTarget(false), trackingTarget(0.0f),
      lastSavedPosition(-999.0f) {}

DishRotationNode::~DishRotationNode() {}

void DishRotationNode::hwInit() {
  vTaskDelay(pdMS_TO_TICKS(200));
  driver.begin(Serial1, TMC2209::SERIAL_ADDRESS_2);

  float lastPos = StorageManager::loadDishRotationPosition();
  currentPosition = lastPos;
  lastSavedPosition = lastPos;

  // Apply StallGuard threshold from SystemState
  int sg = StorageManager::loadDishRotationSGThreshold(100);
  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
      systemState.dishRotationSGThreshold = sg;
      xSemaphoreGive(systemStateMutex);
  }
  driver.setStallGuardThreshold(sg);

  PEACH_LOGI(TAG, "Loaded rotation: Pos=%.1f", lastPos);
}

void DishRotationNode::processCommand(const DishRotationCommand &cmd) {
  switch (cmd.action) {
  case DishRotationCmdAction::SET_SPEED:
    targetSpeed = (int)cmd.value;
    isTrackingTarget = false;
    PEACH_LOGD(TAG, "Rotation set speed: %d", targetSpeed);
    break;

  case DishRotationCmdAction::STOP:
    targetSpeed = 0;
    if (!isTrackingTarget) {
      PEACH_LOGD(TAG, "Rotation stopped");
    }
    break;

  case DishRotationCmdAction::SET_TARGET:
    trackingTarget = cmd.value;
    targetTrackingSpeed = cmd.targetSpeed;
    isTrackingTarget = true;
    PEACH_LOGI(TAG, "Rotation tracking target: %.2f steps at speed %d", trackingTarget, cmd.targetSpeed);
    break;

  case DishRotationCmdAction::JOG:
    if (!isTrackingTarget) {
      trackingTarget = currentPosition;
      isTrackingTarget = true;
    }
    trackingTarget += cmd.value;
    break;

  case DishRotationCmdAction::ZERO_POS:
    currentPosition = 0.0f;
    PEACH_LOGI(TAG, "Rotation zeroed");
    break;
  }
}

void DishRotationNode::hwUpdate() {
  // Update StallGuard threshold dynamically if changed
  static int lastSg = -1;
  int currentSg = 100;
  if (xSemaphoreTake(systemStateMutex, 0) == pdTRUE) {
      currentSg = systemState.dishRotationSGThreshold;
      xSemaphoreGive(systemStateMutex);
  }
  if (currentSg != lastSg) {
      driver.setStallGuardThreshold(currentSg);
      lastSg = currentSg;
  }

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
    }
  }

  // StallGuard Detection
  if (targetSpeed != 0) {
      uint16_t sgResult = driver.getStallGuardResult();
      // If SG Result drops to 0, motor is stalling.
      // Usually need to ignore SG during acceleration.
      if (sgResult == 0 && std::abs(targetSpeed) > 100) {
          PEACH_LOGW(TAG, "Stall detected on Rotation!");
          targetSpeed = 0;
          isTrackingTarget = false;
          LCD_setMessage("Rot: STALL!");
      }
  }

  // Velocity control and position integration
  if (targetSpeed != 0) {
    float stepsPerSec = (float)targetSpeed * 0.715f;
    float deltaPos = stepsPerSec * ((float)TASK_UPDATE_INTERVAL_MS / 1000.0f);
    currentPosition += deltaPos;
  }

  driver.setVelocity(targetSpeed);

  // Save position to NVS when stopped and position has changed
  if (targetSpeed == 0 && previousTargetSpeed != 0) {
    if (std::abs(currentPosition - lastSavedPosition) > 0.1f) {
      StorageManager::saveDishRotationPosition(currentPosition);
      lastSavedPosition = currentPosition;
      PEACH_LOGI(TAG, "Saved rotation position: %.2f", currentPosition);
    }
    xEventGroupSetBits(controlEvents, BIT_POS_REACHED_ACT);
  }
  previousTargetSpeed = targetSpeed;
}

DishRotationTelemetry DishRotationNode::generateTelemetry() {
  DishRotationTelemetry tel;
  tel.currentPosition = currentPosition;
  tel.targetPosition = isTrackingTarget ? trackingTarget : currentPosition;
  tel.isMoving = (targetSpeed != 0);
  return tel;
}

bool DishRotationNode::setSpeed(int speed) {
  DishRotationCommand cmd;
  cmd.action = DishRotationCmdAction::SET_SPEED;
  cmd.value = (float)speed;
  return sendCommand(cmd);
}

bool DishRotationNode::stop() {
  DishRotationCommand cmd;
  cmd.action = DishRotationCmdAction::STOP;
  cmd.value = 0.0f;
  return sendCommand(cmd);
}

bool DishRotationNode::jog(float relativeSteps) {
  DishRotationCommand cmd;
  cmd.action = DishRotationCmdAction::JOG;
  cmd.value = relativeSteps;
  return sendCommand(cmd);
}

bool DishRotationNode::setTarget(float position, int targetSpeed) {
  DishRotationCommand cmd;
  cmd.action = DishRotationCmdAction::SET_TARGET;
  cmd.value = position;
  cmd.targetSpeed = targetSpeed;
  return sendCommand(cmd);
}

bool DishRotationNode::zeroPosition() {
  DishRotationCommand cmd;
  cmd.action = DishRotationCmdAction::ZERO_POS;
  cmd.value = 0.0f;
  return sendCommand(cmd);
}
