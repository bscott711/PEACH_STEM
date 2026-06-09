#include "tasks/DishRotationNode.h"

#include "controller.h"
#include "core/NetworkManager.h"
#include <cmath>

static const char *TAG = "ROTATION_NODE";

DishRotationNode::DishRotationNode()
    : currentPercent(0.0f), targetPercent(0), targetSpeedPWM(255),
      lastSavedPercent(-1.0f), wasMoving(false) {
  limits[0] = 0;
  limits[1] = 0;
  limits[2] = 0;
  limitSet[0] = false;
  limitSet[1] = false;
  limitSet[2] = false;
}

DishRotationNode::~DishRotationNode() {}

void DishRotationNode::hwInit() {

  // Open NVS namespace for limit and position storage
  StorageManager::loadDishRotationLimits(limits, limitSet);
  float lastPos = StorageManager::loadDishRotationPosition();

  currentPercent = lastPos;
  targetPercent = (int)currentPercent;
  lastSavedPercent = lastPos;

  PEACH_LOGI(TAG, "Loaded limits: Bot=%d(%s), Mid=%d(%s), Top=%d(%s), Pos=%.1f",
             limits[0], limitSet[0] ? "Y" : "N", limits[1],
             limitSet[1] ? "Y" : "N", limits[2], limitSet[2] ? "Y" : "N",
             lastPos);
}

void DishRotationNode::processCommand(const DishRotationCommand &cmd) {
  switch (cmd.action) {
  case DishRotationCmdAction::SET_TARGET:
    targetPercent = constrain(cmd.value, 0, 100);
    targetSpeedPWM = constrain(cmd.pwmSpeed, 0, 255);
    wasMoving = true; // Ensure event triggers even if already at target
    PEACH_LOGD(TAG, "Set target: %d%%, spd: %d", targetPercent, targetSpeedPWM);
    break;

  case DishRotationCmdAction::SET_LIMIT_BOT:
    limits[0] = cmd.value;
    limitSet[0] = true;
    StorageManager::saveDishRotationLimit(StorageManager::LIMIT_BOT, limits[0],
                                          true);
    PEACH_LOGI(TAG, "Bottom limit set to %d%%", limits[0]);
    break;

  case DishRotationCmdAction::SET_LIMIT_MID:
    limits[1] = cmd.value;
    limitSet[1] = true;
    StorageManager::saveDishRotationLimit(StorageManager::LIMIT_MID, limits[1],
                                          true);
    PEACH_LOGI(TAG, "Middle limit set to %d%%", limits[1]);
    break;

  case DishRotationCmdAction::SET_LIMIT_TOP:
    limits[2] = cmd.value;
    limitSet[2] = true;
    StorageManager::saveDishRotationLimit(StorageManager::LIMIT_TOP, limits[2],
                                          true);
    PEACH_LOGI(TAG, "Top limit set to %d%%", limits[2]);
    break;

  case DishRotationCmdAction::CLEAR_LIMIT_BOT:
    limitSet[0] = false;
    StorageManager::saveDishRotationLimit(StorageManager::LIMIT_BOT, limits[0],
                                          false);
    PEACH_LOGI(TAG, "Bottom limit cleared");
    break;

  case DishRotationCmdAction::CLEAR_LIMIT_MID:
    limitSet[1] = false;
    StorageManager::saveDishRotationLimit(StorageManager::LIMIT_MID, limits[1],
                                          false);
    PEACH_LOGI(TAG, "Middle limit cleared");
    break;

  case DishRotationCmdAction::CLEAR_LIMIT_TOP:
    limitSet[2] = false;
    StorageManager::saveDishRotationLimit(StorageManager::LIMIT_TOP, limits[2],
                                          false);
    PEACH_LOGI(TAG, "Top limit cleared");
    break;

  case DishRotationCmdAction::GET_LIMITS:
    // Telemetry will include limit data automatically
    break;
  }
}

// Helper function for empirical piecewise linear interpolation
static float interpolateTime(int pwm, const int pwms[], const float times[],
                             int size) {
  if (pwm <= pwms[0])
    return times[0];
  if (pwm >= pwms[size - 1])
    return times[size - 1];

  for (int i = 0; i < size - 1; i++) {
    if (pwm >= pwms[i] && pwm <= pwms[i + 1]) {
      float t = (float)(pwm - pwms[i]) / (float)(pwms[i + 1] - pwms[i]);
      return times[i] + t * (times[i + 1] - times[i]);
    }
  }
  return times[size - 1];
}

void DishRotationNode::hwUpdate() {
  float timeMs = 3000.0f; // Default safe fallback

  if (currentPercent < targetPercent) {
    wasMoving = true;
    // ==========================
    // EXTENDING (Forward)
    // Measured empirical data
    // ==========================
    const int pwms[] = {155, 165, 175, 205, 255};
    const float times[] = {6000.0f, 4000.0f, 3000.0f, 1800.0f, 800.0f};
    timeMs = interpolateTime(targetSpeedPWM, pwms, times, 5);

    float dynamicPctPerTick =
        (100.0f * (float)TASK_UPDATE_INTERVAL_MS) / timeMs;
    currentPercent += dynamicPctPerTick;

    if (currentPercent > targetPercent) {
      currentPercent = targetPercent; // Clamp exact arrival
    }

  } else if (currentPercent > targetPercent) {
    wasMoving = true;
    // ==========================
    // RETRACTING (Reverse)
    // Measured empirical data
    // ==========================
    const int pwms[] = {155, 175, 205, 255};
    const float times[] = {3000.0f, 3000.0f, 1500.0f, 800.0f};
    timeMs = interpolateTime(targetSpeedPWM, pwms, times, 4);

    float dynamicPctPerTick =
        (100.0f * (float)TASK_UPDATE_INTERVAL_MS) / timeMs;
    currentPercent -= dynamicPctPerTick;

    if (currentPercent < targetPercent) {
      currentPercent = targetPercent; // Clamp exact arrival
    }

  } else {

    if (wasMoving) {
      // Save position to NVS if it has changed since last save
      if (std::abs(currentPercent - lastSavedPercent) > 0.1f) {
        StorageManager::saveDishRotationPosition(currentPercent);
        lastSavedPercent = currentPercent;
        PEACH_LOGI(TAG, "Saved actuator position: %.2f%%", currentPercent);
      }
      xEventGroupSetBits(controlEvents, BIT_POS_REACHED_ACT);
      wasMoving = false;
    }
  }
}

DishRotationTelemetry DishRotationNode::generateTelemetry() {
  DishRotationTelemetry tel;
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

bool DishRotationNode::setTarget(int percent, int pwmSpeed) {
  DishRotationCommand cmd;
  cmd.action = DishRotationCmdAction::SET_TARGET;
  cmd.value = percent;
  cmd.pwmSpeed = pwmSpeed;
  return sendCommand(cmd);
}

bool DishRotationNode::setLimitBot(int percent) {
  DishRotationCommand cmd;
  cmd.action = DishRotationCmdAction::SET_LIMIT_BOT;
  cmd.value = percent;
  return sendCommand(cmd);
}

bool DishRotationNode::setLimitMid(int percent) {
  DishRotationCommand cmd;
  cmd.action = DishRotationCmdAction::SET_LIMIT_MID;
  cmd.value = percent;
  return sendCommand(cmd);
}

bool DishRotationNode::setLimitTop(int percent) {
  DishRotationCommand cmd;
  cmd.action = DishRotationCmdAction::SET_LIMIT_TOP;
  cmd.value = percent;
  return sendCommand(cmd);
}

bool DishRotationNode::clearLimitBot() {
  DishRotationCommand cmd;
  cmd.action = DishRotationCmdAction::CLEAR_LIMIT_BOT;
  cmd.value = 0;
  return sendCommand(cmd);
}

bool DishRotationNode::clearLimitMid() {
  DishRotationCommand cmd;
  cmd.action = DishRotationCmdAction::CLEAR_LIMIT_MID;
  cmd.value = 0;
  return sendCommand(cmd);
}

bool DishRotationNode::clearLimitTop() {
  DishRotationCommand cmd;
  cmd.action = DishRotationCmdAction::CLEAR_LIMIT_TOP;
  cmd.value = 0;
  return sendCommand(cmd);
}
