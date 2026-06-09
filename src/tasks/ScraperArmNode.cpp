#include "ScraperArmNode.h"
#include "controller.h"
#include "core/NetworkManager.h"
#include "drivers/LCDDriver.h"
#include <cmath>
#include <esp_log.h>

static const char *TAG = "SCRAPER_NODE";

ScraperArmNode::ScraperArmNode()
    : currentPosition(0.0f), targetSpeed(0), previousTargetSpeed(0), posClear(-1),
      posScrape(-1), isTrackingTarget(false), targetTrackingAbsSteps(0.0f),
      lastSavedPosition(-999.0f) {}

ScraperArmNode::~ScraperArmNode() {}

void ScraperArmNode::hwInit() {
  vTaskDelay(pdMS_TO_TICKS(200));
  driver.begin(Serial1, TMC2209::SERIAL_ADDRESS_1);

  StorageManager::loadScraperArmCalibration(posClear, posScrape);
  float lastPos = StorageManager::loadScraperArmPosition();

  currentPosition = lastPos;
  lastSavedPosition = lastPos;

  // Apply StallGuard threshold from SystemState
  int sg = StorageManager::loadScraperArmSGThreshold(100);
  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
      systemState.scraperArmSGThreshold = sg;
      xSemaphoreGive(systemStateMutex);
  }
  driver.setStallGuardThreshold(sg);

  PEACH_LOGI(TAG, "Loaded calibration: Clear=%d, Scrape=%d, Pos=%.1f", posClear,
             posScrape, lastPos);
}

void ScraperArmNode::processCommand(const ScraperArmCommand &cmd) {
  switch (cmd.action) {
  case ScraperArmCmdAction::SET_SPEED:
    targetSpeed = (int)cmd.value;
    isTrackingTarget = false;
    PEACH_LOGD(TAG, "Arm set speed: %d", targetSpeed);
    break;

  case ScraperArmCmdAction::STOP:
    targetSpeed = 0;
    if (!isTrackingTarget) {
      PEACH_LOGD(TAG, "Arm stopped");
    }
    break;

  case ScraperArmCmdAction::SET_TARGET:
    if (posClear != -1 && posScrape != -1) {
      targetTrackingAbsSteps = posClear + (cmd.value / 100.0f) * (posScrape - posClear);
      PEACH_LOGI(TAG, "Arm tracking target: %.2f%% -> %.2f steps at speed %d",
                 cmd.value, targetTrackingAbsSteps, cmd.targetSpeed);
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

  case ScraperArmCmdAction::SET_POS_CLEAR:
    currentPosition = 0.0f;
    posClear = 0;
    StorageManager::saveScraperArmPosClear(posClear);
    PEACH_LOGI(TAG, "Arm posClear set to 0 and position zeroed");
    break;

  case ScraperArmCmdAction::SET_POS_SCRAPE:
    posScrape = (int)currentPosition;
    StorageManager::saveScraperArmPosScrape(posScrape);
    PEACH_LOGI(TAG, "Arm posScrape set to %d", posScrape);
    break;

  case ScraperArmCmdAction::CLEAR_CAL:
    posClear = -1;
    posScrape = -1;
    StorageManager::saveScraperArmPosClear(-1);
    StorageManager::saveScraperArmPosScrape(-1);
    PEACH_LOGI(TAG, "Arm calibration cleared");
    break;
  }
}

void ScraperArmNode::hwUpdate() {
  // Update StallGuard threshold dynamically if changed
  static int lastSg = -1;
  int currentSg = 100;
  if (xSemaphoreTake(systemStateMutex, 0) == pdTRUE) {
      currentSg = systemState.scraperArmSGThreshold;
      xSemaphoreGive(systemStateMutex);
  }
  if (currentSg != lastSg) {
      driver.setStallGuardThreshold(currentSg);
      lastSg = currentSg;
  }

  if (isTrackingTarget) {
    float error = targetTrackingAbsSteps - currentPosition;
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
      currentPosition = targetTrackingAbsSteps;
      targetSpeed = 0;
      isTrackingTarget = false;
    }
  }

  // StallGuard Detection (specifically useful when reaching the Scrape limit)
  if (targetSpeed != 0) {
      uint16_t sgResult = driver.getStallGuardResult();
      if (sgResult == 0 && std::abs(targetSpeed) > 100) {
          PEACH_LOGW(TAG, "Stall detected on Arm!");
          targetSpeed = 0;
          isTrackingTarget = false;
          LCD_setMessage("Arm: STALL/LIMIT!");
          
          // Optionally, auto-set Scrape position here if not calibrated
          // But usually we just let it stop.
      }
  }

  // Velocity control and position integration
  if (targetSpeed != 0) {
    float stepsPerSec = (float)targetSpeed * 0.715f;
    float deltaPos = stepsPerSec * ((float)TASK_UPDATE_INTERVAL_MS / 1000.0f);
    currentPosition += deltaPos;
  }

  // Read Z motor telemetry for interlock logic
  DishLiftTelemetry motorTel;
  bool zAtTilt = false;
  if (dishLiftTelQueue != NULL &&
      xQueuePeek(dishLiftTelQueue, &motorTel, 0) == pdPASS) {
    if (motorTel.posTiltSet &&
        motorTel.currentPosition >= motorTel.posTilt - 5.0f) {
      zAtTilt = true;
    }
  }

  // Collision logic: If not at Clear and not at Tilt, warning? 
  // Wait, Z must be at Tilt (Upper) to move the Arm safely?
  // Let's just prevent Arm movement if Z isn't at Tilt and we are not close to Clear.
  bool inCollisionZone = false;
  if (posClear != -1 && posScrape != -1) {
    float minZone = min((float)posClear, (float)posScrape) + 200.0f; // Add buffer zone near clear?
    float maxZone = max((float)posClear, (float)posScrape);
    if (currentPosition >= minZone && currentPosition <= maxZone) {
      inCollisionZone = true;
    }
  }

  // Hard Endstops
  if (posClear != -1 && posScrape != -1 && targetSpeed != 0) {
    float minLim = min((float)posClear, (float)posScrape);
    float maxLim = max((float)posClear, (float)posScrape);

    if (currentPosition >= maxLim && targetSpeed > 0) {
      targetSpeed = 0;
      currentPosition = maxLim;
      isTrackingTarget = false;
      LCD_setMessage((posScrape > posClear) ? "Arm Scrape Reached"
                                      : "Arm Clear Reached");
    } else if (currentPosition <= minLim && targetSpeed < 0) {
      targetSpeed = 0;
      currentPosition = minLim;
      isTrackingTarget = false;
      LCD_setMessage((posScrape < posClear) ? "Arm Scrape Reached"
                                      : "Arm Clear Reached");
    }
  }

  // Interlock Check (simplified)
  if (inCollisionZone && !zAtTilt && targetSpeed != 0) {
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
  tel.posClear = posClear;
  tel.posScrape = posScrape;
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

bool ScraperArmNode::setPosClear() {
  ScraperArmCommand cmd;
  cmd.action = ScraperArmCmdAction::SET_POS_CLEAR;
  cmd.value = 0.0f;
  return sendCommand(cmd);
}

bool ScraperArmNode::setPosScrape() {
  ScraperArmCommand cmd;
  cmd.action = ScraperArmCmdAction::SET_POS_SCRAPE;
  cmd.value = 0.0f;
  return sendCommand(cmd);
}

bool ScraperArmNode::clearCal() {
  ScraperArmCommand cmd;
  cmd.action = ScraperArmCmdAction::CLEAR_CAL;
  cmd.value = 0.0f;
  return sendCommand(cmd);
}
