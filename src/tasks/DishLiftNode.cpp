#include "tasks/DishLiftNode.h"
#include "controller.h"
#include "drivers/LCDDriver.h"
#include <cmath>

static void savePos(float pos) { StorageManager::saveDishLiftState(true, pos); }
static float loadPos() {
  bool isHomed;
  float pos;
  StorageManager::loadDishLiftState(isHomed, pos);
  return pos;
}
static void saveHome(float pos) { StorageManager::saveDishLiftPosHome(pos); }
static void saveTilt(float pos) { StorageManager::saveDishLiftPosTilt(pos); }
static void loadLim(float &A, float &B, bool &ASet, bool &BSet) {
  StorageManager::loadDishLiftPositions(A, B, ASet, BSet);
}

DishLiftNode::DishLiftNode()
    : StepperAxisNode({
          "LIFT_NODE", &Serial1, TMC2209::SERIAL_ADDRESS_0, -1, -1, -1,
          SG_DIAG3,
          true, // Limits enabled
          0.5f,  // sgVelocityGatePercent
          savePos, loadPos, saveHome, saveTilt, loadLim,
          20, // Default SG threshold
          StorageManager::loadDishLiftSGThreshold, // Function pointer to load from NVS
          STEPPER_VEL_MULT // Lift velocity multiplier
      }) {}

DishLiftNode::~DishLiftNode() {}

bool DishLiftNode::checkInterlock(int desiredSpeed) {
  // TEMPORARILY DISABLED: "The dishLift node should not be able to Lift if the
  // dish is rotating or if the scraper arm is not in clear position" Lifting
  // means desiredSpeed > 0 (moving towards Tilt/LimitB)
  /*
  if (desiredSpeed > 0) {
      // Check rotation
      AxisTelemetry rotTel;
      if (dishRotationTelQueue != NULL && xQueuePeek(dishRotationTelQueue,
  &rotTel, 0) == pdPASS) { uint32_t now = xTaskGetTickCount() *
  portTICK_PERIOD_MS; if ((now - rotTel.timestamp) > 500) { LCD_setMessage("Lift
  Blocked: Timeout"); return true;
          }
          if (rotTel.isMoving) {
              LCD_setMessage("Lift Blocked: Rot!");
              return true;
          }
      } else {
          LCD_setMessage("Lift Blocked: No Tel");
          return true;
      }

      // Check Arm
      AxisTelemetry armTel;
      if (scraperArmTelQueue != NULL && xQueuePeek(scraperArmTelQueue, &armTel,
  0) == pdPASS) { uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS; if
  ((now - armTel.timestamp) > 500) { LCD_setMessage("Lift Blocked: Timeout");
              return true;
          }
          // Check if arm is not in Clear position (posA). Allow a small buffer.
          if (!armTel.posASet || std::abs(armTel.currentPosition - armTel.posA)
  > 10.0f) { LCD_setMessage("Lift Blocked: Arm!"); return true;
          }
      } else {
          LCD_setMessage("Lift Blocked: No Tel");
          return true;
      }
  }
  */
  return false;
}
