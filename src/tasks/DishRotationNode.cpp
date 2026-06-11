#include "tasks/DishRotationNode.h"
#include "controller.h"
#include "drivers/LCDDriver.h"
#include <cmath>

static void savePos(float pos) {
  StorageManager::saveDishRotationPosition(pos);
}
static float loadPos() { return StorageManager::loadDishRotationPosition(); }

DishRotationNode::DishRotationNode()
    : StepperAxisNode({
          "ROTATION_NODE", &Serial1, TMC2209::SERIAL_ADDRESS_2, -1, -1, -1,
          SG_DIAG1,
          false, // NO limits (continuous rotation)
          0.5f,  // sgVelocityGatePercent
          savePos, loadPos, nullptr, nullptr, nullptr,
          StorageManager::loadDishRotationSGThreshold(20), // initial SG
          STEPPER_VEL_MULT // Rotation velocity multiplier
      }) {}

DishRotationNode::~DishRotationNode() {}

bool DishRotationNode::checkInterlock(int desiredSpeed) {
  // TEMPORARILY DISABLED: "Dish rotation is blocked if the Lift is not in its
  // home position"
  /*
  if (desiredSpeed != 0) {
      AxisTelemetry liftTel;
      if (dishLiftTelQueue != NULL && xQueuePeek(dishLiftTelQueue, &liftTel, 0)
  == pdPASS) { uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS; if ((now
  - liftTel.timestamp) > 500) { LCD_setMessage("Rot Blocked: Timeout"); return
  true;
          }

          // Check if lift is not in Home position (posA)
          if (!liftTel.posASet || std::abs(liftTel.currentPosition -
  liftTel.posA) > 5.0f) { LCD_setMessage("Rot Blocked: Lift!"); return true;
          }
      } else {
          LCD_setMessage("Rot Blocked: No Tel");
          return true;
      }
  }
  */
  return false;
}
