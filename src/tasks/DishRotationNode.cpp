#include "tasks/DishRotationNode.h"
#include "drivers/LCDDriver.h"
#include "controller.h"
#include <cmath>

static void savePos(float pos) { StorageManager::saveDishRotationPosition(pos); }
static float loadPos() { return StorageManager::loadDishRotationPosition(); }
static int getSG() { return systemState.dishRotationSGThreshold; }

DishRotationNode::DishRotationNode() : StepperAxisNode({
    "ROT_NODE",
    &Serial1,
    TMC2209::SERIAL_ADDRESS_2,
    -1, -1, -1,
    false, // NO limits (continuous rotation)
    savePos, loadPos, nullptr, nullptr, nullptr, getSG,
    0.715f // Rotation velocity multiplier
}) {}

DishRotationNode::~DishRotationNode() {}

bool DishRotationNode::checkInterlock(int desiredSpeed) {
    // "Dish rotation is blocked if the Lift is not in its home position"
    if (desiredSpeed != 0) {
        AxisTelemetry liftTel;
        if (dishLiftTelQueue != NULL && xQueuePeek(dishLiftTelQueue, &liftTel, 0) == pdPASS) {
            // Check if lift is not in Home position (posA)
            if (!liftTel.posASet || std::abs(liftTel.currentPosition - liftTel.posA) > 5.0f) {
                LCD_setMessage("Rot Blocked: Lift!");
                return true;
            }
        }
    }
    return false;
}
