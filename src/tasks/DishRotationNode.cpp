#include "tasks/DishRotationNode.h"
#include "drivers/LCDDriver.h"
#include "controller.h"
#include <cmath>

static void savePos(float pos) { StorageManager::saveDishRotationPosition(pos); }
static float loadPos() { return StorageManager::loadDishRotationPosition(); }

DishRotationNode::DishRotationNode() : StepperAxisNode({
    "ROTATION_NODE",
    &Serial1,
    TMC2209::SERIAL_ADDRESS_2,
    -1, -1, -1, SG_DIAG1,
    false, // NO limits (continuous rotation)
    0.5f,  // sgVelocityGatePercent
    savePos, loadPos, nullptr, nullptr, nullptr,
    StorageManager::loadDishRotationSGThreshold(20), // initial SG
    ROT_VEL_MULT // Rotation velocity multiplier
}) {}

DishRotationNode::~DishRotationNode() {}

bool DishRotationNode::checkInterlock(int desiredSpeed) {

    return false;
}
