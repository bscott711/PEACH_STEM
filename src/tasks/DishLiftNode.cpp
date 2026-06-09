#include "tasks/DishLiftNode.h"
#include "drivers/LCDDriver.h"
#include "controller.h"
#include <cmath>

static void saveHome(float pos) { StorageManager::saveDishLiftPosHome(pos); }
static void saveTilt(float pos) { StorageManager::saveDishLiftPosTilt(pos); }
static void loadLim(float& a, float& b, bool& aSet, bool& bSet) {
    StorageManager::loadDishLiftPositions(a, b, aSet, bSet);
}
static void savePos(float pos) { StorageManager::saveDishLiftState(true, pos); }
static float loadPos() {
    bool isHomed;
    float pos;
    StorageManager::loadDishLiftState(isHomed, pos);
    return pos;
}
static int getSG() { return systemState.dishLiftSGThreshold; }

DishLiftNode::DishLiftNode() : StepperAxisNode({
    "LIFT_NODE",
    &Serial1,
    TMC2209::SERIAL_ADDRESS_0,
    -1, -1, -1,
    true, // has limits
    savePos, loadPos, saveHome, saveTilt, loadLim, getSG,
    1.372e-6f // DishLift velocity multiplier
}) {}

DishLiftNode::~DishLiftNode() {}

bool DishLiftNode::checkInterlock(int desiredSpeed) {
    // "The dishLift node should not be able to Lift if the dish is rotating or if the scraper arm is not in clear position"
    // Lifting means desiredSpeed > 0 (moving towards Tilt/LimitB)
    if (desiredSpeed > 0) {
        // Check rotation
        AxisTelemetry rotTel;
        if (dishRotationTelQueue != NULL && xQueuePeek(dishRotationTelQueue, &rotTel, 0) == pdPASS) {
            if (rotTel.isMoving) {
                LCD_setMessage("Lift Blocked: Rot!");
                return true;
            }
        }
        
        // Check Arm
        AxisTelemetry armTel;
        if (scraperArmTelQueue != NULL && xQueuePeek(scraperArmTelQueue, &armTel, 0) == pdPASS) {
            // Check if arm is not in Clear position (posA). Allow a small buffer.
            if (!armTel.posASet || std::abs(armTel.currentPosition - armTel.posA) > 10.0f) {
                LCD_setMessage("Lift Blocked: Arm!");
                return true;
            }
        }
    }
    return false;
}
