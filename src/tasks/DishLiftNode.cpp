#include "tasks/DishLiftNode.h"
#include "drivers/LCDDriver.h"
#include "controller.h"
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
static void loadLim(float& A, float& B, bool& ASet, bool& BSet) {
    StorageManager::loadDishLiftPositions(A, B, ASet, BSet);
}

DishLiftNode::DishLiftNode() : StepperAxisNode({
    "LIFT_NODE",
    &Serial1,
    TMC2209::SERIAL_ADDRESS_0,
    -1, -1, -1, SG_DIAG3,
    false, // Temporarily disabled limits per user request
    0.5f,  // sgVelocityGatePercent
    savePos, loadPos, saveHome, saveTilt, loadLim,
    StorageManager::loadDishLiftSGThreshold(20), // initial SG
    Z_VEL_MULT // Lift velocity multiplier
}) {}

DishLiftNode::~DishLiftNode() {}

bool DishLiftNode::checkInterlock(int desiredSpeed) {

    return false;
}
