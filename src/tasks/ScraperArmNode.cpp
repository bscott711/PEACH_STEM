#include "tasks/ScraperArmNode.h"
#include "drivers/LCDDriver.h"
#include "controller.h"
#include <cmath>

static void saveClear(float pos) { StorageManager::saveScraperArmPosClear((int)pos); }
static void saveScrape(float pos) { StorageManager::saveScraperArmPosScrape((int)pos); }
static void loadLim(float& A, float& B, bool& ASet, bool& BSet) {
    int pa, pb;
    StorageManager::loadScraperArmCalibration(pa, pb);
    if (pa != -1 && pb != -1) {
        A = pa; B = pb;
        ASet = true; BSet = true;
    }
}
static void savePos(float pos) { StorageManager::saveScraperArmPosition(pos); }
static float loadPos() { return StorageManager::loadScraperArmPosition(); }

ScraperArmNode::ScraperArmNode() : StepperAxisNode({
    "ARM_NODE",
    &Serial1,
    TMC2209::SERIAL_ADDRESS_1,
    -1, -1, -1, SG_DIAG2,
    false, // Temporarily disabled limits per user request
    0.5f,  // sgVelocityGatePercent
    savePos, loadPos, saveClear, saveScrape, loadLim,
    StorageManager::loadScraperArmSGThreshold(20), // initial SG Threshold
    ROT_VEL_MULT // Arm velocity multiplier
}) {}

ScraperArmNode::~ScraperArmNode() {}

bool ScraperArmNode::checkInterlock(int desiredSpeed) {

    return false;
}
