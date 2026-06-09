#include "tasks/ScraperArmNode.h"
#include "drivers/LCDDriver.h"
#include "controller.h"
#include <cmath>

static void saveClear(float pos) { StorageManager::saveScraperArmPosClear((int)pos); }
static void saveScrape(float pos) { StorageManager::saveScraperArmPosScrape((int)pos); }
static void loadLim(float& a, float& b, bool& aSet, bool& bSet) {
    int iA, iB;
    StorageManager::loadScraperArmCalibration(iA, iB);
    a = (float)iA;
    b = (float)iB;
    aSet = (iA != -1);
    bSet = (iB != -1);
}
static void savePos(float pos) { StorageManager::saveScraperArmPosition(pos); }
static float loadPos() { return StorageManager::loadScraperArmPosition(); }
static int getSG() { return systemState.scraperArmSGThreshold; }

ScraperArmNode::ScraperArmNode() : StepperAxisNode({
    "ARM_NODE",
    &Serial1,
    TMC2209::SERIAL_ADDRESS_1,
    -1, -1, -1,
    true, // has limits
    savePos, loadPos, saveClear, saveScrape, loadLim, getSG,
    0.715f // Arm velocity multiplier
}) {}

ScraperArmNode::~ScraperArmNode() {}

bool ScraperArmNode::checkInterlock(int desiredSpeed) {
    // "Scraper arm is blocked from moving down only if the DishLift is not in it's Home position"
    // Moving down means moving towards Scrape (LimitB). In our config, LimitA is Clear (approx 0), LimitB is Scrape (positive).
    // So moving down is desiredSpeed > 0
    if (desiredSpeed > 0) {
        AxisTelemetry liftTel;
        if (dishLiftTelQueue != NULL && xQueuePeek(dishLiftTelQueue, &liftTel, 0) == pdPASS) {
            // Check if lift is not in Home position (posA)
            if (!liftTel.posASet || std::abs(liftTel.currentPosition - liftTel.posA) > 5.0f) {
                LCD_setMessage("Arm Blocked: Lift!");
                return true;
            }
        }
    }
    return false;
}
