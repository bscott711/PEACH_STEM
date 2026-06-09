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
    savePos, loadPos, saveClear, saveScrape, loadLim,
    StorageManager::loadScraperArmSGThreshold(100), // initial SG Threshold
    ROT_VEL_MULT // Arm velocity multiplier
}) {}

ScraperArmNode::~ScraperArmNode() {}

bool ScraperArmNode::checkInterlock(int desiredSpeed) {
    // TEMPORARILY DISABLED: "Scraper arm is blocked from moving down only if the DishLift is not in it's Home position"
    // Moving down means moving towards Scrape (LimitB). In our config, LimitA is Clear (approx 0), LimitB is Scrape (positive).
    // So moving down is desiredSpeed > 0
    /*
    if (desiredSpeed > 0) {
        AxisTelemetry liftTel;
        if (dishLiftTelQueue != NULL && xQueuePeek(dishLiftTelQueue, &liftTel, 0) == pdPASS) {
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if ((now - liftTel.timestamp) > 500) {
                LCD_setMessage("Arm Blocked: Timeout");
                return true;
            }

            // Check if lift is not in Home position (posA)
            if (!liftTel.posASet || std::abs(liftTel.currentPosition - liftTel.posA) > 5.0f) {
                LCD_setMessage("Arm Blocked: Lift!");
                return true;
            }
        } else {
            // Queue is empty or null, block just in case
            LCD_setMessage("Arm Blocked: No Tel");
            return true;
        }
    }
    */
    return false;
}
