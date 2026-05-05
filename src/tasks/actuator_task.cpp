#include "actuator_task.h"

// Actuator
// - Fully retracted: 10mm
// - Full extended:   10mm
// - Half way @ 15mm
// - Time to fully extend: 1000ms (timed)

// Time from 0% (retracted) to 100% (extended)
static const uint32_t FULL_EXTEND_TIME_MS = 1000;   // Calibrate!

// Tracks our estimated current position (open-loop)
static int g_actuatorCurrentPct = 0;

static void Actuator_MoveToPercent(int targetPct)
{
    targetPct = constrain(targetPct, 0, 100);

    int diff = targetPct - g_actuatorCurrentPct;
    if (diff == 0) return;

    // Convert percent difference -> milliseconds
    uint32_t moveMs = (uint32_t)(abs(diff) * FULL_EXTEND_TIME_MS) / 100;

    // Pick direction
    if (diff > 0) HBridge_Set(ACT_FORWARD);
    else          HBridge_Set(ACT_REVERSE);

    // Move for computed time, then stop
    vTaskDelay(pdMS_TO_TICKS(moveMs));
    HBridge_Set(ACT_STOP);

    // Update our estimate
    g_actuatorCurrentPct = targetPct;
}

void actuator_task(void *pvParameters)
{
    HBridge_Init();
    // Home (retract) on boot
    HBridge_Set(ACT_REVERSE);
    vTaskDelay(pdMS_TO_TICKS(FULL_EXTEND_TIME_MS));
    HBridge_Set(ACT_STOP);
    g_actuatorCurrentPct = 0;

    int lastTarget = -1;

    while (1)
    {
        int target = systemState.actuatorTargetPercent;

        if (target != lastTarget)
        {
            Actuator_MoveToPercent(target);
            lastTarget = target;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

