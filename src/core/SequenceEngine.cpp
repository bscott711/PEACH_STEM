#include "core/SequenceEngine.h"
#include "controller.h"
#include "messaging.h"
#include "tasks/DishLiftNode.h"
#include "tasks/DishRotationNode.h"
#include "tasks/ScraperArmNode.h"
#include "drivers/LCDDriver.h"
#include "esp_log.h"
#include <cstdio>
#include <cmath>
#include <cstdint>

extern ScraperArmNode g_scraperArmNode;
extern DishRotationNode g_dishRotationNode;
extern DishLiftNode g_dishLiftNode;

static bool wait_for_event(EventBits_t bitToWait) {
    while (true) {
        EventBits_t uxBits = xEventGroupWaitBits(
            controlEvents, bitToWait | BIT_ESTOP_REQUEST,
            pdTRUE, pdFALSE, portMAX_DELAY);
        
        if (uxBits & BIT_ESTOP_REQUEST) {
            xEventGroupSetBits(controlEvents, BIT_ESTOP_REQUEST);
            return false;
        }
        if (uxBits & bitToWait) {
            return true;
        }
    }
    return false;
}

static bool move_lift(bool toTilt) {
    DishLiftTelemetry motorTel;
    float currentPos = 0;
    float targetZ = 0;
    if (xQueuePeek(dishLiftTelQueue, &motorTel, pdMS_TO_TICKS(10)) == pdPASS) {
        currentPos = motorTel.currentPosition;
        targetZ = toTilt ? motorTel.posTilt : motorTel.posHome;
    }

    if (std::abs(targetZ - currentPos) <= 0.1f) {
        return true;
    }

    int goSpeed = 5000;
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        goSpeed = systemState.dishLiftGoSpeed;
        xSemaphoreGive(systemStateMutex);
    }
    g_dishLiftNode.setTarget(targetZ, goSpeed);

    xEventGroupClearBits(controlEvents, BIT_POS_REACHED_Z);
    bool success = wait_for_event(BIT_POS_REACHED_Z);
    g_dishLiftNode.setSpeed(0);
    return success;
}

static bool move_scraper(bool toScrape) {
    ScraperArmTelemetry armTel;
    if (xQueuePeek(scraperArmTelQueue, &armTel, pdMS_TO_TICKS(10)) == pdPASS) {
        if (armTel.posClear == -1 || armTel.posScrape == -1) return true; // Uncalibrated
        
        float targetAbs = toScrape ? 100.0f : 0.0f;
        if (std::abs(armTel.currentPosition - targetAbs) < 2.0f) {
            return true;
        }

        int goSpeed = 5000;
        if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            goSpeed = systemState.scraperArmGoSpeed;
            xSemaphoreGive(systemStateMutex);
        }
        g_scraperArmNode.setTarget(targetAbs, goSpeed);

        xEventGroupClearBits(controlEvents, BIT_POS_REACHED_ARM);
        return wait_for_event(BIT_POS_REACHED_ARM);
    }
    return true;
}

static bool rotate_dish(int numRotations) {
    if (numRotations <= 0) return true;

    int goSpeed = 5000;
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        goSpeed = systemState.dishRotationGoSpeed;
        xSemaphoreGive(systemStateMutex);
    }

    // Command the jog
    g_dishRotationNode.setSpeed(goSpeed);
    g_dishRotationNode.jog(numRotations * 3200.0f); // 3200 steps = 1 rotation (approx)

    xEventGroupClearBits(controlEvents, BIT_POS_REACHED_ACT);
    return wait_for_event(BIT_POS_REACHED_ACT);
}

void autonomous_task(void *pvParameters) {
    int seqType = (int)(intptr_t)pvParameters; // 0 = Auto, 1 = Shutdown
    bool aborted = false;

    LCD_setMessage("Auto: Running");
    printf("Starting Sequence...\n");

    if (seqType == 1) {
        // Shutdown Sequence: Lower dish (Home), raise arm (Clear)
        LCD_setMessage("Auto: Lower Z");
        if (!aborted) aborted = !move_lift(false); // Home
        
        LCD_setMessage("Auto: Raise Arm");
        if (!aborted) aborted = !move_scraper(false); // Clear
    } else {
        // Auto Sequence:
        // Lower arm (Scrape), rotate dish (x number of times), raise arm (Clear), 
        // (lift dish-> lower dish) x number of time, end with dish raised (Tilt)
        
        int numRotations = 1;
        int numMix = 1;
        if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            numRotations = systemState.dishRotationNumRotations;
            numMix = systemState.dishLiftNumMix;
            xSemaphoreGive(systemStateMutex);
        }

        LCD_setMessage("Auto: Lower Arm");
        if (!aborted) aborted = !move_scraper(true); // Scrape
        
        LCD_setMessage("Auto: Rotating");
        if (!aborted) aborted = !rotate_dish(numRotations);
        
        LCD_setMessage("Auto: Raise Arm");
        if (!aborted) aborted = !move_scraper(false); // Clear

        for (int i = 0; i < numMix; i++) {
            if (aborted) break;
            LCD_setMessage("Auto: Lift Dish");
            aborted = !move_lift(true); // Tilt
            
            if (aborted) break;
            LCD_setMessage("Auto: Lower Dish");
            aborted = !move_lift(false); // Home
        }

        if (!aborted) {
            LCD_setMessage("Auto: Final Lift");
            aborted = !move_lift(true); // Tilt
        }
    }

    // ---- Cleanup ----
    if (aborted) {
        g_dishLiftNode.setSpeed(0);
        g_dishRotationNode.setSpeed(0);
        LCD_setMessage("Auto: E-STOPPED");
        printf("!!! Sequence E-STOPPED !!!\n");
        xEventGroupClearBits(controlEvents, BIT_ESTOP_REQUEST);
    } else {
        LCD_setMessage("Auto: Complete");
        printf("Sequence Complete.\n");
    }

    xEventGroupClearBits(controlEvents, BIT_AUTO_RUNNING);
    vTaskDelete(NULL);
}

void motor_goto_task(void *pvParameters) {
    int limitIdx = (int)(intptr_t)pvParameters; // 0=Bot(Home), 2=Top(Tilt)
    bool aborted = false;

    LCD_setMessage("Auto: GOTO");
    aborted = !move_lift(limitIdx == 2);

    if (aborted) {
        LCD_setMessage("GOTO E-STOPPED");
        xEventGroupClearBits(controlEvents, BIT_ESTOP_REQUEST);
    } else {
        LCD_setMessage("GOTO Complete");
    }

    xEventGroupClearBits(controlEvents, BIT_AUTO_RUNNING);
    vTaskDelete(NULL);
}
