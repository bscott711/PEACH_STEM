#pragma once
#include "core/SystemState.h"

struct UIData {
    bool isAutoRunning;
    DeviceMode currentMode;
    Enc1Menu enc1Menu;
    Enc3Menu enc3Menu;
    
    // Hardware states
    int armTarget;
    int armActual;
    int armPosOut;
    int armPosIn;

    int actuatorTarget;
    int actuatorActual;

    int motorTargetSpeed;
    float motorPos;
    float motorLimits[3];
    bool motorLimitSet[3];
};
