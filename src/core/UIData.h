#pragma once
#include "core/SystemState.h"

struct UIData {
    bool isAutoRunning;
    DeviceMode currentMode;
    
    // S1 Arm — jog direction + position
    int armJogDir;        // -1, 0, +1
    float armPosition;    // Current absolute step position
    int armPosOut;        // Calibration: Out position (-1 = unset)
    int armPosIn;         // Calibration: In position (-1 = unset)

    // S2 Actuator — jog direction + position
    int actJogDir;        // -1, 0, +1
    int actuatorPercent;  // Current position percent
    int actuatorLimits[3];
    bool actuatorLimitSet[3];

    // S3 Z Motor — jog direction + position
    int zJogDir;          // -1, 0, +1
    float motorPos;
    float motorLimits[3];
    bool motorLimitSet[3];

    // S4 Menu state
    S4Level0 s4Menu;
    int s4SubMenu;
    bool s4InSubMenu;
    bool s4InSpeedEdit;

    // Configurable Speeds
    int armJogSpeed;
    int armGoSpeed;
    int actJogSpeed;
    int actGoSpeed;
    int zJogSpeed;
    int zGoSpeed;
};
