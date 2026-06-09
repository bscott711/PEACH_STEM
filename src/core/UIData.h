#pragma once
#include "core/SystemState.h"

struct UIData {
    bool isAutoRunning;
    DeviceMode currentMode;
    
    // S1 Arm — jog direction + position
    int scraperArmJogDir;        // -1, 0, +1
    float scraperArmPosition;    // Current absolute step position
    int scraperArmPosOut;        // Calibration: Out position (-1 = unset)
    int scraperArmPosBuffer;     // Calibration: Buffer position (-1 = unset)
    int scraperArmPosIn;         // Calibration: In position (-1 = unset)

    // S2 Actuator — jog direction + position
    int dishRotationJogDir;        // -1, 0, +1
    int dishRotationPercent;  // Current position percent
    int dishRotationLimits[3];
    bool dishRotationLimitSet[3];

    // S3 Z Motor — jog direction + position
    int dishLiftJogDir;          // -1, 0, +1
    float dishLiftPos;
    float dishLiftLimits[3];
    bool dishLiftLimitSet[3];

    // S4 Menu state
    S4Level0 s4Menu;
    int s4SubMenu;
    bool s4InSubMenu;
    bool s4InSpeedEdit;

    // Configurable Speeds
    int scraperArmJogSpeed;
    int scraperArmGoSpeed;
    int dishRotationJogSpeed;
    int dishRotationGoSpeed;
    int dishLiftJogSpeed;
    int dishLiftGoSpeed;
};
