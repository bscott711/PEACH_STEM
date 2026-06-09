#pragma once
#include <Arduino.h>

class StorageManager {
public:
    enum LimitIndex { LIMIT_BOT = 0, LIMIT_MID = 1, LIMIT_TOP = 2 };

    static void init();

    // --- Actuator ---
    static void saveDishRotationLimit(LimitIndex idx, int limit, bool isSet);
    static void loadDishRotationLimits(int limits[3], bool limitSet[3]);
    static void saveDishRotationPosition(float pos);
    static float loadDishRotationPosition();
    static void saveDishRotationJogSpeed(int speed);
    static int loadDishRotationJogSpeed(int defaultSpeed);
    static void saveDishRotationGoSpeed(int speed);
    static int loadDishRotationGoSpeed(int defaultSpeed);

    // --- Motor ---
    static void saveDishLiftLimit(LimitIndex idx, float limit, bool isSet);
    static void loadDishLiftLimits(float limits[3], bool limitSet[3]);
    static void saveDishLiftState(bool isHomed, float pos);
    static void loadDishLiftState(bool &isHomed, float &pos);
    static void saveDishLiftJogSpeed(int speed);
    static int loadDishLiftJogSpeed(int defaultSpeed);
    static void saveDishLiftGoSpeed(int speed);
    static int loadDishLiftGoSpeed(int defaultSpeed);

    // --- Arm ---
    static void saveScraperArmPosOut(int pos);
    static void saveScraperArmPosIn(int pos);
    static void loadScraperArmCalibration(int &posOut, int &posIn);
    static void saveScraperArmPosBuffer(int pos);
    static int loadScraperArmPosBuffer();
    static void saveScraperArmPosition(float pos);
    static float loadScraperArmPosition();
    static void saveScraperArmJogSpeed(int speed);
    static int loadScraperArmJogSpeed(int defaultSpeed);
    static void saveScraperArmGoSpeed(int speed);
    static int loadScraperArmGoSpeed(int defaultSpeed);
};
