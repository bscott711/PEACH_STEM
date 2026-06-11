#pragma once
#include <Arduino.h>

class StorageManager {
public:
    static void init();

    // --- Actuator (Rotation) ---
    static void saveDishRotationPosition(float pos);
    static float loadDishRotationPosition();
    static void saveDishRotationJogSpeed(int speed);
    static int loadDishRotationJogSpeed(int defaultSpeed);
    static void saveDishRotationGoSpeed(int speed);
    static int loadDishRotationGoSpeed(int defaultSpeed);
    static void saveDishRotationNumRotations(int num);
    static int loadDishRotationNumRotations(int defaultNum);
    static void saveDishRotationSGThreshold(int sg);
    static int loadDishRotationSGThreshold(int defaultSg);

    // --- Motor (Lift) ---
    static void saveDishLiftPosHome(float pos);
    static void saveDishLiftPosTilt(float pos);
    static void loadDishLiftPositions(float &posHome, float &posTilt, bool &homeSet, bool &tiltSet);
    static void saveDishLiftState(bool isHomed, float pos);
    static void loadDishLiftState(bool &isHomed, float &pos);
    static void saveDishLiftJogSpeed(int speed);
    static int loadDishLiftJogSpeed(int defaultSpeed);
    static void saveDishLiftGoSpeed(int speed);
    static int loadDishLiftGoSpeed(int defaultSpeed);
    static void saveDishLiftNumMix(int num);
    static int loadDishLiftNumMix(int defaultNum);
    static void saveDishLiftSGThreshold(int sg);
    static int loadDishLiftSGThreshold(int defaultSg);

    // --- Arm ---
    static void saveScraperArmPosClear(int pos);
    static void saveScraperArmPosScrape(int pos);
    static void loadScraperArmCalibration(int &posClear, int &posScrape);
    static void saveScraperArmPosition(float pos);
    static float loadScraperArmPosition();
    static void saveScraperArmJogSpeed(int speed);
    static int loadScraperArmJogSpeed(int defaultSpeed);
    static void saveScraperArmGoSpeed(int speed);
    static int loadScraperArmGoSpeed(int defaultSpeed);
    static void saveScraperArmSGThreshold(int sg);
    static int loadScraperArmSGThreshold(int defaultSg);
    static void saveScraperArmDropPos(int pos);
    static int loadScraperArmDropPos();
    static void saveScraperArmTenCur(int current);
    static int loadScraperArmTenCur(int defaultCurrent);
};
