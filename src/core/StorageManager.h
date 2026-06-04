#pragma once
#include <Arduino.h>

class StorageManager {
public:
    enum LimitIndex { LIMIT_BOT = 0, LIMIT_MID = 1, LIMIT_TOP = 2 };

    static void init();

    // --- Actuator ---
    static void saveActuatorLimit(LimitIndex idx, int limit, bool isSet);
    static void loadActuatorLimits(int limits[3], bool limitSet[3]);
    static void saveActuatorPosition(float pos);
    static float loadActuatorPosition();
    static void saveActuatorJogSpeed(int speed);
    static int loadActuatorJogSpeed(int defaultSpeed);
    static void saveActuatorGoSpeed(int speed);
    static int loadActuatorGoSpeed(int defaultSpeed);

    // --- Motor ---
    static void saveMotorLimit(LimitIndex idx, float limit, bool isSet);
    static void loadMotorLimits(float limits[3], bool limitSet[3]);
    static void saveMotorState(bool isHomed, float pos);
    static void loadMotorState(bool &isHomed, float &pos);
    static void saveZJogSpeed(int speed);
    static int loadZJogSpeed(int defaultSpeed);
    static void saveZGoSpeed(int speed);
    static int loadZGoSpeed(int defaultSpeed);

    // --- Arm ---
    static void saveArmPosOut(int pos);
    static void saveArmPosIn(int pos);
    static void loadArmCalibration(int &posOut, int &posIn);
    static void saveArmPosBuffer(int pos);
    static int loadArmPosBuffer();
    static void saveArmPosition(float pos);
    static float loadArmPosition();
    static void saveArmJogSpeed(int speed);
    static int loadArmJogSpeed(int defaultSpeed);
    static void saveArmGoSpeed(int speed);
    static int loadArmGoSpeed(int defaultSpeed);
};
