#pragma once
#include <Arduino.h>

class StorageManager {
public:
    static void init();

    // --- Actuator ---
    static void saveActuatorLimitBot(int limit, bool isSet);
    static void saveActuatorLimitMid(int limit, bool isSet);
    static void saveActuatorLimitTop(int limit, bool isSet);
    static void loadActuatorLimits(int limits[3], bool limitSet[3]);
    static void saveActuatorPosition(float pos);
    static float loadActuatorPosition();

    // --- Motor ---
    static void saveMotorLimitBot(float limit, bool isSet);
    static void saveMotorLimitMid(float limit, bool isSet);
    static void saveMotorLimitTop(float limit, bool isSet);
    static void loadMotorLimits(float limits[3], bool limitSet[3]);
    static void saveMotorState(bool isHomed, float pos);
    static void loadMotorState(bool &isHomed, float &pos);

    // --- Arm ---
    static void saveArmPosOut(int pos);
    static void saveArmPosIn(int pos);
    static void loadArmCalibration(int &posOut, int &posIn);
    static void saveArmPosition(float pos);
    static float loadArmPosition();
};
