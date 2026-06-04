#include "core/StorageManager.h"
#include <Preferences.h>
#include "esp_log.h"
#include <array>

static const char* TAG = "STORAGE";
static Preferences prefs;

void StorageManager::init() {
    if (!prefs.begin("peach", false)) {
        ESP_LOGE(TAG, "Failed to open NVS namespace on init");
    }
}

// --- Actuator ---
void StorageManager::saveActuatorLimit(LimitIndex idx, int limit, bool isSet) {
    static const std::array<const char*, 3> keys = {"actB", "actM", "actT"};
    static const std::array<const char*, 3> setKeys = {"actS_B", "actS_M", "actS_T"};
    if (idx >= 0 && idx < 3) {
        prefs.putInt(keys[idx], limit);
        prefs.putBool(setKeys[idx], isSet);
    }
}

void StorageManager::loadActuatorLimits(int limits[3], bool limitSet[3]) {
    limits[0] = prefs.getInt("actB", 0);
    limits[1] = prefs.getInt("actM", 0);
    limits[2] = prefs.getInt("actT", 0);
    limitSet[0] = prefs.getBool("actS_B", false);
    limitSet[1] = prefs.getBool("actS_M", false);
    limitSet[2] = prefs.getBool("actS_T", false);
}

void StorageManager::saveActuatorPosition(float pos) {
    prefs.putFloat("actPos", pos);
}

float StorageManager::loadActuatorPosition() {
    return prefs.getFloat("actPos", 0.0f);
}

void StorageManager::saveActuatorJogSpeed(int speed) {
    prefs.putInt("actJogSpd", speed);
}

int StorageManager::loadActuatorJogSpeed(int defaultSpeed) {
    return prefs.getInt("actJogSpd", defaultSpeed);
}

void StorageManager::saveActuatorGoSpeed(int speed) {
    prefs.putInt("actGoSpd", speed);
}

int StorageManager::loadActuatorGoSpeed(int defaultSpeed) {
    return prefs.getInt("actGoSpd", defaultSpeed);
}

// --- Motor ---
void StorageManager::saveMotorLimit(LimitIndex idx, float limit, bool isSet) {
    static const std::array<const char*, 3> keys = {"limB", "limM", "limT"};
    static const std::array<const char*, 3> setKeys = {"limS_B", "limS_M", "limS_T"};
    if (idx >= 0 && idx < 3) {
        prefs.putFloat(keys[idx], limit);
        prefs.putBool(setKeys[idx], isSet);
    }
}

void StorageManager::loadMotorLimits(float limits[3], bool limitSet[3]) {
    limits[0] = prefs.getFloat("limB", 0.0f);
    limits[1] = prefs.getFloat("limM", 0.0f);
    limits[2] = prefs.getFloat("limT", 0.0f);
    limitSet[0] = prefs.getBool("limS_B", false);
    limitSet[1] = prefs.getBool("limS_M", false);
    limitSet[2] = prefs.getBool("limS_T", false);
}

void StorageManager::saveMotorState(bool isHomed, float pos) {
    prefs.putBool("isHomed", isHomed);
    prefs.putFloat("pos", pos);
}

void StorageManager::loadMotorState(bool &isHomed, float &pos) {
    isHomed = prefs.getBool("isHomed", false);
    pos = prefs.getFloat("pos", 0.0f);
}

void StorageManager::saveZJogSpeed(int speed) {
    prefs.putInt("zJogSpd", speed);
}

int StorageManager::loadZJogSpeed(int defaultSpeed) {
    return prefs.getInt("zJogSpd", defaultSpeed);
}

void StorageManager::saveZGoSpeed(int speed) {
    prefs.putInt("zGoSpd", speed);
}

int StorageManager::loadZGoSpeed(int defaultSpeed) {
    return prefs.getInt("zGoSpd", defaultSpeed);
}

// --- Arm ---
void StorageManager::saveArmPosOut(int pos) {
    prefs.putInt("armPosO", pos);
}

void StorageManager::saveArmPosIn(int pos) {
    prefs.putInt("armPosI", pos);
}

void StorageManager::saveArmPosBuffer(int pos) {
    prefs.putInt("armBuf", pos);
}

int StorageManager::loadArmPosBuffer() {
    return prefs.getInt("armBuf", -1);
}

void StorageManager::loadArmCalibration(int &posOut, int &posIn) {
    posOut = prefs.getInt("armPosO", -1);
    posIn = prefs.getInt("armPosI", -1);
}

void StorageManager::saveArmPosition(float pos) {
    prefs.putFloat("armPos", pos);
}

float StorageManager::loadArmPosition() {
    return prefs.getFloat("armPos", 0.0f);
}

void StorageManager::saveArmJogSpeed(int speed) {
    prefs.putInt("armJogSpd", speed);
}

int StorageManager::loadArmJogSpeed(int defaultSpeed) {
    return prefs.getInt("armJogSpd", defaultSpeed);
}

void StorageManager::saveArmGoSpeed(int speed) {
    prefs.putInt("armGoSpd", speed);
}

int StorageManager::loadArmGoSpeed(int defaultSpeed) {
    return prefs.getInt("armGoSpd", defaultSpeed);
}
