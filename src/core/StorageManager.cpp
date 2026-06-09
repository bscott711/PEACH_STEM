#include "StorageManager.h"
#include <Preferences.h>
#include "NetworkManager.h"
#include "esp_log.h"
#include <array>

static const char* TAG = "STORAGE";
static Preferences prefs;

void StorageManager::init() {
    if (!prefs.begin("peach", false)) {
        PEACH_LOGE(TAG, "Failed to open NVS namespace on init");
    }
}

// --- Actuator ---
void StorageManager::saveDishRotationLimit(LimitIndex idx, int limit, bool isSet) {
    static const std::array<const char*, 3> keys = {"actB", "actM", "actT"};
    static const std::array<const char*, 3> setKeys = {"actS_B", "actS_M", "actS_T"};
    if (idx >= 0 && idx < 3) {
        prefs.putInt(keys[idx], limit);
        prefs.putBool(setKeys[idx], isSet);
    }
}

void StorageManager::loadDishRotationLimits(int limits[3], bool limitSet[3]) {
    limits[0] = prefs.getInt("actB", 0);
    limits[1] = prefs.getInt("actM", 0);
    limits[2] = prefs.getInt("actT", 0);
    limitSet[0] = prefs.getBool("actS_B", false);
    limitSet[1] = prefs.getBool("actS_M", false);
    limitSet[2] = prefs.getBool("actS_T", false);
}

void StorageManager::saveDishRotationPosition(float pos) {
    prefs.putFloat("actPos", pos);
}

float StorageManager::loadDishRotationPosition() {
    return prefs.getFloat("actPos", 0.0f);
}

void StorageManager::saveDishRotationJogSpeed(int speed) {
    prefs.putInt("actJogSpd", speed);
}

int StorageManager::loadDishRotationJogSpeed(int defaultSpeed) {
    return prefs.getInt("actJogSpd", defaultSpeed);
}

void StorageManager::saveDishRotationGoSpeed(int speed) {
    prefs.putInt("actGoSpd", speed);
}

int StorageManager::loadDishRotationGoSpeed(int defaultSpeed) {
    return prefs.getInt("actGoSpd", defaultSpeed);
}

// --- Motor ---
void StorageManager::saveDishLiftLimit(LimitIndex idx, float limit, bool isSet) {
    static const std::array<const char*, 3> keys = {"limB", "limM", "limT"};
    static const std::array<const char*, 3> setKeys = {"limS_B", "limS_M", "limS_T"};
    if (idx >= 0 && idx < 3) {
        prefs.putFloat(keys[idx], limit);
        prefs.putBool(setKeys[idx], isSet);
    }
}

void StorageManager::loadDishLiftLimits(float limits[3], bool limitSet[3]) {
    limits[0] = prefs.getFloat("limB", 0.0f);
    limits[1] = prefs.getFloat("limM", 0.0f);
    limits[2] = prefs.getFloat("limT", 0.0f);
    limitSet[0] = prefs.getBool("limS_B", false);
    limitSet[1] = prefs.getBool("limS_M", false);
    limitSet[2] = prefs.getBool("limS_T", false);
}

void StorageManager::saveDishLiftState(bool isHomed, float pos) {
    prefs.putBool("isHomed", isHomed);
    prefs.putFloat("pos", pos);
}

void StorageManager::loadDishLiftState(bool &isHomed, float &pos) {
    isHomed = prefs.getBool("isHomed", false);
    pos = prefs.getFloat("pos", 0.0f);
}

void StorageManager::saveDishLiftJogSpeed(int speed) {
    prefs.putInt("zJogSpd", speed);
}

int StorageManager::loadDishLiftJogSpeed(int defaultSpeed) {
    return prefs.getInt("zJogSpd", defaultSpeed);
}

void StorageManager::saveDishLiftGoSpeed(int speed) {
    prefs.putInt("zGoSpd", speed);
}

int StorageManager::loadDishLiftGoSpeed(int defaultSpeed) {
    return prefs.getInt("zGoSpd", defaultSpeed);
}

// --- Arm ---
void StorageManager::saveScraperArmPosOut(int pos) {
    prefs.putInt("armPosO", pos);
}

void StorageManager::saveScraperArmPosIn(int pos) {
    prefs.putInt("armPosI", pos);
}

void StorageManager::saveScraperArmPosBuffer(int pos) {
    prefs.putInt("armBuf", pos);
}

int StorageManager::loadScraperArmPosBuffer() {
    return prefs.getInt("armBuf", -1);
}

void StorageManager::loadScraperArmCalibration(int &posOut, int &posIn) {
    posOut = prefs.getInt("armPosO", -1);
    posIn = prefs.getInt("armPosI", -1);
}

void StorageManager::saveScraperArmPosition(float pos) {
    prefs.putFloat("armPos", pos);
}

float StorageManager::loadScraperArmPosition() {
    return prefs.getFloat("armPos", 0.0f);
}

void StorageManager::saveScraperArmJogSpeed(int speed) {
    prefs.putInt("armJogSpd", speed);
}

int StorageManager::loadScraperArmJogSpeed(int defaultSpeed) {
    return prefs.getInt("armJogSpd", defaultSpeed);
}

void StorageManager::saveScraperArmGoSpeed(int speed) {
    prefs.putInt("armGoSpd", speed);
}

int StorageManager::loadScraperArmGoSpeed(int defaultSpeed) {
    return prefs.getInt("armGoSpd", defaultSpeed);
}
