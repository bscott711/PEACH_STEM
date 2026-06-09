#include "core/StorageManager.h"
#include <Preferences.h>
#include "core/NetworkManager.h"
#include "esp_log.h"
#include <array>

static const char* TAG = "STORAGE";
static Preferences prefs;

void StorageManager::init() {
    if (!prefs.begin("peach", false)) {
        PEACH_LOGE(TAG, "Failed to open NVS namespace on init");
    }
}

// --- Actuator (Rotation) ---
void StorageManager::saveDishRotationPosition(float pos) {
    prefs.putFloat("rotPos", pos);
}

float StorageManager::loadDishRotationPosition() {
    return prefs.getFloat("rotPos", 0.0f);
}

void StorageManager::saveDishRotationJogSpeed(int speed) {
    prefs.putInt("rotJogSpd", speed);
}

int StorageManager::loadDishRotationJogSpeed(int defaultSpeed) {
    return prefs.getInt("rotJogSpd", defaultSpeed);
}

void StorageManager::saveDishRotationGoSpeed(int speed) {
    prefs.putInt("rotGoSpd", speed);
}

int StorageManager::loadDishRotationGoSpeed(int defaultSpeed) {
    return prefs.getInt("rotGoSpd", defaultSpeed);
}

void StorageManager::saveDishRotationNumRotations(int num) {
    prefs.putInt("rotNum", num);
}

int StorageManager::loadDishRotationNumRotations(int defaultNum) {
    return prefs.getInt("rotNum", defaultNum);
}

void StorageManager::saveDishRotationSGThreshold(int sg) {
    prefs.putInt("rotSG", sg);
}

int StorageManager::loadDishRotationSGThreshold(int defaultSg) {
    return prefs.getInt("rotSG", defaultSg);
}

// --- Motor (Lift) ---
void StorageManager::saveDishLiftPosHome(float pos) {
    prefs.putFloat("limH", pos);
    prefs.putBool("limS_H", true);
}

void StorageManager::saveDishLiftPosTilt(float pos) {
    prefs.putFloat("limT", pos);
    prefs.putBool("limS_T", true);
}

void StorageManager::loadDishLiftPositions(float &posHome, float &posTilt, bool &homeSet, bool &tiltSet) {
    posHome = prefs.getFloat("limH", 0.0f);
    posTilt = prefs.getFloat("limT", 0.0f);
    homeSet = prefs.getBool("limS_H", false);
    tiltSet = prefs.getBool("limS_T", false);
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

void StorageManager::saveDishLiftNumMix(int num) {
    prefs.putInt("zMixNum", num);
}

int StorageManager::loadDishLiftNumMix(int defaultNum) {
    return prefs.getInt("zMixNum", defaultNum);
}

void StorageManager::saveDishLiftSGThreshold(int sg) {
    prefs.putInt("zSG", sg);
}

int StorageManager::loadDishLiftSGThreshold(int defaultSg) {
    return prefs.getInt("zSG", defaultSg);
}

// --- Arm ---
void StorageManager::saveScraperArmPosClear(int pos) {
    prefs.putInt("armPosC", pos);
}

void StorageManager::saveScraperArmPosScrape(int pos) {
    prefs.putInt("armPosS", pos);
}

void StorageManager::loadScraperArmCalibration(int &posClear, int &posScrape) {
    posClear = prefs.getInt("armPosC", -1);
    posScrape = prefs.getInt("armPosS", -1);
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

void StorageManager::saveScraperArmSGThreshold(int sg) {
    prefs.putInt("armSG", sg);
}

int StorageManager::loadScraperArmSGThreshold(int defaultSg) {
    return prefs.getInt("armSG", defaultSg);
}
