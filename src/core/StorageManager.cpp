#include "core/StorageManager.h"
#include <Preferences.h>
#include "esp_log.h"

static const char* TAG = "STORAGE";
static Preferences prefs;

void StorageManager::init() {
    if (!prefs.begin("peach", false)) {
        ESP_LOGE(TAG, "Failed to open NVS namespace on init");
    }
    prefs.end();
}

// --- Actuator ---
void StorageManager::saveActuatorLimitBot(int limit, bool isSet) {
    if (prefs.begin("peach", false)) {
        prefs.putInt("actB", limit);
        prefs.putBool("actS_B", isSet);
        prefs.end();
    }
}

void StorageManager::saveActuatorLimitMid(int limit, bool isSet) {
    if (prefs.begin("peach", false)) {
        prefs.putInt("actM", limit);
        prefs.putBool("actS_M", isSet);
        prefs.end();
    }
}

void StorageManager::saveActuatorLimitTop(int limit, bool isSet) {
    if (prefs.begin("peach", false)) {
        prefs.putInt("actT", limit);
        prefs.putBool("actS_T", isSet);
        prefs.end();
    }
}

void StorageManager::loadActuatorLimits(int limits[3], bool limitSet[3]) {
    if (prefs.begin("peach", false)) {
        limits[0] = prefs.getInt("actB", 0);
        limits[1] = prefs.getInt("actM", 0);
        limits[2] = prefs.getInt("actT", 0);
        limitSet[0] = prefs.getBool("actS_B", false);
        limitSet[1] = prefs.getBool("actS_M", false);
        limitSet[2] = prefs.getBool("actS_T", false);
        prefs.end();
    }
}

void StorageManager::saveActuatorPosition(float pos) {
    if (prefs.begin("peach", false)) {
        prefs.putFloat("actPos", pos);
        prefs.end();
    }
}

float StorageManager::loadActuatorPosition() {
    float pos = 0.0f;
    if (prefs.begin("peach", false)) {
        pos = prefs.getFloat("actPos", 0.0f);
        prefs.end();
    }
    return pos;
}



void StorageManager::saveActuatorJogSpeed(int speed) {
    if (prefs.begin("peach", false)) {
        prefs.putInt("actJogSpd", speed);
        prefs.end();
    }
}

int StorageManager::loadActuatorJogSpeed(int defaultSpeed) {
    int speed = defaultSpeed;
    if (prefs.begin("peach", false)) {
        speed = prefs.getInt("actJogSpd", defaultSpeed);
        prefs.end();
    }
    return speed;
}

void StorageManager::saveActuatorGoSpeed(int speed) {
    if (prefs.begin("peach", false)) {
        prefs.putInt("actGoSpd", speed);
        prefs.end();
    }
}

int StorageManager::loadActuatorGoSpeed(int defaultSpeed) {
    int speed = defaultSpeed;
    if (prefs.begin("peach", false)) {
        speed = prefs.getInt("actGoSpd", defaultSpeed);
        prefs.end();
    }
    return speed;
}

// --- Motor ---
void StorageManager::saveMotorLimitBot(float limit, bool isSet) {
    if (prefs.begin("peach", false)) {
        prefs.putFloat("limB", limit);
        prefs.putBool("limS_B", isSet);
        prefs.end();
    }
}

void StorageManager::saveMotorLimitMid(float limit, bool isSet) {
    if (prefs.begin("peach", false)) {
        prefs.putFloat("limM", limit);
        prefs.putBool("limS_M", isSet);
        prefs.end();
    }
}

void StorageManager::saveMotorLimitTop(float limit, bool isSet) {
    if (prefs.begin("peach", false)) {
        prefs.putFloat("limT", limit);
        prefs.putBool("limS_T", isSet);
        prefs.end();
    }
}

void StorageManager::loadMotorLimits(float limits[3], bool limitSet[3]) {
    if (prefs.begin("peach", false)) {
        limits[0] = prefs.getFloat("limB", 0.0f);
        limits[1] = prefs.getFloat("limM", 0.0f);
        limits[2] = prefs.getFloat("limT", 0.0f);
        limitSet[0] = prefs.getBool("limS_B", false);
        limitSet[1] = prefs.getBool("limS_M", false);
        limitSet[2] = prefs.getBool("limS_T", false);
        prefs.end();
    }
}

void StorageManager::saveMotorState(bool isHomed, float pos) {
    if (prefs.begin("peach", false)) {
        prefs.putBool("isHomed", isHomed);
        prefs.putFloat("pos", pos);
        prefs.end();
    }
}

void StorageManager::loadMotorState(bool &isHomed, float &pos) {
    if (prefs.begin("peach", false)) {
        isHomed = prefs.getBool("isHomed", false);
        pos = prefs.getFloat("pos", 0.0f);
        prefs.end();
    }
}

void StorageManager::saveZJogSpeed(int speed) {
    if (prefs.begin("peach", false)) {
        prefs.putInt("zJogSpd", speed);
        prefs.end();
    }
}

int StorageManager::loadZJogSpeed(int defaultSpeed) {
    int speed = defaultSpeed;
    if (prefs.begin("peach", false)) {
        speed = prefs.getInt("zJogSpd", defaultSpeed);
        prefs.end();
    }
    return speed;
}

void StorageManager::saveZGoSpeed(int speed) {
    if (prefs.begin("peach", false)) {
        prefs.putInt("zGoSpd", speed);
        prefs.end();
    }
}

int StorageManager::loadZGoSpeed(int defaultSpeed) {
    int speed = defaultSpeed;
    if (prefs.begin("peach", false)) {
        speed = prefs.getInt("zGoSpd", defaultSpeed);
        prefs.end();
    }
    return speed;
}

// --- Arm ---
void StorageManager::saveArmPosOut(int pos) {
    if (prefs.begin("peach", false)) {
        prefs.putInt("armPosO", pos);
        prefs.end();
    }
}

void StorageManager::saveArmPosIn(int pos) {
    if (prefs.begin("peach", false)) {
        prefs.putInt("armPosI", pos);
        prefs.end();
    }
}

void StorageManager::loadArmCalibration(int &posOut, int &posIn) {
    if (prefs.begin("peach", false)) {
        posOut = prefs.getInt("armPosO", -1);
        posIn = prefs.getInt("armPosI", -1);
        prefs.end();
    }
}

void StorageManager::saveArmPosition(float pos) {
    if (prefs.begin("peach", false)) {
        prefs.putFloat("armPos", pos);
        prefs.end();
    }
}

float StorageManager::loadArmPosition() {
    float pos = 0.0f;
    if (prefs.begin("peach", false)) {
        pos = prefs.getFloat("armPos", 0.0f);
        prefs.end();
    }
    return pos;
}

void StorageManager::saveArmJogSpeed(int speed) {
    if (prefs.begin("peach", false)) {
        prefs.putInt("armJogSpd", speed);
        prefs.end();
    }
}

int StorageManager::loadArmJogSpeed(int defaultSpeed) {
    int speed = defaultSpeed;
    if (prefs.begin("peach", false)) {
        speed = prefs.getInt("armJogSpd", defaultSpeed);
        prefs.end();
    }
    return speed;
}

void StorageManager::saveArmGoSpeed(int speed) {
    if (prefs.begin("peach", false)) {
        prefs.putInt("armGoSpd", speed);
        prefs.end();
    }
}

int StorageManager::loadArmGoSpeed(int defaultSpeed) {
    int speed = defaultSpeed;
    if (prefs.begin("peach", false)) {
        speed = prefs.getInt("armGoSpd", defaultSpeed);
        prefs.end();
    }
    return speed;
}
