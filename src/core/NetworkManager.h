#pragma once
#include <Arduino.h>

class NetworkManager {
public:
    static void init();
    static void handle();

    static bool isOTAActive();
    static int getOTAProgress();
    static const char* getOTAStatus();
    
    // Custom Logger to bypass Arduino Core hijacking of ESP_LOG
    static void logToWiFi(const char* level, const char* tag, const char* format, ...);
};

// Global Logging Macros
#define PEACH_LOGI(tag, fmt, ...) NetworkManager::logToWiFi("I", tag, fmt, ##__VA_ARGS__)
#define PEACH_LOGE(tag, fmt, ...) NetworkManager::logToWiFi("E", tag, fmt, ##__VA_ARGS__)
#define PEACH_LOGD(tag, fmt, ...) NetworkManager::logToWiFi("D", tag, fmt, ##__VA_ARGS__)
#define PEACH_LOGW(tag, fmt, ...) NetworkManager::logToWiFi("W", tag, fmt, ##__VA_ARGS__)
