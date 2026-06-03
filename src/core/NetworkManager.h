#pragma once
#include <Arduino.h>

class NetworkManager {
public:
    static void init();
    static void handle();

    static bool isOTAActive();
    static int getOTAProgress();
    static const char* getOTAStatus();
};
