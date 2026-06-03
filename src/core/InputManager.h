#pragma once
#include "core/UIData.h"

class InputManager {
public:
    static void init();
    static void process();
    static void populateUIData(UIData& data);

private:
    static void handleArmEncoder();
    static void handleActuatorEncoder();
    static void handleMotorEncoder();
    static void handleMenuEncoder();
};
