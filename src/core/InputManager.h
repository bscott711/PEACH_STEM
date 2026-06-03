#pragma once

class InputManager {
public:
    static void init();
    static void process();

private:
    static void handleArmEncoder();
    static void handleActuatorEncoder();
    static void handleMotorEncoder();
    static void handleAutonomousEncoder();
};
