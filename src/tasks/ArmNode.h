#pragma once
#include "tasks/ActiveMotionNode.h"
#include "messaging.h"
#include "drivers/MotorDriver.h"
#include <Preferences.h>

/**
 * @brief Arm Stepper motor control node using Active Object pattern.
 * 
 * Replaces the old Servo. Encapsulates a second TMC2209 driver (address 1).
 * Supports setting a Start and Center calibration boundary.
 */
class ArmNode : public ActiveMotionNode<ArmCommand, ArmTelemetry> {
private:
    motorDriver driver;
    
    float currentPosition;
    int targetSpeed;
    
    int calStart;
    int calCenter;
    
    bool isTrackingTarget;
    float targetTrackingAbsSteps;
    
    Preferences preferences;
    
public:
    ArmNode();
    virtual ~ArmNode();
    
    void hwInit() override;
    void processCommand(const ArmCommand& cmd) override;
    void hwUpdate() override;
    ArmTelemetry generateTelemetry() override;
    
    bool setSpeed(int speed);
    bool setTarget(float targetStep);
    bool setCalStart(int value);
    bool setCalCenter(int value);
};
