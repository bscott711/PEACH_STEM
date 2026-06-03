#pragma once
#include "tasks/ActiveMotionNode.h"
#include "messaging.h"
#include "drivers/MotorDriver.h"
#include "core/StorageManager.h"

/**
 * @brief Arm Stepper motor control node using Active Object pattern.
 * 
 * Controls a TMC2209 driver (address 1) to rotate a tube between two
 * calibrated positions: Out (away from pipette) and In (underneath pipette).
 * Supports encoder jog (proportional speed), long-press calibration,
 * and NVS-persisted positions.
 */
class ArmNode : public ActiveMotionNode<ArmCommand, ArmTelemetry> {
private:
    motorDriver driver;
    
    float currentPosition;   // Tracked absolute step position
    int targetSpeed;         // Current velocity command
    
    int posOut;              // Calibrated "Out" position in steps (-1 = unset)
    int posIn;               // Calibrated "In" position in steps (-1 = unset)
    
    bool isTrackingTarget;   // True when P-controller is driving to a target
    float targetTrackingAbsSteps;
    
    float lastSavedPosition; // Last position written to NVS (to debounce saves)
    
    
public:
    ArmNode();
    virtual ~ArmNode();
    
    void hwInit() override;
    void processCommand(const ArmCommand& cmd) override;
    void hwUpdate() override;
    ArmTelemetry generateTelemetry() override;
    
    bool setSpeed(int speed);
    bool stop();
    bool jog(float relativeSteps);
    bool setTarget(float percent);
    bool setPosOut();
    bool setPosIn();
    bool clearCal();
};
