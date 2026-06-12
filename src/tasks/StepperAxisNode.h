#pragma once

#include "tasks/ActiveMotionNode.h"
#include "messaging.h"
#include "core/SystemState.h"
#include "core/StorageManager.h"
#include "drivers/MotorDriver.h"

struct StepperAxisConfig {
    const char* axisName;
    HardwareSerial* serialPort;
    TMC2209::SerialAddress serialAddress;
    int rxPin;
    int txPin;
    int enPin;
    int diagPin;              // DIAG pin for hardware interrupt (if used)
    bool hasLimits;           // Does this axis have endstops/limits?
    float sgVelocityGatePercent; // Percentage of target speed to activate SG
    
    // NVS storage function pointers
    void (*savePositionFn)(float);
    float (*loadPositionFn)();
    
    void (*saveLimitAFn)(float);
    void (*saveLimitBFn)(float);
    
    // Load limits
    void (*loadLimitsFn)(float&, float&, bool&, bool&);

    // Initial StallGuard threshold loaded from NVS
    int initialSgThreshold;

    // FIX: Add function pointer to load SG threshold dynamically
    int (*loadSgThresholdFn)(int); 

    // Position integration multiplier (how many steps per second per unit of targetSpeed)
    float velocityMultiplier;
};

class StepperAxisNode : public ActiveMotionNode<AxisCommand, AxisTelemetry> {
protected:
    motorDriver driver;
    void updateVelocityGate(int speed);
    int currentGateVelocity = 0; // Current velocity threshold for SG

protected:
    StepperAxisConfig config;

    float currentPosition;
    int targetSpeed;
    int previousTargetSpeed;

    bool isTrackingTarget;
    float trackingTarget;
    int targetTrackingSpeed;

    float lastSavedPosition;

    // Limits
    float limitA;
    float limitB;
    bool limitASet;
    bool limitBSet;
    
    bool isHoming;
    bool isHomed;
    
    bool motorLocked;
    int currentSgThreshold; // StallGuard Threshold
    int lastPushedSg;
    uint16_t filteredSgResult;
    uint32_t movementStartTime;

    // Override this in derived classes to implement custom interlocks
    // Return true if movement should be blocked
    virtual bool checkInterlock(int desiredSpeed) { return false; }

public:
    StepperAxisNode(const StepperAxisConfig& cfg);
    virtual ~StepperAxisNode();

    virtual void hwInit() override;
    virtual void processCommand(const AxisCommand& cmd) override;
    virtual void hwUpdate() override;
    virtual AxisTelemetry generateTelemetry() override;

    // Helpers
    bool setSpeed(int speed);
    bool setTarget(float targetPos, int speed);
    bool jog(float relativeSteps);
    bool stop();
    bool setLimitA(float position);
    bool setLimitB(float position);
    bool clearLimits();
    bool zeroPosition();
    bool startHoming();
    bool setSGThreshold(int threshold);
    bool setCurrent(uint8_t currentPercent);
};
