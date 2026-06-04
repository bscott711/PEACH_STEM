#pragma once
#include "tasks/ActiveMotionNode.h"
#include "messaging.h"
#include "drivers/MotorDriver.h"
#include "core/StorageManager.h"

/**
 * @brief Stepper motor control node using Active Object pattern.
 * 
 * Encapsulates TMC2209 driver, limit management, SG4 homing state,
 * and float-based position tracking with lock-free message passing.
 */
class MotorNode : public ActiveMotionNode<MotorCommand, MotorTelemetry> {
private:
    // TMC2209 driver instance
    motorDriver driver;
    
    // Position tracking (float units)
    float currentPosition;
    int targetSpeed;
    int previousTargetSpeed;
    
    // Homing and collision state
    bool isHomed;
    bool isHoming;
    bool motorLocked;
    
    // Optical endstop states
    bool topEndstopTriggered;
    bool botEndstopTriggered;

    float trackingTarget; // Absolute position target for GOTO
    bool isTrackingTarget; // True if GOTO active
    
    // Limit positions (NVS persisted)
    float limits[3];     // [0]=Bot, [1]=Mid, [2]=Top
    bool limitSet[3];    // Whether each limit is configured
    
    // NVS storage
    
    // Homing state machine
    enum HomingState { H_IDLE, H_MOVING_TOP, H_BACKOFF, H_MOVING_BOT };
    HomingState homingState;
    TickType_t homingStartTime;
    
    // Arm interlock data (read from telemetry)
    int armStepPos;
    int armCalStart;
    int armBufferPos;
    int armInPos;
    
public:
    MotorNode();
    virtual ~MotorNode();
    
    // Override base class pure virtuals
    void hwInit() override;
    void processCommand(const MotorCommand& cmd) override;
    void hwUpdate() override;
    MotorTelemetry generateTelemetry() override;
    
    // Convenience methods for sending commands
    bool setSpeed(int speed); // Note: Set speed to 0 for E-STOP
    bool setTarget(float position, int speed); // Note: Set speed > 0, direction is auto-calculated
    bool startHoming();
    bool setLimitBot(float position);
    bool setLimitMid(float position);
    bool setLimitTop(float position);
    bool clearLimitBot();
    bool clearLimitMid();
    bool clearLimitTop();
    
    // Getters for state
    float getPosition() const { return currentPosition; }
    bool getIsHomed() const { return isHomed; }
};
