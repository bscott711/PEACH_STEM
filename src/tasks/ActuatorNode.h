#pragma once
#include "tasks/ActiveMotionNode.h"
#include "messaging.h"
#include "core/StorageManager.h"

/**
 * @brief Linear actuator control node using Active Object pattern.
 * 
 * Manages H-bridge driven actuator with float-based percentage tracking,
 * NVS-persisted limits (Bot/Mid/Top), and lock-free message passing.
 */
class ActuatorNode : public ActiveMotionNode<ActuatorCommand, ActuatorTelemetry> {
private:
    // High-resolution position tracking (float percent for smooth ramping)
    float currentPercent;
    int targetPercent;
    float lastSavedPercent;
    
    // Limit positions (NVS persisted)
    int limits[3];       // [0]=Bot, [1]=Mid, [2]=Top
    bool limitSet[3];    // Whether each limit is configured
    
    // NVS storage
    
    // Motion parameters
    static constexpr uint32_t FULL_EXTEND_TIME_MS = 800;  // Time 0% to 100%
    
public:
    ActuatorNode();
    virtual ~ActuatorNode();
    
    // Override base class pure virtuals
    void hwInit() override;
    void processCommand(const ActuatorCommand& cmd) override;
    void hwUpdate() override;
    ActuatorTelemetry generateTelemetry() override;
    
    // Convenience methods for sending commands
    bool setTarget(int percent);
    bool setLimitBot(int percent);
    bool setLimitMid(int percent);
    bool setLimitTop(int percent);
    bool clearLimitBot();
    bool clearLimitMid();
    bool clearLimitTop();
    
    // Getters for limit data
    int getLimit(int index) const { return limits[index]; }
    bool isLimitSet(int index) const { return limitSet[index]; }
};
