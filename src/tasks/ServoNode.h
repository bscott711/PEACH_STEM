#pragma once
#include "tasks/ActiveMotionNode.h"
#include "messaging.h"
#include <Preferences.h>

/**
 * @brief Servo control node using Active Object pattern.
 * 
 * Manages servo position with high-resolution float math,
 * NVS-persisted calibration, and lock-free message passing.
 */
class ServoNode : public ActiveMotionNode<ServoCommand, ServoTelemetry> {
private:
    // High-resolution position tracking (float percent)
    float currentPercent;
    float targetPercent;
    bool isActive;
    
    // Calibration data (NVS persisted)
    int calStart;
    int calCenter;
    
    // NVS storage
    Preferences preferences;
    
    // Motion ramping
    static constexpr float STEP_SIZE = 1.0f;  // Percent per tick
    
public:
    ServoNode();
    virtual ~ServoNode();
    
    // Override base class pure virtuals
    void hwInit() override;
    void processCommand(const ServoCommand& cmd) override;
    void hwUpdate() override;
    ServoTelemetry generateTelemetry() override;
    
    // Convenience methods for sending commands
    bool setTarget(float percent);
    bool activate();
    bool deactivate();
    bool setCalStart(int value);
    bool setCalCenter(int value);
    
    // Getters for calibration data
    int getCalStart() const { return calStart; }
    int getCalCenter() const { return calCenter; }
};
