#pragma once
#include "ActiveMotionNode.h"
#include "core/StorageManager.h"
#include "messaging.h"

/**
 * @brief Linear actuator control node using Active Object pattern.
 *
 * Manages H-bridge driven actuator with float-based percentage tracking,
 * NVS-persisted limits (Bot/Mid/Top), and lock-free message passing.
 */
class DishRotationNode
    : public ActiveMotionNode<DishRotationCommand, DishRotationTelemetry> {
private:
  // High-resolution position tracking (float percent for smooth ramping)
  float currentPercent;
  int targetPercent;
  int targetSpeedPWM;
  float lastSavedPercent;
  bool wasMoving;

  // Limit positions (NVS persisted)
  int limits[3];    // [0]=Bot, [1]=Mid, [2]=Top
  bool limitSet[3]; // Whether each limit is configured

  // NVS storage

  // Motion parameters
  static constexpr uint32_t FULL_EXTEND_TIME_MS = 800; // Time 0% to 100%
  static constexpr uint8_t MIN_ACTUATOR_PWM =
      155; // Deadband: Minimum PWM required to physically move the actuator

public:
  DishRotationNode();
  virtual ~DishRotationNode();

  // Override base class pure virtuals
  void hwInit() override;
  void processCommand(const DishRotationCommand &cmd) override;
  void hwUpdate() override;
  DishRotationTelemetry generateTelemetry() override;

  // Convenience methods for sending commands
  bool setTarget(int percent, int pwmSpeed = 255);
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
