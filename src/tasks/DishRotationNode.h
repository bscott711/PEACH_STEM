#pragma once
#include "tasks/ActiveMotionNode.h"
#include "core/StorageManager.h"
#include "drivers/MotorDriver.h"
#include "messaging.h"

/**
 * @brief Dish Rotation Stepper motor control node using Active Object pattern.
 *
 * Encapsulates TMC2209 driver (Address 2) with float-based position tracking 
 * and lock-free message passing.
 */
class DishRotationNode
    : public ActiveMotionNode<DishRotationCommand, DishRotationTelemetry> {
private:
  motorDriver driver;

  // Position tracking (float units)
  float currentPosition;
  int targetSpeed;
  int previousTargetSpeed;

  float trackingTarget;  // Absolute position target for GOTO
  bool isTrackingTarget; // True if GOTO active
  int targetTrackingSpeed;

  float lastSavedPosition;

public:
  DishRotationNode();
  virtual ~DishRotationNode();

  // Override base class pure virtuals
  void hwInit() override;
  void processCommand(const DishRotationCommand &cmd) override;
  void hwUpdate() override;
  DishRotationTelemetry generateTelemetry() override;

  // Convenience methods for sending commands
  bool setSpeed(int speed);
  bool stop();
  bool jog(float relativeSteps);
  bool setTarget(float position, int speed);
  bool zeroPosition();
};
