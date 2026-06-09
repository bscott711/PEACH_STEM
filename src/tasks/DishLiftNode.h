#pragma once
#include "core/StorageManager.h"
#include "drivers/MotorDriver.h"
#include "messaging.h"
#include "tasks/ActiveMotionNode.h"

/**
 * @brief Stepper motor control node using Active Object pattern.
 *
 * Encapsulates TMC2209 driver, limit management, SG4 homing state,
 * and float-based position tracking with lock-free message passing.
 */
class DishLiftNode
    : public ActiveMotionNode<DishLiftCommand, DishLiftTelemetry> {
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

  float trackingTarget;  // Absolute position target for GOTO
  bool isTrackingTarget; // True if GOTO active

  float posHome;
  float posTilt;
  bool posHomeSet;
  bool posTiltSet;

  // Arm interlock data (read from telemetry)
  int armStepPos;
  int armCalStart;
  int armBufferPos;
  int armInPos;

public:
  DishLiftNode();
  virtual ~DishLiftNode();

  // Override base class pure virtuals
  void hwInit() override;
  void processCommand(const DishLiftCommand &cmd) override;
  void hwUpdate() override;
  DishLiftTelemetry generateTelemetry() override;

  // Convenience methods for sending commands
  bool setSpeed(int speed);
  bool setTarget(float position, int speed);
  bool startHoming();
  bool setPosHome(float position);
  bool setPosTilt(float position);
  bool clearCal();

  // Getters for state
  float getPosition() const { return currentPosition; }
  bool getIsHomed() const { return isHomed; }
};
