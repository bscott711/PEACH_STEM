#pragma once
#include "core/StorageManager.h"
#include "drivers/MotorDriver.h"
#include "messaging.h"
#include "tasks/ActiveMotionNode.h"

/**
 * @brief Arm Stepper motor control node using Active Object pattern.
 *
 * Controls a TMC2209 driver (address 1) to rotate a tube between two
 * calibrated positions: Out (away from pipette) and In (underneath pipette).
 * Supports encoder jog (proportional speed), long-press calibration,
 * and NVS-persisted positions.
 */
class ScraperArmNode
    : public ActiveMotionNode<ScraperArmCommand, ScraperArmTelemetry> {
private:
  motorDriver driver;

  float currentPosition; // Tracked absolute step position
  int targetSpeed;       // Current velocity command
  int previousTargetSpeed;

  int posOut;    // Calibrated "Out" position in steps (-1 = unset)
  int posBuffer; // Calibrated "Buffer" position in steps (-1 = unset)
  int posIn;     // Calibrated "In" position in steps (-1 = unset)

  bool isTrackingTarget; // True when driving to a target
  float targetTrackingAbsSteps;
  int targetTrackingSpeed; // The speed to use while tracking

  float lastSavedPosition; // Last position written to NVS (to debounce saves)

public:
  ScraperArmNode();
  virtual ~ScraperArmNode();

  void hwInit() override;
  void processCommand(const ScraperArmCommand &cmd) override;
  void hwUpdate() override;
  ScraperArmTelemetry generateTelemetry() override;

  bool setSpeed(int speed);
  bool stop();
  bool jog(float relativeSteps);
  bool setTarget(float percent, int targetSpeed = 5000);
  bool setPosOut();
  bool setPosBuffer();
  bool setPosIn();
  bool clearCal();
};
