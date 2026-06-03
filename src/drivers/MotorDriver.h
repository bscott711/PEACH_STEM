#pragma once
#include <TMC2209.h>
#include "HardwareConfig.h"

class motorDriver {
public:
  void begin(HardwareSerial &serial, TMC2209::SerialAddress address);
  void setVelocity(int newSpeed);
  void stop();

private:
  TMC2209 driver;
};