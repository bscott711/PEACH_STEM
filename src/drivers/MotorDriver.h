#pragma once
#include <TMC2209.h>

#define TXD1 32
#define RXD1 33

#define ENABLE_OPTICAL_ENDSTOPS 0
#define TOP_ENDSTOP_PIN 35
#define BOT_ENDSTOP_PIN 34

#define MOTOR_MIN_SAFE_STEPS 0
#define MOTOR_MAX_SAFE_STEPS 100000
#define MOTOR_MAX_SAFE_ACCEL 4000
#define RUN_CURRENT_PERCENT 100
#define SERIAL_BAUD_RATE 115200

#define TASK_UPDATE_MOTOR 10

class motorDriver {
public:
  void begin(HardwareSerial &serial, TMC2209::SerialAddress address);
  void setVelocity(int newSpeed);
  void stop();

private:
  TMC2209 driver;
};