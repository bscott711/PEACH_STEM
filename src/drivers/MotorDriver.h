#pragma once
#include <TMC2209.h>

// TMC2209 Driver (UART1)
// - Note: with UART we can chain and independently control 4 drivers from one pin
#define TXD1 32  // Any GPIO
#define RXD1 33  // Any GPIO (Don't need to wrire, but library requires you to define)
#define MOTOR_MIN_SAFE_STEPS       0  // Min motor speed (Microsteps/second)
#define MOTOR_MAX_SAFE_STEPS  100000  // Max motor speed (Microsteps/second)
#define MOTOR_MAX_SAFE_ACCEL    4000
#define RUN_CURRENT_PERCENT      100
#define SERIAL_BAUD_RATE      115200  // UART BAUD Rate

// Stepper Motor Configs
#define TASK_UPDATE_MOTOR    10  // Update the stepper motor every n ms

class motorDriver
{
    public:
        void begin(HardwareSerial& serial, TMC2209::SerialAddress address);
        void setVelocity(int newSpeed);
        void stop();
    
    private:
        TMC2209 driver;
};