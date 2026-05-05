#include "MotorDriver.h"

TMC2209 stepper_driver;

// Initialize Motor
void motorDriver::begin(HardwareSerial& serial, TMC2209::SerialAddress address)
{
    driver.setup(serial, SERIAL_BAUD_RATE, address, RXD1, TXD1);
    driver.setRunCurrent(RUN_CURRENT_PERCENT);
    driver.enableCoolStep();
    driver.enable();
};

void motorDriver::setVelocity(int newSpeed)
{
    // Constrain speed within bounds
    newSpeed = constrain(newSpeed, -MOTOR_MAX_SAFE_STEPS, MOTOR_MAX_SAFE_STEPS);

    if( newSpeed > 0 )
    {
        driver.disableInverseMotorDirection(); 
    }
    else
    {
        driver.enableInverseMotorDirection(); 
    }
    // Update Driver
    driver.moveAtVelocity(abs(newSpeed));
}

void motorDriver::stop()
{
    driver.moveAtVelocity(0);
}