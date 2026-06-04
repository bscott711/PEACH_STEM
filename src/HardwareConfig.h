#pragma once

/**
 * @file HardwareConfig.h
 * @brief Master hardware pinout and mapping for the PEACH PIT ESP32 system.
 * 
 * Centralizing pin definitions here prevents collisions and makes porting
 * to new ESP32 boards significantly easier.
 */

// ==========================================
// I2C Bus (Rotary Encoders / I2C Peripherals)
// ==========================================
#define I2C_SDA_PIN 26
#define I2C_SCL_PIN 27
#define ENCODER_INT_PIN 25
#define ENCODER_I2C_ADDR 0x49

// Seesaw GPIO pins for encoder buttons (from Adafruit datasheet)
#define SEESAW_BTN_ENC0 12
#define SEESAW_BTN_ENC1 14
#define SEESAW_BTN_ENC2 17
#define SEESAW_BTN_ENC3 9

// ==========================================
// SPI Bus (OLED/TFT Display)
// ==========================================
#define LCD_SCK     18
#define LCD_MOSI    23
#define LCD_CS      5
#define LCD_DC      21
#define LCD_RESET   22

// ==========================================
// Hardware Serial (TMC2209 Stepper Drivers)
// ==========================================
#define SERIAL_BAUD_RATE 115200
#define MOTOR_MIN_SAFE_STEPS 0
#define MOTOR_MAX_SAFE_STEPS 100000
#define MOTOR_MAX_SAFE_ACCEL 4000
#define RUN_CURRENT_PERCENT 100

// Serial1 mapped to RX2/TX2 pins on standard WROOM ESP32
// Swapped in software to handle straight-through wiring (Driver TX -> ESP TX2, Driver RX -> ESP RX2)
#define TXD1 16 // ESP Transmits on physical RX2 pin
#define RXD1 17 // ESP Receives on physical TX2 pin 

// ==========================================
// Z-Axis Optical Endstops
// ==========================================
#define ENABLE_OPTICAL_ENDSTOPS 0

#define TOP_ENDSTOP_PIN 35 // Input Only Pin
#define BOT_ENDSTOP_PIN 34 // Input Only Pin

// ==========================================
// Linear Actuator (DRV8871 H-Bridge)
// ==========================================
#define HB_IN1 33
#define HB_IN2 32
// PWM Channels for ledc
#define HB_PWM_CH_IN1 1
#define HB_PWM_CH_IN2 2
#define HB_PWM_FREQ 30000
#define HB_PWM_RES 8

