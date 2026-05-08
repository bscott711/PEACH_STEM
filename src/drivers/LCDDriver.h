#pragma once
#include "controller.h" // for SystemState, DeviceMode, ActuatorDirection
#include "drivers/EncoderDriver.h" // for g_encoderState
#include <Arduino.h>               // for millis(), uint32_t
#include <U8g2lib.h>

// SPI - OLED LCD
#define LCD_SCK 18
#define LCD_MOSI 23
#define LCD_CS 5
#define LCD_DC 17
#define LCD_RESET 16

// LCD Display Update Intervals (in ms)
#define TASK_REFRESH_LCD 100
#define LCD_UPDATE_MOTOR 100
#define LCD_UPDATE_ENCODER 100

// Shared action message buffer (for controller → LCD communication)
#define LCD_MSG_LEN 32
extern char lcdActionMessage[LCD_MSG_LEN];
extern volatile bool lcdMessagePending;
extern volatile uint32_t lcdMessageTimestamp; // ← Fixed: volatile here too

// Helper to set a temporary message (call from any task)
void LCD_setMessage(const char *msg);

// Helper to notify LCD of a button press (for visual flash)
void LCD_notifyButtonPress(int index);

void LCDInit();
void draw_menu();
void draw_displayTimer();