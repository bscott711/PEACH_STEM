#pragma once
#include "controller.h"
#include "drivers/EncoderDriver.h"
#include <Arduino.h>
#include <U8g2lib.h>

// SPI - OLED/TFT LCD
#define LCD_SCK 18
#define LCD_MOSI 23
#define LCD_CS 5
#define LCD_DC 21
#define LCD_RESET 22

// LCD Display Update Intervals (in ms)
#define TASK_REFRESH_LCD 100

// Helper to set a temporary message (call from any task)
void LCD_setMessage(const char *msg);

// Helper to notify LCD of a button press (for visual flash)
void LCD_notifyButtonPress(int index);

void LCDInit();
void draw_menu();