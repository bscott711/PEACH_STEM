#pragma once
#include <U8g2lib.h>

// SPI - OLED LCD
#define LCD_SCK   18  // Default hardware SPI (no other choice)
#define LCD_MOSI  23  // Default hardware SPI (no other choice)
#define LCD_CS    5   // Any GPIO
#define LCD_DC    17  // Any GPIO
#define LCD_RESET 16  // Any GPIO

// LCD Display Update Intervals (in ms)
// - Updates respective variable(s) on the LCD screen every n ms.
// - Some variables update very quickly; hard to read on the LCD. (Slows it down)
#define TASK_REFRESH_LCD    100  // LCD Overall Refresh rate
#define LCD_UPDATE_MOTOR    100
#define LCD_UPDATE_ENCODER  100

void LCDInit();
void draw_menu();
void draw_displayTimer();