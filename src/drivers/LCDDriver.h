#pragma once
#include "controller.h"
#include "core/UIData.h"
#include "drivers/EncoderDriver.h"
#include <Arduino.h>
#include <U8g2lib.h>
#include "HardwareConfig.h"

// LCD Display Update Intervals (in ms)
#define TASK_REFRESH_LCD 100

// Helper to set a temporary message (call from any task)
void LCD_setMessage(const char *msg);

// Helper to notify LCD of a button press (for visual flash)
void LCD_notifyButtonPress(int index);

void LCDInit();
void draw_menu(const UIData& data);