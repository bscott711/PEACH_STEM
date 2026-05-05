#include "LCDDriver.h"

// Instantiation for our LCD screen
// - ESP32 hardware SPI (VSPI)
// - U8g2 will use the default VSPI pins automatically for MOSI and SCLK
U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2 
(
  U8G2_R0,      // Rotation: U8G2_R0 = no rotation
  LCD_CS,       // CS (Chip Select)  (Any GPIO)
  LCD_DC,       // DC (Data/Command) (Any GPIO)
  LCD_RESET     // RESET             (Any GPIO)
);

uint32_t lcdStartTime = 0;

void LCDInit()
{
  lcdStartTime = millis();
  u8g2.begin();
  u8g2.setFont(u8g2_font_tiny5_tf);
}

void draw_displayTimer()
{
    uint32_t t = millis() / 1000;
    uint32_t m = t / 60;
    uint32_t s = t % 60;

    char buf[32];              // plenty of space
    snprintf(buf, sizeof(buf), "Runtime: %02u:%02u", m, s);

    u8g2.drawStr(78, 6, buf);  // top-right
}

// Helper Functions
void draw_menu()
{
    u8g2.clearBuffer();
    draw_displayTimer();
    u8g2.sendBuffer();  // Display Screen
}