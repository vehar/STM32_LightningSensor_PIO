#pragma once
// SSD1306_DisplayAdapter.h
#include "DisplayInterface.h"
#include <Adafruit_SSD1306.h>

class AdafruitDisplayAdapter : public DisplayInterface
{
public:
    AdafruitDisplayAdapter(Adafruit_SSD1306 &oledDisplay) : _display(oledDisplay) {}

    void clear() override { _display.clearDisplay(); }
    void setTextSize(int size) override { _display.setTextSize(size); }
    void setCursor(int x, int y) override { _display.setCursor(x, y); }
    void print(const char *text) override { _display.print(text); }
    void print(int number) override { _display.print(number); }
    void display() override { _display.display(); }

    // Additional methods to match Adafruit_SSD1306
    void setTextColor(uint16_t color) { _display.setTextColor(color); }
    void println(const char *text) { _display.println(text); }
    void println(const String &text) { _display.println(text.c_str()); }
    void println(int number) { _display.println(number); }

    bool begin(uint8_t switchvcc, uint8_t i2caddr) { return _display.begin(switchvcc, i2caddr); }
    void drawLine(int x0, int y0, int x1, int y1, uint16_t color)
    {
        _display.drawLine(x0, y0, x1, y1, color);
    }

private:
    Adafruit_SSD1306 &_display; // Renamed to avoid conflict with `display()`
};
