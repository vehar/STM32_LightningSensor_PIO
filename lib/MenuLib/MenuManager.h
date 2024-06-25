#ifndef MENUMANAGER_H
#define MENUMANAGER_H

#include "Menu.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <keypad.h>

class MenuManager
{
public:
    MenuManager(Adafruit_SSD1306 &display, Menu *rootMenu);
    void handleInput(Button button);
    void updateDisplay();

private:
    Adafruit_SSD1306 &display;
    Menu *currentMenu;
    int currentIndex;
    void displayMenu();
    void displayParameter(Parameter *parameter);
};

#endif
