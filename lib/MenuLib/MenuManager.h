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
    void waitTillButtonReleased(Button bt);
    void handleInput(Button button);
    void updateMenu();

private:
    Adafruit_SSD1306 &display;
    Menu *currentMenu;
    int currentIndex;
    int topIndex;
    static const int maxVisibleItems = 7;
    void displayMenu();
    Button debounceButton();
    void displayParameter(Parameter *parameter);
};

#endif
