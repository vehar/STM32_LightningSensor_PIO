#ifndef MENUMANAGER_H
#define MENUMANAGER_H

#include "DisplayInterface.h"
#include "Menu.h"
#include <Arduino.h>
#include <keypad.h>

class MenuManager
{
public:
    MenuManager(DisplayInterface &display, Menu *rootMenu);
    void waitTillButtonReleased(Button bt);
    void switchScale();
    void handleInput(Button button);
    void updateMenu();

private:
    DisplayInterface &display;
    Menu *currentMenu;
    int currentIndex;
    int topIndex;
    static const int maxVisibleItems = 7;
    void displayMenu();
    Button debounceButton();
    void clearAndSetupDisplay(int textSize, int cursorX = 0, int cursorY = 0);
    void displayText(const char *text);
    void displayParameterDetails(Parameter *parameter);
    void displayMenuItem(int index, const char *label, bool isSelected);

    void displayParameter(Parameter *parameter);

    int currentScale = 1; // Default to units of 1
};

#endif
