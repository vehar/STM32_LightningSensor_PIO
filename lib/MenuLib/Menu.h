#ifndef MENU_H
#define MENU_H

#include "MenuItem.h"
#include <Arduino.h>

class Menu
{
public:
    Menu(const char *title, MenuItem *items[], int itemCount);
    const char *getTitle();
    MenuItem *getItem(int index);
    int getItemCount();

private:
    const char *title;
    MenuItem **items;
    int itemCount;
};

#endif
