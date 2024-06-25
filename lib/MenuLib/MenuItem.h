#ifndef MENUITEM_H
#define MENUITEM_H

#include <Arduino.h>

enum MenuItemType
{
    MENU_ITEM_ACTION,
    MENU_ITEM_PARAMETER
};

class Parameter;

class MenuItem
{
public:
    MenuItem(const char *label, MenuItemType type, void (*action)() = nullptr,
             Parameter *parameter = nullptr);
    const char *getLabel();
    MenuItemType getType();
    void executeAction();
    Parameter *getParameter();

private:
    const char *label;
    MenuItemType type;
    void (*action)();
    Parameter *parameter;
};

#endif
