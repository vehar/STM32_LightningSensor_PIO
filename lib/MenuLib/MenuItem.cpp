#include "MenuItem.h"
#include "Parameter.h"

MenuItem::MenuItem(const char *label, MenuItemType type, void (*action)(), Parameter *parameter)
    : label(label), type(type), action(action), parameter(parameter)
{
}

const char *MenuItem::getLabel() { return label; }

MenuItemType MenuItem::getType() { return type; }

void MenuItem::executeAction()
{
    if (action)
        action();
}

Parameter *MenuItem::getParameter() { return parameter; }
