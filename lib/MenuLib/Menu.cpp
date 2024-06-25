#include "Menu.h"

Menu::Menu(const char *title, MenuItem *items[], int itemCount)
    : title(title), items(items), itemCount(itemCount)
{
}

const char *Menu::getTitle() { return title; }

MenuItem *Menu::getItem(int index)
{
    if (index < 0 || index >= itemCount)
        return nullptr;
        
    return items[index];
}

int Menu::getItemCount() { return itemCount; }
