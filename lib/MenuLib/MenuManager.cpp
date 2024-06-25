#include "MenuManager.h"
#include "Parameter.h"
MenuManager::MenuManager(Adafruit_SSD1306 &display, Menu *rootMenu)
    : display(display), currentMenu(rootMenu), currentIndex(0)
{
    // updateDisplay();
}

void MenuManager::handleInput(Button button)
{
    MenuItem *item = nullptr;

    switch (button)
    {
    case BUTTON_UP:
        currentIndex--;
        if (currentIndex < 0)
            currentIndex = currentMenu->getItemCount() - 1;

        updateDisplay();
        break;
    case BUTTON_DOWN:
        currentIndex++;
        if (currentIndex >= currentMenu->getItemCount())
            currentIndex = 0;

        updateDisplay();
        break;
    case BUTTON_CENTER:
        item = currentMenu->getItem(currentIndex);
        if (item != nullptr)
        {
            if (item->getType() == MENU_ITEM_ACTION)
                item->executeAction();
            else if (item->getType() == MENU_ITEM_PARAMETER)
                displayParameter(item->getParameter());

            updateDisplay(); // Return to the menu after adjusting
        }
        break;
    case BUTTON_LEFT:
        // Implement "Back" functionality if needed
        break;
    case BUTTON_RIGHT:
        // Implement any additional functionality if needed
        break;
    default:
        break;
    }
}

void MenuManager::updateDisplay()
{
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print(currentMenu->getTitle());
    for (int i = 0; i < currentMenu->getItemCount(); i++)
    {
        display.setCursor(0, (i + 1) * 8);
        if (i == currentIndex)
            display.print("> ");
        else
            display.print("  ");

        display.print(currentMenu->getItem(i)->getLabel());
    }
    display.display();
}

void MenuManager::displayParameter(Parameter *parameter)
{
    bool tuneFlag = true;
    Button button = getPressedButton();
    while (button != BUTTON_RIGHT)
    {
        delay(10); // Wait till release
        button = getPressedButton();
    }

    while (tuneFlag)
    {
        delay(100); // Debounce delay
        button = getPressedButton();
        switch (button)
        {
        case BUTTON_LEFT:
            if (parameter->decrement() == 0)
            {
                display.clearDisplay();
                display.setCursor(0, 0);
                display.printf("Adjust %s = %d", parameter->getName(), parameter->getValue());
                display.display();
            }
            break;

        case BUTTON_RIGHT:
            if (parameter->increment() == 0)
            {
                display.clearDisplay();
                display.setCursor(0, 0);
                display.printf("Adjust %s = %d", parameter->getName(), parameter->getValue());
                display.display();
            }
            break;

        case BUTTON_CENTER: // Exit adjustment mode
            tuneFlag = false;
            break;
        }
    }
}