#include "MenuManager.h"
#include "Parameter.h"
MenuManager::MenuManager(Adafruit_SSD1306 &display, Menu *rootMenu)
    : display(display), currentMenu(rootMenu), currentIndex(0)
{
}

void MenuManager::waitTillButtonReleased(Button bt)
{
    Button button = getPressedButton();
    while (button == bt) // Wait till release
    {
        delay(10);
        button = getPressedButton();
    }
}

void MenuManager::handleInput(Button button)
{
    MenuItem *item = nullptr;

    waitTillButtonReleased(BUTTON_DOWN);

    switch (button)
    {
    case BUTTON_UP:
        currentIndex--;
        if (currentIndex < 0)
        {
            currentIndex = currentMenu->getItemCount() - 1;
            topIndex = max(0, currentMenu->getItemCount() - maxVisibleItems);
        }

        if (currentIndex < topIndex)
            topIndex--;
        break;
    case BUTTON_DOWN:
        currentIndex++;
        if (currentIndex >= currentMenu->getItemCount())
        {
            currentIndex = 0;
            topIndex = 0;
        }

        if (currentIndex >= topIndex + maxVisibleItems)
            topIndex++;
        break;

    case BUTTON_RIGHT:
    case BUTTON_CENTER:
        item = currentMenu->getItem(currentIndex);
        if (item != nullptr)
        {
            if (item->getType() == MENU_ITEM_ACTION)
                item->executeAction();
            else if (item->getType() == MENU_ITEM_PARAMETER)
                displayParameter(item->getParameter());
        }
        waitTillButtonReleased(BUTTON_CENTER);
        break;
    case BUTTON_LEFT:
        // Implement "Back" functionality if needed
        break;

    default:
        break;
    }
    updateDisplay();
}

void MenuManager::updateDisplay()
{
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(currentMenu->getTitle());

    int endIndex = min(topIndex + maxVisibleItems, currentMenu->getItemCount());
    for (int i = topIndex; i < endIndex; i++)
    {
        display.setCursor(0, (i - topIndex + 1) * 8);
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

    waitTillButtonReleased(BUTTON_CENTER);

    while (tuneFlag)
    {
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(0, 0);
        display.printf("SET\n%s\n=  %d", parameter->getName(), parameter->getValue());
        display.display();

        delay(100); // Debounce delay
        Button bt = getPressedButton();
        switch (bt)
        {
        case BUTTON_LEFT:
        case BUTTON_DOWN:
            parameter->decrement();
            break;

        case BUTTON_RIGHT:
        case BUTTON_UP:
            parameter->increment();
            break;

        case BUTTON_CENTER: // Apply and exit adjustment mode
            tuneFlag = false;
            break;
        }
    }
}