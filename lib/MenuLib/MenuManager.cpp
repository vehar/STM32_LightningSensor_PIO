#include "MenuManager.h"
#include "Parameter.h"
MenuManager::MenuManager(Adafruit_SSD1306 &display, Menu *rootMenu)
    : display(display), currentMenu(rootMenu), currentIndex(0)
{
}

void MenuManager::waitTillButtonReleased(Button bt)
{
    while (getPressedButton() == bt)
        delay(10); // Debounce delay
}

Button MenuManager::debounceButton()
{
    Button lastButton = getPressedButton();
    delay(50); // Short delay for debounce
    if (lastButton == getPressedButton())
        return lastButton;
    return BUTTON_NONE;
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
    updateMenu();
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

        Button bt = debounceButton();
        switch (bt)
        {
        case BUTTON_LEFT:
        case BUTTON_DOWN:
            parameter->decrement(); // TODO add mechanism for feedback if limit reached
            break;

        case BUTTON_RIGHT:
        case BUTTON_UP:
            parameter->increment(); // TODO add mechanism for feedback
            break;

        case BUTTON_CENTER:
            tuneFlag = false; // Exit adjustment mode
            break;

        default:
            break;
        }
    }
}

void MenuManager::updateMenu()
{
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(currentMenu->getTitle());

    int itemCount = currentMenu->getItemCount();
    currentIndex = constrain(currentIndex, 0, itemCount - 1);
    topIndex = max(0, min(topIndex, itemCount - maxVisibleItems));

    int endIndex = min(topIndex + maxVisibleItems, itemCount);
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
