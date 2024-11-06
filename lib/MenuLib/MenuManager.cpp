#include "MenuManager.h"
#include "Parameter.h"
MenuManager::MenuManager(DisplayInterface &display, Menu *menu, ButtonInterface *buttonInterface)
    : display(display),
      currentMenu(menu),
      buttonInterface(buttonInterface),
      currentIndex(0),
      currentScale(1)
{
}

void MenuManager::waitTillButtonReleased(Button bt)
{
    while (buttonInterface->getPressedButton() == bt)
        delay(10);
}

Button MenuManager::debounceButton()
{
    Button lastButton =
        buttonInterface->getPressedButton(); // Use buttonInterface->getPressedButton()
    delay(50);                               // Short delay for debounce
    if (lastButton ==
        buttonInterface->getPressedButton()) // Use buttonInterface->getPressedButton()
        return lastButton;
    return BUTTON_NONE;
}

void MenuManager::clearAndSetupDisplay(int textSize, int cursorX, int cursorY)
{
    display.clear();
    display.setTextSize(textSize);
    display.setCursor(cursorX, cursorY);
}

void MenuManager::displayText(const char *text) { display.print(text); }

void MenuManager::displayParameterDetails(Parameter *parameter)
{
    clearAndSetupDisplay(2);
    displayText("SET x");
    display.print(currentScale);
    displayText("\n\n");

    displayText(parameter->getName());
    displayText("\n=  ");
    display.print(parameter->getValue()); // Display parameter value

    display.display();
}

void MenuManager::displayMenuItem(int index, const char *label, bool isSelected)
{
    display.setCursor(0, (index + 1) * 8); // Adjust y-position based on index
    if (isSelected)
        displayText("> ");
    else
        displayText("  ");
    displayText(label);
}

void MenuManager::switchScale()
{
    if (currentScale == 1)
        currentScale = 10;
    else if (currentScale == 10)
        currentScale = 100;
    else if (currentScale == 100)
        currentScale = 1000;
    else
        currentScale = 1;
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
        displayParameterDetails(parameter);
        // Display the current scale on a specific part of the screen

        Button bt = debounceButton();
        switch (bt)
        {
        case BUTTON_LEFT:
        case BUTTON_DOWN:
            parameter->decrement(currentScale);
            break;

        case BUTTON_UP:
            parameter->increment(currentScale);
            break;

        case BUTTON_CENTER:
            tuneFlag = false; // Exit adjustment mode
            break;

        case BUTTON_RIGHT: // A new button (e.g., long press or other) to switch scales
            switchScale(); // Switch to the next scale
            break;

        default:
            break;
        }
    }
}

void MenuManager::updateMenu()
{
    clearAndSetupDisplay(1);
    displayText(currentMenu->getTitle());

    int itemCount = currentMenu->getItemCount();
    currentIndex = constrain(currentIndex, 0, itemCount - 1);
    topIndex = max(0, min(topIndex, itemCount - maxVisibleItems));

    int endIndex = min(topIndex + maxVisibleItems, itemCount);
    for (int i = topIndex; i < endIndex; i++)
    {
        bool isSelected = (i == currentIndex);
        displayMenuItem(i - topIndex, currentMenu->getItem(i)->getLabel(), isSelected);
    }
    display.display();
}
