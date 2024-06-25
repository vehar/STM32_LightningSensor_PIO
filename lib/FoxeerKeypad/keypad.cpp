#include <keypad.h>

// Function to get the pressed button
Button getPressedButton()
{
    int analogValue = analogRead(KEYPAD_PIN);
    // char buffer[128]; // Adjust size as needed
    // sprintf(buffer, "ADC: %d \r\n", analogValue);
    // SerialUSB.print(buffer);

    for (int i = 0; i < buttonCount; i++)
    {
        if (abs(analogValue - buttonThresholds[i].threshold) <= TOLERANCE)
            return buttonThresholds[i].button;
    }

    return BUTTON_NONE;
}