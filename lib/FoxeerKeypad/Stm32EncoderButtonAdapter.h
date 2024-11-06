// Stm32EncoderButtonAdapter.h
#ifndef STM32_ENCODER_BUTTON_ADAPTER_H
#define STM32_ENCODER_BUTTON_ADAPTER_H

#include "ButtonInterface.h"
#include <Rotary.h> // Include the Rotary encoder library

class Stm32EncoderButtonAdapter : public ButtonInterface
{
public:
    Stm32EncoderButtonAdapter(int pinA, int pinB, int buttonPin)
        : encoder(pinA, pinB), buttonPin(buttonPin), lastPosition(0)
    {
        pinMode(buttonPin, INPUT_PULLUP); // Assume button uses a pull-up resistor
    }

    Button getPressedButton() override
    {
        // Check rotary encoder rotation
        unsigned char result = encoder.process();
        if (result == DIR_CW)
        {
            lastPosition++;
            return BUTTON_UP;
        }
        else if (result == DIR_CCW)
        {
            lastPosition--;
            return BUTTON_DOWN;
        }

        // Check encoder button press
        if (digitalRead(buttonPin) == LOW)
        {
            return BUTTON_CENTER;
        }

        return BUTTON_NONE;
    }

private:
    Rotary encoder; // Use the Rotary library
    int buttonPin;
    int lastPosition;
};

#endif // STM32_ENCODER_BUTTON_ADAPTER_H
