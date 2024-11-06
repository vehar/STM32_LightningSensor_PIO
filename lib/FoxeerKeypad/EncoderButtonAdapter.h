// EncoderButtonAdapter.h
#ifndef ENCODER_BUTTON_ADAPTER_H
#define ENCODER_BUTTON_ADAPTER_H

#include "ButtonInterface.h"
#include <Encoder.h>

class EncoderButtonAdapter : public ButtonInterface
{
public:
    EncoderButtonAdapter(int pinA, int pinB, int buttonPin)
        : encoder(pinA, pinB), buttonPin(buttonPin), lastPosition(0)
    {
        pinMode(buttonPin, INPUT_PULLUP); // Assume button uses a pull-up resistor
    }

    Button getPressedButton() override
    {
        // Check rotary encoder rotation
        long newPosition = encoder.read();
        if (newPosition > lastPosition)
        {
            lastPosition = newPosition;
            return BUTTON_UP;
        }
        else if (newPosition < lastPosition)
        {
            lastPosition = newPosition;
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
    Encoder encoder;
    int buttonPin;
    long lastPosition;
};

#endif // ENCODER_BUTTON_ADAPTER_H
