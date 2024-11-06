// AnalogButtonAdapter.h
#ifndef ANALOG_BUTTON_ADAPTER_H
#define ANALOG_BUTTON_ADAPTER_H

#include "ButtonInterface.h"
#include <Arduino.h>

#define KEYPAD_PIN A0
const int TOLERANCE = 20; // Tolerance for analog readings

// Thresholds for analog values (specific to your resistor configuration)
const int UP_THRESHOLD = 389;
const int LEFT_THRESHOLD = 520;
const int DOWN_THRESHOLD = 0;
const int RIGHT_THRESHOLD = 235;
const int CENTER_THRESHOLD = 628;

// Struct to associate buttons with thresholds
struct ButtonThreshold
{
    Button button;
    int threshold;
};

// Define button thresholds
const ButtonThreshold buttonThresholds[] = { { BUTTON_UP, UP_THRESHOLD },
                                             { BUTTON_LEFT, LEFT_THRESHOLD },
                                             { BUTTON_DOWN, DOWN_THRESHOLD },
                                             { BUTTON_RIGHT, RIGHT_THRESHOLD },
                                             { BUTTON_CENTER, CENTER_THRESHOLD } };

class AnalogButtonAdapter : public ButtonInterface
{
public:
    AnalogButtonAdapter(int pin, const ButtonThreshold *thresholds, int count, int tolerance)
        : pin(pin), thresholds(thresholds), count(count), tolerance(tolerance)
    {
    }

    Button getPressedButton() override
    {
        int analogValue = analogRead(pin);
        for (int i = 0; i < count; i++)
        {
            if (abs(analogValue - thresholds[i].threshold) <= tolerance)
                return thresholds[i].button;
        }
        return BUTTON_NONE;
    }

private:
    int pin;
    const ButtonThreshold *thresholds;
    int count;
    int tolerance;
};

#endif
