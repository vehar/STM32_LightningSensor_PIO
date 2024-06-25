#ifndef KEYPAD_H
#define KEYPAD_H

#include <Arduino.h>

#define KEYPAD_PIN A0

// Function to get the pressed button
// Thresholds for analog values with a 22kΩ pull-up resistor
const int UP_THRESHOLD = 389;     // Approximate analog value for UP button
const int LEFT_THRESHOLD = 520;   // Approximate analog value for LEFT button
const int DOWN_THRESHOLD = 0;     // Approximate analog value for DOWN button
const int RIGHT_THRESHOLD = 235;  // Approximate analog value for RIGHT button
const int CENTER_THRESHOLD = 628; // Approximate analog value for CENTER button

const int TOLERANCE = 20; // Tolerance for analog readings

// Enum for button states
enum Button
{
    BUTTON_NONE = 1,
    BUTTON_UP,
    BUTTON_LEFT,
    BUTTON_DOWN,
    BUTTON_RIGHT,
    BUTTON_CENTER
};

// Structure to hold button threshold data
struct ButtonThreshold
{
    Button button;
    int threshold;
};

// Array of button thresholds
const ButtonThreshold buttonThresholds[] = { { BUTTON_UP, UP_THRESHOLD },
                                             { BUTTON_LEFT, LEFT_THRESHOLD },
                                             { BUTTON_DOWN, DOWN_THRESHOLD },
                                             { BUTTON_RIGHT, RIGHT_THRESHOLD },
                                             { BUTTON_CENTER, CENTER_THRESHOLD } };

const int buttonCount = sizeof(buttonThresholds) / sizeof(buttonThresholds[0]);

Button getPressedButton();

#endif