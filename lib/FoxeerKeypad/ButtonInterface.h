
// ButtonInterface.h
#ifndef BUTTON_INTERFACE_H
#define BUTTON_INTERFACE_H

enum Button
{
    BUTTON_NONE = 1,
    BUTTON_UP,
    BUTTON_LEFT,
    BUTTON_DOWN,
    BUTTON_RIGHT,
    BUTTON_CENTER
};

class ButtonInterface
{
public:
    virtual Button getPressedButton() = 0;
    virtual ~ButtonInterface() = default;
};

#endif
