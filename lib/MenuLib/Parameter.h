#ifndef PARAMETER_H
#define PARAMETER_H

#include <Arduino.h>

class Parameter
{
public:
    Parameter(const char *name, int &value, int minValue, int maxValue);
    const char *getName();
    int getValue();
    int increment(int scale = 1);
    int decrement(int scale = 1);

private:
    const char *name;
    int &value;
    int minValue;
    int maxValue;
};

#endif
