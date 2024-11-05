#include "Parameter.h"

Parameter::Parameter(const char *name, int &value, int minValue, int maxValue)
    : name(name), value(value), minValue(minValue), maxValue(maxValue)
{
}

const char *Parameter::getName() { return name; }

int Parameter::getValue() { return value; }

int Parameter::increment(int scale)
{
    if (value + scale <= maxValue)
    {
        value += scale;
        return 0;
    }
    return -1; // Indicate limit reached
}

int Parameter::decrement(int scale)
{
    if (value - scale >= minValue)
    {
        value -= scale;
        return 0;
    }
    return -1; // Indicate limit reached
}
