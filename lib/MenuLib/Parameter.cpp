#include "Parameter.h"

Parameter::Parameter(const char *name, int &value, int minValue, int maxValue)
    : name(name), value(value), minValue(minValue), maxValue(maxValue)
{
}

const char *Parameter::getName() { return name; }

int Parameter::getValue() { return value; }

int Parameter::increment()
{
    if (value < maxValue)
    {
        value++;
        return 0;
    }

    return -1;
}

int Parameter::decrement()
{
    if (value > minValue)
    {
        value--;
        return 0;
    }

    return -1;
}
