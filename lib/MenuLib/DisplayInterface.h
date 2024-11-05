// DisplayInterface.h
#ifndef DISPLAY_INTERFACE_H
#define DISPLAY_INTERFACE_H

class DisplayInterface
{
public:
    virtual void clear() = 0;
    virtual void setTextSize(int size) = 0;
    virtual void setCursor(int x, int y) = 0;
    virtual void print(const char *text) = 0;
    virtual void print(int number) = 0;
    virtual void display() = 0;
    virtual ~DisplayInterface() = default;
};

#endif
