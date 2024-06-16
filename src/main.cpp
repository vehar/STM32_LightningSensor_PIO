#include <Arduino.h>
#include <Wire.h>
#include <cstddef>

#define LED_PIN PC13

void receiveEvent(int howMany)
{
    // Handle I2C receive event
}

void requestEvent()
{
    // Handle I2C request event
}

void setupPins()
{
    pinMode(LED_PIN, OUTPUT);
    // analogWriteFrequency(FAN_PWM_PIN, 10000); // This function is not needed
}

void setupADC()
{
    analogReadResolution(12);
    analogRead(PA0); // Dummy read to initialize ADC
}

void setupI2C()
{
    Wire.begin(0x33);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
}

void setup()
{
    setupPins();
    setupADC();
    setupI2C();
    Serial.begin(115200);
    digitalWrite(LED_PIN, LOW); // Indicate initialization complete
}

void loop()
{

}

