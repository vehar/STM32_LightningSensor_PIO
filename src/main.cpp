#include <AS3935.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Wire.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C
#define LED_PIN PC13

#define AS3935_IRQ_PIN PB5
#define AS3935_I2C_ADDR 0x01

struct AS3935Registers
{
    int noiseFloor;
    int spikeRejection;
    int watchdogThreshold;
    int distance;
    int minNumberOfLightnings;
    int afe;
    int trco;
    int srco;
    int capVal;
};

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
AS3935 as3935(Wire, AS3935_IRQ_PIN, AS3935_I2C_ADDR);

void handleLightning();
volatile int AS3935IrqTriggered = 0;

void setup();
void loop();
void scanI2CDevices();
void displayMessage(const String &message, int textSize);
void displayLightningInfo(uint8_t dist, long energy);
AS3935Registers getAS3935Registers();
void displayAS3935Registers(AS3935Registers regs);
void printAS3935Registers(AS3935Registers regs);

void setup()
{
    AS3935Registers regs;
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // Turn off the built-in LED

    SerialUSB.begin(115200);
    SerialUSB.setTimeout(10000);
    delay(3000); // Delay to allow USB connection to establish

    Wire.setSCL(PB6);
    Wire.setSDA(PB7);
    Wire.begin();

    scanI2CDevices();

    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR))
    {
        while (1)
            ;
    }

    display.display();
    delay(500);

    SerialUSB.println(F("System online."));

    displayMessage("Lightning sensor!", 1);
    SerialUSB.println(F("Display initialized and message displayed."));

    if (!as3935.begin())
    {
        SerialUSB.println("AS3935 initialization failed!");
        while (1)
            ;
    }

    as3935.reset();
    regs = getAS3935Registers();
    printAS3935Registers(regs);
    displayAS3935Registers(regs);

    as3935.setOutdoors();
    as3935.enableDisturbers();
    as3935.setNoiseFloor(1);
    as3935.setMinimumLightnings(0);

    if (!as3935.calibrate())
        SerialUSB.println("Tuning out of range, check your wiring, your sensor!");
    else
        SerialUSB.println("Calibration done!");

    delay(2000);
    regs = getAS3935Registers();
    printAS3935Registers(regs);
    displayAS3935Registers(regs);

    AS3935IrqTriggered = 0;
    attachInterrupt(digitalPinToInterrupt(AS3935_IRQ_PIN), handleLightning, RISING);
    SerialUSB.println("AS3935 initialized and ready.");

    delay(2000);
    displayLightningInfo(50, 2000000);
    delay(2000);
}

void handleNoiseInterrupt()
{
    SerialUSB.println("Noise level too high, try adjusting noise floor");
}

void handleDisturberInterrupt() { SerialUSB.println("Disturber detected"); }

void handleLightningInterrupt()
{
    int distance = as3935.lightningDistanceKm();
    long energy = as3935.lightningEnergy();

    if (distance == 1)
    {
        SerialUSB.println("Storm overhead, watch out!");
    }
    else if (distance == 63)
    {
        SerialUSB.println("Out of range lightning detected.");
    }
    else
    {
        SerialUSB.print("Lightning detected ");
        SerialUSB.print(distance);
        SerialUSB.println(" kilometers away.");
        SerialUSB.print(energy);
        SerialUSB.println(" energy of 2 097 151");
        displayLightningInfo(distance, energy);
    }
}

void loop()
{
    delay(100);
    digitalWrite(LED_PIN, HIGH); // Turn the LED off
    delay(100);

    if (AS3935IrqTriggered)
    {
        digitalWrite(LED_PIN, LOW); // Turn the LED on
        AS3935IrqTriggered = 0;

        int irqSource = as3935.interruptSource();
        if (irqSource & 0x01)
        {
            handleNoiseInterrupt();
        }
        else if (irqSource & 0x04)
        {
            handleDisturberInterrupt();
        }
        else if (irqSource & 0x08)
        {
            handleLightningInterrupt();
        }
        else
        {
            SerialUSB.println("Dummy interrupt");
        }
    }
}

void handleLightning() { AS3935IrqTriggered = 1; }

void scanI2CDevices()
{
    SerialUSB.println("Scanning...");

    int nDevices = 0;
    for (byte address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();

        if (error == 0)
        {
            SerialUSB.print("I2C device found at address 0x");
            if (address < 16)
                SerialUSB.print("0");
            SerialUSB.print(address, HEX);
            SerialUSB.println(" !");
            nDevices++;
        }
        else if (error == 4)
        {
            SerialUSB.print("Unknown error at address 0x");
            if (address < 16)
                SerialUSB.print("0");
            SerialUSB.println(address, HEX);
        }
    }

    if (nDevices == 0)
        SerialUSB.println("No I2C devices found\n");
    else
        SerialUSB.println("done\n");
}

void displayMessage(const String &message, int textSize)
{
    display.clearDisplay();
    display.setTextSize(textSize);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(message);
    display.display();
}

void displayLightningInfo(uint8_t dist, long energy)
{
    long persent = (energy * 100) / 2097151;
    display.clearDisplay();

    displayMessage("Lightning", 2);

    display.setTextSize(1);
    display.setCursor(0, 20);
    display.print("L: ");
    display.print(dist);
    display.print(" km; ");
    display.setCursor(0, 30);
    display.print("E: ");
    display.print(energy);
    display.print(" = ");
    display.print(persent);
    display.print(" %");
    display.display();
}

AS3935Registers getAS3935Registers()
{
    AS3935Registers regs;
    regs.noiseFloor = as3935.getNoiseFloor();
    regs.spikeRejection = as3935.getSpikeRejection();
    regs.watchdogThreshold = as3935.getWatchdogThreshold();
    regs.distance = as3935.lightningDistanceKm();
    regs.minNumberOfLightnings = as3935.getMinimumLightnings();
    regs.afe = as3935.getAfe();
    regs.trco = as3935.getTRCO();
    regs.srco = as3935.getSRCO();
    regs.capVal = as3935.getTuneCapVal();
    return regs;
}

void displayAS3935Registers(AS3935Registers regs)
{
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Noise: ");
    display.println(regs.noiseFloor);
    display.print("Spike: ");
    display.println(regs.spikeRejection);
    display.print("Watchdog: ");
    display.println(regs.watchdogThreshold);
    display.print("Distance: ");
    display.println(regs.distance);
    display.print("Min Light: ");
    display.println(regs.minNumberOfLightnings);
    display.print("AFE: ");
    display.println(regs.afe);
    display.print("TRCO: ");
    display.print(regs.trco);
    display.print(" SRCO: ");
    display.println(regs.srco);
    display.print("CapVal: ");
    display.println(regs.capVal);
    display.display();
}

void printAS3935Registers(AS3935Registers regs)
{
    char buffer[512]; // Adjust size as needed

    sprintf(buffer,
            "Noise floor is:         %d\n"
            "Spike rejection is:     %d\n"
            "Watchdog threshold is:  %d\n"
            "Lightning Distance Km:  %d\n"
            "Min Lightnings:         %d\n"
            "AFE:                    %d\n"
            "TRCO:                   %d\n"
            "SRCO:                   %d\n"
            "CapVal:                 %d\n",
            regs.noiseFloor, regs.spikeRejection, regs.watchdogThreshold, regs.distance,
            regs.minNumberOfLightnings, regs.afe, regs.trco, regs.srco, regs.capVal);

    SerialUSB.print(buffer);
}
