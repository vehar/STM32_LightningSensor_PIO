#include <AS3935.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Menu.h>
#include <MenuItem.h>
#include <MenuManager.h>
#include <Parameter.h>
#include <Wire.h>
#include <keypad.h>

#define LED_PIN PC13

#define AS3935_IRQ_PIN PB5
#define AS3935_I2C_ADDR 0x01

#define DATA_POINTS 55
#define MAX_ENERGY 2097151
#define HISTOGRAM_HEIGHT 14 // Height of each histogram section

struct TimeInfo
{
    unsigned long hours;
    unsigned long minutes;
    unsigned long seconds;
};

struct MeasurementData
{
    uint8_t noise;
    uint8_t disturber;
    uint8_t lightningCount;
    uint8_t lightningDistance;
    uint8_t lightningEnergy;
};

MeasurementData data[DATA_POINTS];
TimeInfo currentTimeInfo = { 0, 0, 0 };

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

struct LightningInfo
{
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t distance;
    uint8_t energyPercent;
};

LightningInfo spikeInfo;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
AS3935 as3935(Wire, AS3935_IRQ_PIN, AS3935_I2C_ADDR);

void handleLightning();
volatile int AS3935IrqTriggered = 0;

bool lightningDetected = false;

void setup();
void loop();
void scanI2CDevices();
void displayMessage(const String &message, int textSize, bool immediateUpdate = true);
void displayLightningInfo(uint8_t dist, long energy, uint8_t percent);
AS3935Registers getAS3935Registers();
void printAS3935Registers(AS3935Registers regs);
void handleNoiseInterrupt(int index);
void handleDisturberInterrupt(int index);
void handleLightningInterrupt(int index);
void collectData(int index);
void updateDisplay();
void normalizeData(uint8_t *data, uint8_t length, uint8_t maxValue, uint8_t &maxDataValue);

///////////
// Function prototypes for menu actions
void action1();
void action2();

// External variables to be tuned
int externalVar1 = 0;
int externalVar2 = 50;

// Parameters referencing external variables
Parameter param1("Param1", externalVar1, -10, 10);
Parameter param2("Param2", externalVar2, 0, 100);

// Menu items and menus
MenuItem item1("Action 1", MENU_ITEM_ACTION, action1);
MenuItem item2(" Param1", MENU_ITEM_PARAMETER, nullptr, &param1);
MenuItem item3(" Param2", MENU_ITEM_PARAMETER, nullptr, &param2);
MenuItem item4("Action 2", MENU_ITEM_ACTION, action2);

MenuItem *mainMenuItems[] = { &item1, &item2, &item3, &item4 };
Menu mainMenu("Main Menu", mainMenuItems, 4);

MenuManager menuManager(display, &mainMenu);

void action1() { Serial.println("Action 1 executed"); }

bool activateMenuMode = false;
void action2()
{
    Serial.println("Action 2 executed");
    activateMenuMode = false;
}
/////////////

void setup()
{
    // For keypad pullUp
    pinMode(PA1, OUTPUT);
    digitalWrite(PA1, HIGH);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // Turn off the built-in LED

    SerialUSB.begin(115200);
    SerialUSB.setTimeout(10000);
    delay(3000); // Delay to allow USB connection to establish

    SerialUSB.println(F("System online."));

    Wire.setSCL(PB6);
    Wire.setSDA(PB7);
    Wire.begin();

    scanI2CDevices();

    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR))
    {
        SerialUSB.println(F("SSD1306 allocation failed"));
        while (1)
            ;
    }

    displayMessage("Lightning sensor!", 1);
    SerialUSB.println(F("Display initialized and message displayed."));

    if (!as3935.begin())
    {
        SerialUSB.println("AS3935 initialization failed!");
        while (1)
            ;
    }

    as3935.reset();
    AS3935Registers regs = getAS3935Registers();
    printAS3935Registers(regs);

    as3935.setOutdoors();
    as3935.enableDisturbers();
    as3935.setNoiseFloor(5);
    as3935.setMinimumLightnings(0);
    as3935.setSpikeRejection(0);
    as3935.setWatchdogThreshold(0);

    if (!as3935.calibrate())
    {
        SerialUSB.println("Tuning out of range, check your wiring, your sensor!");
    }
    else
    {
        SerialUSB.println("Calibration done!");
    }

    delay(1000);
    regs = getAS3935Registers();
    printAS3935Registers(regs);

    AS3935IrqTriggered = 0;
    attachInterrupt(digitalPinToInterrupt(AS3935_IRQ_PIN), handleLightning, RISING);
    SerialUSB.println("AS3935 initialized and ready.");

    // displayLightningInfo(42, 2000000, 99);
    delay(1000);

    // Initialize the menu manager with the root menu
    menuManager.updateDisplay();
}

void handleNoiseInterrupt(int index)
{
    data[index].noise++;
    SerialUSB.println("Noise level too high, try adjusting noise floor");
}

void handleDisturberInterrupt(int index)
{
    data[index].disturber++;
    SerialUSB.println("Disturber detected");
}

void handleLightningInterrupt(int index)
{
    lightningDetected = true;
    int distance = as3935.lightningDistanceKm();
    long energy = as3935.lightningEnergy();

    spikeInfo.distance = distance;
    spikeInfo.energyPercent = (energy * 100) / MAX_ENERGY;
    spikeInfo.sec = currentTimeInfo.seconds;
    spikeInfo.min = currentTimeInfo.minutes;
    spikeInfo.hour = currentTimeInfo.hours;

    data[index].lightningCount++;
    data[index].lightningDistance = distance;
    data[index].lightningEnergy = spikeInfo.energyPercent;

    displayLightningInfo(distance, energy, spikeInfo.energyPercent);

    if (distance == 1)
        SerialUSB.println("Storm overhead, watch out!");
    else if (distance == 63)
        SerialUSB.println("Out of range lightning detected.");
}

void loop()
{
    Button pressedButton = getPressedButton();
    if ((pressedButton == BUTTON_DOWN) && (activateMenuMode == false))
        activateMenuMode = true;

    if (activateMenuMode)
        menuManager.handleInput(pressedButton);

    char buffer[128]; // Adjust size as needed
    sprintf(buffer, "1 = : %d, 2 = : %d  \r\n", externalVar1, externalVar2);
    SerialUSB.print(buffer);

    delay(200); // Debounce delay

    /*
        static int index = 0;
        static unsigned long lastDataCollectionTime = 0;
        static unsigned long lastDisplayUpdateTime = 0;
        static unsigned long lastDetectedTime = 0;

        unsigned long currentTime = millis();

        // Collect data every 100 milliseconds
        if (currentTime - lastDataCollectionTime >= 100)
        {
            collectData(index);
            lastDataCollectionTime = currentTime;
        }

        // Update display every 1000 milliseconds
        if (currentTime - lastDisplayUpdateTime >= 1000)
        {
            currentTimeInfo.seconds++;
            if (currentTimeInfo.seconds >= 60)
            {
                currentTimeInfo.seconds = 0;
                currentTimeInfo.minutes++;
                if (currentTimeInfo.minutes >= 60)
                {
                    currentTimeInfo.minutes = 0;
                    currentTimeInfo.hours++;
                }
            }

            // Accumulate and display cyclic
            // index = (index + 1) % DATA_POINTS;

            // OR
            // Shift data to the right
            for (int i = DATA_POINTS - 1; i > 0; i--)
                data[i] = data[i - 1];

            data[0] = { 0, 0, 0, 0, 0 }; // Clear the first element

            if (lightningDetected == false)
                updateDisplay();

            lastDisplayUpdateTime = currentTime;
        }

        if (lightningDetected)
        {
            if (lastDetectedTime == 0)
                lastDetectedTime = currentTime;

            if (currentTime - lastDetectedTime >= 5000)
            {
                lightningDetected = false;
                lastDetectedTime = 0;
            }
        }*/
}

void collectData(int index)
{
    if (AS3935IrqTriggered)
    {
        AS3935IrqTriggered = 0;

        int irqSource = as3935.interruptSource();
        if (irqSource & 0x01)
        {
            handleNoiseInterrupt(index);
        }
        else if (irqSource & 0x04)
        {
            handleDisturberInterrupt(index);
        }
        else if (irqSource & 0x08)
        {
            handleLightningInterrupt(index);
        }
    }
}

void normalizeData(uint8_t *data, uint8_t length, uint8_t maxValue, uint8_t &maxDataValue)
{
    maxDataValue = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        if (data[i] > maxDataValue)
            maxDataValue = data[i];
    }
    for (uint8_t i = 0; i < length; i++)
    {
        data[i] = (data[i] * maxValue) / maxDataValue;
    }
}

void drawHistogramSection(uint8_t *data, int yStart, int line)
{
    int yOffset = HISTOGRAM_HEIGHT * line;
    for (int i = 0; i < DATA_POINTS; i++)
        display.drawLine(i * 2, yStart + yOffset, i * 2, yStart + yOffset - data[i], SSD1306_WHITE);
}

void printMaxValue(int maxValue, int yStart, int line)
{
    int yPosition = yStart + HISTOGRAM_HEIGHT * line - HISTOGRAM_HEIGHT;
    display.setCursor(SCREEN_WIDTH - 15, yPosition);
    display.printf("%d", maxValue);
}

void updateDisplay()
{
    display.clearDisplay();

    // Draw uptime
    char buffer[128]; // Display uptime

    sprintf(buffer, "%02lu:%02lu:%02lu L %dkm E %d%%\n", currentTimeInfo.hours,
            currentTimeInfo.minutes, currentTimeInfo.seconds, spikeInfo.distance,
            spikeInfo.energyPercent);

    displayMessage(buffer, 1, false);

    uint8_t maxNoise, maxDisturber, maxDistance, maxEnergy;
    uint8_t noiseCounts[DATA_POINTS];
    uint8_t disturberCounts[DATA_POINTS];
    uint8_t lightningDistances[DATA_POINTS];
    uint8_t lightningEnergies[DATA_POINTS];

    // Collect raw data
    for (int i = 0; i < DATA_POINTS; i++)
    {
        noiseCounts[i] = data[i].noise;
        disturberCounts[i] = data[i].disturber;
        lightningDistances[i] = data[i].lightningDistance;
        lightningEnergies[i] = data[i].lightningEnergy;
    }

    // Normalize data
    normalizeData(lightningDistances, DATA_POINTS, HISTOGRAM_HEIGHT, maxDistance);
    normalizeData(lightningEnergies, DATA_POINTS, HISTOGRAM_HEIGHT, maxEnergy);
    normalizeData(disturberCounts, DATA_POINTS, HISTOGRAM_HEIGHT, maxDisturber);
    normalizeData(noiseCounts, DATA_POINTS, HISTOGRAM_HEIGHT, maxNoise);

    // Draw histograms
    int y = 21;
    drawHistogramSection(lightningDistances, y, 0);
    drawHistogramSection(lightningEnergies, y, 1);
    drawHistogramSection(disturberCounts, y, 2);
    drawHistogramSection(noiseCounts, y, 3);

    y += 5;
    // Display max values
    printMaxValue(maxDistance, y, 0);
    printMaxValue(maxEnergy, y, 1);
    printMaxValue(maxDisturber, y, 2);
    printMaxValue(maxNoise, y, 3);

    display.display();
}

void handleLightning() { AS3935IrqTriggered = 1; }

void appendToBuffer(char *buffer, int &idx, size_t bufferSize, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    idx += vsnprintf(buffer + idx, bufferSize - idx, format, args);
    va_end(args);
}

void scanI2CDevices()
{
    char buff[1024];
    int idx = 0;
    byte foundDevices[127];
    int deviceCount = 0;

    appendToBuffer(buff, idx, sizeof(buff), "Scanning...\n");

    for (byte address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();

        if (error == 0)
            foundDevices[deviceCount++] = address;
        else if (error == 4)
            appendToBuffer(buff, idx, sizeof(buff), "Unknown error at addr 0x%02X\n", address);
    }

    if (deviceCount == 0)
    {
        appendToBuffer(buff, idx, sizeof(buff), "No I2C devices found\n");
    }
    else
    {
        appendToBuffer(buff, idx, sizeof(buff), "Found %d I2C device(s):\n", deviceCount);
        for (int i = 0; i < deviceCount; i++)
            appendToBuffer(buff, idx, sizeof(buff), " - 0x%02X\n", foundDevices[i]);
    }

    SerialUSB.print(buff);
}

void displayMessage(const String &message, int textSize, bool immediateUpdate)
{
    display.clearDisplay();
    display.setTextSize(textSize);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(message);

    if (immediateUpdate)
        display.display();
}

void displayLightningInfo(uint8_t dist, long energy, uint8_t percent)
{
    char buffer[128]; // Adjust size as needed

    sprintf(buffer, " L: %d km\n E: %d %%", dist, percent);

    display.clearDisplay();
    displayMessage("Lightning!", 2);

    // display.setTextSize(1);
    display.setCursor(0, 30);
    display.print(buffer);
    display.display();

    SerialUSB.print(buffer);
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

void printAS3935Registers(AS3935Registers regs)
{
    char buffer[512]; // Adjust size as needed

    sprintf(buffer,
            "Noise: %d Spike: %d\n"
            "Watchdog: %d\n"
            "Distance: %d\n"
            "Min Light: %d\n"
            "TRCO: %d SRCO: %d\n"
            "AFE: %d CapVal: %d\n",
            regs.noiseFloor, regs.spikeRejection, regs.watchdogThreshold, regs.distance,
            regs.minNumberOfLightnings, regs.trco, regs.srco, regs.afe, regs.capVal);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(buffer);
    display.display();

    SerialUSB.print(buffer);
}
