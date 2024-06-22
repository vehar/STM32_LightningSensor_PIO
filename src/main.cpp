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

#define DATA_POINTS 60
#define ENERGY_SCALE_FACTOR 1000
#define MAX_ENERGY 2097151
#define HISTOGRAM_HEIGHT 12 // Height of each histogram section

uint8_t noiseCounts[DATA_POINTS] = { 0 };
uint8_t disturberCounts[DATA_POINTS] = { 0 };
uint8_t lightningCounts[DATA_POINTS] = { 0 };
uint8_t lightningDistances[DATA_POINTS] = { 0 };
uint8_t lightningEnergies[DATA_POINTS] = { 0 };

unsigned long startTime = 0;
unsigned long lastUpdateTime = 0;
unsigned long uptimeSeconds = 0;

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
    uint8_t energyPersent;
};

LightningInfo spikeInfo;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
AS3935 as3935(Wire, AS3935_IRQ_PIN, AS3935_I2C_ADDR);

void handleLightning();
volatile int AS3935IrqTriggered = 0;

void setup();
void loop();
void scanI2CDevices();
void displayMessage(const String &message, int textSize, bool emidiateUpdate = true);
void displayLightningInfo(uint8_t dist, long energy, uint8_t percent);
AS3935Registers getAS3935Registers();
void printAS3935Registers(AS3935Registers regs);
void handleNoiseInterrupt(int index);
void handleDisturberInterrupt(int index);
void handleLightningInterrupt(int index);
void collectData(int index);
void updateDisplay();
void normalizeData(uint8_t *data, uint8_t length, uint8_t maxValue);

void setup()
{
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
    as3935.setNoiseFloor(1);
    as3935.setMinimumLightnings(0);

    if (!as3935.calibrate())
    {
        SerialUSB.println("Tuning out of range, check your wiring, your sensor!");
    }
    else
    {
        SerialUSB.println("Calibration done!");
    }

    delay(2000);
    regs = getAS3935Registers();
    printAS3935Registers(regs);

    AS3935IrqTriggered = 0;
    attachInterrupt(digitalPinToInterrupt(AS3935_IRQ_PIN), handleLightning, RISING);
    SerialUSB.println("AS3935 initialized and ready.");

    // displayLightningInfo(42, 2000000, 99);
    delay(3000);

    startTime = millis();
    lastUpdateTime = millis();
}

void handleNoiseInterrupt(int index)
{
    noiseCounts[index]++;
    SerialUSB.println("Noise level too high, try adjusting noise floor");
}

void handleDisturberInterrupt(int index)
{
    disturberCounts[index]++;
    SerialUSB.println("Disturber detected");
}

void handleLightningInterrupt(int index)
{
    int distance = as3935.lightningDistanceKm();
    long energy = as3935.lightningEnergy();

    spikeInfo.distance = distance;
    spikeInfo.energyPersent = (energy * 100) / MAX_ENERGY;
    spikeInfo.sec = uptimeSeconds % 60;
    spikeInfo.min = (uptimeSeconds / 60) % 60;
    spikeInfo.hour = (uptimeSeconds / 3600);

    lightningCounts[index]++;
    lightningDistances[index] = distance;
    lightningEnergies[index] = spikeInfo.energyPersent;

    displayLightningInfo(distance, energy, spikeInfo.energyPersent);

    if (distance == 1)
        SerialUSB.println("Storm overhead, watch out!");
    else if (distance == 63)
        SerialUSB.println("Out of range lightning detected.");
}

void loop()
{
    static int index = 0;
    static unsigned long lastDataCollectionTime = 0;
    static unsigned long lastDisplayUpdateTime = 0;

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
        uptimeSeconds++;
        index = (index + 1) % DATA_POINTS;
        updateDisplay();
        lastDisplayUpdateTime = currentTime;
    }
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

void normalizeData(uint8_t *data, uint8_t length, uint8_t maxValue)
{
    uint8_t maxData = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        if (data[i] > maxData)
            maxData = data[i];
    }
    for (uint8_t i = 0; i < length; i++)
    {
        data[i] = (data[i] * maxValue) / maxData;
    }
}

void updateDisplay()
{
    display.clearDisplay();

    // Draw uptime
    unsigned long seconds = uptimeSeconds % 60;
    unsigned long minutes = (uptimeSeconds / 60) % 60;
    unsigned long hours = (uptimeSeconds / 3600);

    char buffer[128]; // Display uptime

    sprintf(buffer, "T:%02lu:%02lu:%02lu L%dkm E%d%%\n", hours, minutes, seconds,
            spikeInfo.distance, spikeInfo.energyPersent);

    displayMessage(buffer, 1, false);

    // Normalize data
    normalizeData(noiseCounts, DATA_POINTS, HISTOGRAM_HEIGHT);
    normalizeData(disturberCounts, DATA_POINTS, HISTOGRAM_HEIGHT);
    normalizeData(lightningDistances, DATA_POINTS, HISTOGRAM_HEIGHT);
    normalizeData(lightningEnergies, DATA_POINTS, HISTOGRAM_HEIGHT);

    // Draw histograms
    int y = 20;
    for (int i = 0; i < DATA_POINTS; i++)
    {
        // Noise
        display.drawLine(i * 2, y, i * 2, y - noiseCounts[i], SSD1306_WHITE);
        // Disturber
        display.drawLine(i * 2, y + HISTOGRAM_HEIGHT, i * 2,
                         y + HISTOGRAM_HEIGHT - disturberCounts[i], SSD1306_WHITE);
        // Lightning (distance and energy)
        display.drawLine(i * 2, y + HISTOGRAM_HEIGHT * 2, i * 2,
                         y + HISTOGRAM_HEIGHT * 2 - lightningDistances[i], SSD1306_WHITE);
        display.drawLine(i * 2, y + HISTOGRAM_HEIGHT * 3, i * 2,
                         y + HISTOGRAM_HEIGHT * 3 - lightningEnergies[i], SSD1306_WHITE);
    }

    display.display();
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
    {
        SerialUSB.println("No I2C devices found\n");
    }
    else
    {
        SerialUSB.println("done\n");
    }
}

void displayMessage(const String &message, int textSize, bool emidiateUpdate)
{
    display.clearDisplay();
    display.setTextSize(textSize);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(message);

    if (emidiateUpdate)
        display.display();
}

void displayLightningInfo(uint8_t dist, long energy, uint8_t percent)
{
    char buffer[128]; // Adjust size as needed

    sprintf(buffer, "L: %d km; E: %ld = %d %%", dist, energy / ENERGY_SCALE_FACTOR, percent);

    display.clearDisplay();
    displayMessage("Lightning !", 2);

    display.setTextSize(1);
    display.setCursor(0, 20);
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
