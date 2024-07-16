#include <AS3935.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>

#include <Wire.h>
#include <keypad.h>
#include <main.h>
#include <STM32RTC.h>

STM32RTC& rtc = STM32RTC::getInstance();

volatile int AS3935IrqTriggered = 0;
bool lightningDetected = false;
bool activateMenuMode = false;

int minimumLightnings = 0;
int noiseFloor = 3;
int watchDogTh = 1;
int spikeRegection = 0;
int dispAlgo = 1; // 0 - sum, 1 - shift
int updateSeconds = 1;

void action1()
{
    Serial.println("Action 1 executed");
    AS3935Registers regs = getAS3935Registers();
    printAS3935Registers(regs);
    delay(3000);
}

void as3935_InitRecalibrate()
{
    Serial.println("Calibration..");
    detachInterrupt(digitalPinToInterrupt(AS3935_IRQ_PIN));

    as3935.setMinimumLightnings(minimumLightnings);
    as3935.setNoiseFloor(noiseFloor);
    as3935.setSpikeRejection(spikeRegection);
    as3935.setWatchdogThreshold(watchDogTh);

    if (!as3935.calibrate())
        SerialUSB.println("Tuning out of range, check your wiring, your sensor!");
    else
        SerialUSB.println("Calibration done!");

    delay(1000);
    printAS3935Registers(getAS3935Registers());

    AS3935IrqTriggered = 0;
    attachInterrupt(digitalPinToInterrupt(AS3935_IRQ_PIN), handleLightning, RISING);
}

void action3()
{
    Serial.println("Exit executed");
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
    as3935_InitRecalibrate();

    SerialUSB.println("AS3935 initialized and ready.");

    // displayLightningInfo(42, 99);
    delay(1000);

    menuManager.updateDisplay(); // Initialize the menu manager with the root menu
}

void handleNoiseInterrupt(int index)
{
    spikeArray[index].noise++;
    SerialUSB.println("Noise level too high, try adjusting noise floor");
}

void handleDisturberInterrupt(int index)
{
    spikeArray[index].disturber++;
    SerialUSB.println("Disturber detected");
}

void handleLightningInterrupt(int index)
{
    lightningDetected = true;
    int distance = as3935.lightningDistanceKm();
    long energy = as3935.lightningEnergy();

    lightningData.spike.lightningDistance = distance;
    lightningData.spike.lightningEnergyPercent = (energy * 100) / MAX_ENERGY;
    lightningData.time = currentTimeInfo;

    spikeArray[index].lightningCount++;
    spikeArray[index].lightningDistance = distance;
    spikeArray[index].lightningEnergyPercent = lightningData.spike.lightningEnergyPercent;

    //   displayLightningInfo(lightningData.distance, lightningData.energyPercent);

    if (distance == 1)
        SerialUSB.println("Storm overhead, watch out!");
    else if (distance == LIGHTNING_MAX_DETECTION_DISTANCE)
        SerialUSB.println("Out of range lightning detected.");
}

void loop()
{
    static int index = 0;
    static unsigned long lastDataCollectionTime = 0;
    static unsigned long lastDisplayUpdateTime = 0;
    static unsigned long lastTimeUpdateTime = 0;
    static unsigned long lastDetectedTime = 0;
    static unsigned long lastButtonReadTime = 0;

    unsigned long currentTime = millis();

    if (currentTime - lastButtonReadTime >= 200)
    {
        Button pressedButton = getPressedButton();
        if ((pressedButton == BUTTON_DOWN) && (activateMenuMode == false))
            activateMenuMode = true;

        if (activateMenuMode)
        {
            menuManager.handleInput(pressedButton);
            lastButtonReadTime = currentTime;
            // char buffer[128]; // Adjust size as needed
            // sprintf(buffer, "1 = : %d, 2 = : %d  \r\n", externalVar1, externalVar2);
            // SerialUSB.print(buffer);
        }
    }

    // Collect data every 100 milliseconds
    if (currentTime - lastDataCollectionTime >= 100)
    {
        collectData(index);
        lastDataCollectionTime = currentTime;
    }

    if (currentTime - lastTimeUpdateTime >= 1000)
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
        lastTimeUpdateTime = currentTime;
    }

    // Update display every 1000 milliseconds
    if (currentTime - lastDisplayUpdateTime >= (1000 * updateSeconds))
    {
        if (dispAlgo == 0) // Accumulate and display cyclic
        {
            index = (index + 1) % DATA_POINTS;
        }
        else if (dispAlgo == 1) // Shift data to the right
        {
            index = 0;
            for (int i = DATA_POINTS - 1; i > 0; i--)
                spikeArray[i] = spikeArray[i - 1];

            spikeArray[0] = { 0, 0, 0, 0, 0 }; // Clear the first element
        }

        if ((!lightningDetected) && (!activateMenuMode))
            updateDisplay();

        lastDisplayUpdateTime = currentTime;
    }

    if (lightningDetected)
    {
        if (lastDetectedTime == 0)
        {
            displayLightningInfo(lightningData.spike.lightningDistance,
                                 lightningData.spike.lightningEnergyPercent);
            lastDetectedTime = currentTime;
        }

        if (currentTime - lastDetectedTime >= 5000)
        {
            lightningDetected = false;
            lastDetectedTime = 0;
        }
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
            currentTimeInfo.minutes, currentTimeInfo.seconds, lightningData.spike.lightningDistance,
            lightningData.spike.lightningEnergyPercent);

    displayMessage(buffer, 1, false);

    uint8_t maxNoise, maxDisturber, maxDistance, maxEnergy;

    uint8_t noiseCounts[DATA_POINTS];
    uint8_t disturberCounts[DATA_POINTS];
    uint8_t lightningDistances[DATA_POINTS];
    uint8_t lightningEnergies[DATA_POINTS];

    // Collect raw data
    for (int i = 0; i < DATA_POINTS; i++)
    {
        noiseCounts[i] = spikeArray[i].noise;
        disturberCounts[i] = spikeArray[i].disturber;
        lightningEnergies[i] = spikeArray[i].lightningEnergyPercent;
        lightningDistances[i] = spikeArray[i].lightningDistance;
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

void displayLightningInfo(uint8_t dist, uint8_t percent)
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
            "Min Light: %d\n"
            "Distance: %d\n"
            "TRCO: %d SRCO: %d\n"
            "AFE: %d CapVal: %d\n",
            regs.noiseFloor, regs.spikeRejection, regs.watchdogThreshold,
            regs.minNumberOfLightnings, regs.distance, regs.trco, regs.srco, regs.afe, regs.capVal);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(buffer);
    display.display();

    SerialUSB.print(buffer);
}
