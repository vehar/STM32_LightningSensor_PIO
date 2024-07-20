#include <AS3935.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Thinary_AHT10.h>

#include <Wire.h>
#include <keypad.h>
#include <main.h>

void handleLightning();
volatile int AS3935IrqTriggered = 0;
bool lightningDetected = false;
bool activateMenuMode = false;
bool showAtmosphereData = false;

void appendToBuffer(char *buffer, int &idx, size_t bufferSize, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    idx += vsnprintf(buffer + idx, bufferSize - idx, format, args);
    va_end(args);
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

typedef void (*CallbackFunction)();

void executePeriodicAction(unsigned long currentTime, unsigned long &lastExecutionTime,
                           unsigned long interval, CallbackFunction callback)
{
    if (currentTime - lastExecutionTime >= interval)
    {
        callback();
        lastExecutionTime = currentTime;
    }
}

void handleError(const char *errorMessage)
{
    SerialUSB.println(errorMessage);
    displayMessage(errorMessage);
    // Additional error handling actions can be added here, such as logging or triggering an error

    while (true)
    {
        // Optionally blink an LED or do something to indicate an error state
        digitalWrite(LED_PIN, LOW); // Turn the LED on
        delay(500);
        digitalWrite(LED_PIN, HIGH); // Turn the LED off
        delay(500);
    }
}

void readAtmosphereData(Atmosphere &data)
{
    float temp = AHT10.GetTemperature() + bmp.readTemperature();
    data.temperature = temp / 2;
    data.dewPoint = AHT10.GetDewPoint();
    data.humidity = AHT10.GetHumidity();
    data.pressure = bmp.readPressure();
    data.altitude = bmp.readAltitude(1013.25); // Adjust to local sea level pressure
}

void printAtmosphereData(const Atmosphere &data)
{
    char buffer[256];

    char tempStr[10], dewPointStr[10], humidityStr[10], pressureStr[10], altitudeStr[10];
    dtostrf(data.temperature, 6, 2, tempStr);
    dtostrf(data.dewPoint, 6, 2, dewPointStr);
    dtostrf(data.humidity, 6, 2, humidityStr);
    dtostrf(data.pressure, 6, 2, pressureStr);
    dtostrf(data.altitude, 6, 2, altitudeStr);

    snprintf(buffer, sizeof(buffer),
             "Temp:%s C\nDew Point:%s C\nHumidity:%s %%\nPressure:%s Pa\nAltitude:%s m\n", tempStr,
             dewPointStr, humidityStr, pressureStr, altitudeStr);
    Serial.print(buffer);

    displayMessage(buffer);
}

///////////
// Function prototypes for menu actions
void actionMenuDumpDergs();
void actionManuAs3935Recalibrate();
void actionMenuExit();

// External variables to be tuned
int minimumLightnings = 0;
int noiseFloor = 3;
int watchDogTh = 1;
int spikeRegection = 0;
int dispAlgo = 1; // 0 - sum, 1 - shift
int updateSeconds = 1;

void actionMenuDumpDergs()
{
    SerialUSB.println("Action 1 executed");
    AS3935Registers regs = getAS3935Registers();
    printAS3935Registers(regs);
    delay(3000);
}

void actionManuAs3935Recalibrate()
{
    SerialUSB.println("Calibration..");
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

void actionMenuExit()
{
    SerialUSB.println("Exit executed");
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

    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR))
        handleError("SSD1306 allocation failed");

    if (!as3935.begin())
        handleError("AS3935 initialization failed!");

    if (AHT10.begin(eAHT10Address_Low))
        handleError("AHT10 initialization failed!");

    if (bmp.begin())
        handleError("BMP280 initialization failed!");

    displayMessage("Lightning sensor!", 1);
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    as3935.reset();
    AS3935Registers regs = getAS3935Registers();
    printAS3935Registers(regs);

    as3935.setOutdoors();
    as3935.enableDisturbers();
    actionManuAs3935Recalibrate();

    SerialUSB.println("AS3935 initialized and ready.");
    delay(1000);

    scanI2CDevices();
    delay(2000);

    menuManager.updateMenu(); // Initialize the menu manager with the root menu
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

void updateTimeCb()
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
}

void updateDisplayCb()
{
    static int index = 0;

    if (dispAlgo == 1) // Shift data to the right
    {
        for (int i = DATA_POINTS - 1; i > 0; i--)
            spikeArray[i] = spikeArray[i - 1];

        spikeArray[0] = { 0, 0, 0, 0, 0 }; // Clear the first element
    }
    else // Accumulate and display cyclic
        index = (index + 1) % DATA_POINTS;

    if (!lightningDetected && !activateMenuMode && !showAtmosphereData)
        updateDisplay();
}

void collectDataCb()
{
    static int index = 0;
    collectData(index);
}

void showAtmosphereDataCb()
{
    if (!showAtmosphereData)
        return;

    Atmosphere atmosphereData;
    readAtmosphereData(atmosphereData);
    printAtmosphereData(atmosphereData);
}

void loop()
{
    static int index = 0;
    static unsigned long lastDataCollectionTime = 0;
    static unsigned long lastDisplayUpdTime = 0;
    static unsigned long lastTimeUpdTime = 0;
    static unsigned long lastDetectedTime = 0;
    static unsigned long lastButtonReadTime = 0;
    static unsigned long lastShowAtmosphereDataTime = 0;

    unsigned long currentTime = millis();

    if (currentTime - lastButtonReadTime >= 200)
    {
        Button pressedButton = getPressedButton();
        if ((pressedButton == BUTTON_DOWN) && (activateMenuMode == false))
            activateMenuMode = true;

        if ((pressedButton == BUTTON_UP) && (activateMenuMode == false))
            showAtmosphereData = true;

        if ((pressedButton == BUTTON_CENTER) && (showAtmosphereData == true))
            showAtmosphereData = false;

        if (activateMenuMode)
        {
            menuManager.handleInput(pressedButton);
            lastButtonReadTime = currentTime;
        }
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

    // Collect data every 100 milliseconds
    executePeriodicAction(currentTime, lastDataCollectionTime, 100, collectDataCb);

    // Update time every 1000 milliseconds
    executePeriodicAction(currentTime, lastTimeUpdTime, 1000, updateTimeCb);

    // Update display every updateSeconds
    executePeriodicAction(currentTime, lastDisplayUpdTime, 1000 * updateSeconds, updateDisplayCb);

    // Show atmosphere data if needed
    executePeriodicAction(currentTime, lastShowAtmosphereDataTime, 2000, showAtmosphereDataCb);
}

void collectData(int index)
{
    if (AS3935IrqTriggered)
    {
        AS3935IrqTriggered = 0;

        int irqSource = as3935.interruptSource();
        if (irqSource & 0x01)
            handleNoiseInterrupt(index);
        else if (irqSource & 0x04)
            handleDisturberInterrupt(index);
        else if (irqSource & 0x08)
            handleLightningInterrupt(index);
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
        data[i] = (data[i] * maxValue) / maxDataValue;
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

    char buffer[128]; // Display uptime

    sprintf(buffer, "%02lu:%02lu:%02lu L %dkm E %d%%\n", currentTimeInfo.hours,
            currentTimeInfo.minutes, currentTimeInfo.seconds, lightningData.spike.lightningDistance,
            lightningData.spike.lightningEnergyPercent);

    displayMessage(buffer, 1, false);

    uint8_t row_max[4];
    uint8_t rows[4][DATA_POINTS];

    // Collect and normalize raw data
    for (int i = 0; i < DATA_POINTS; i++)
    {
        rows[0][i] = spikeArray[i].lightningDistance;
        rows[1][i] = spikeArray[i].lightningEnergyPercent;
        rows[2][i] = spikeArray[i].disturber;
        rows[3][i] = spikeArray[i].noise;
    }

    for (int i = 0; i < 4; i++)
    {
        normalizeData(rows[i], DATA_POINTS, HISTOGRAM_HEIGHT, row_max[i]);
        drawHistogramSection(rows[i], 21, i);
        printMaxValue(row_max[i], 26, i);
    }

    display.display();
}

void handleLightning() { AS3935IrqTriggered = 1; }

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

    if (deviceCount != 0)
    {
        appendToBuffer(buff, idx, sizeof(buff), "Found %d I2C devices:\n", deviceCount);
        for (int i = 0; i < deviceCount; i++)
            appendToBuffer(buff, idx, sizeof(buff), " - 0x%02X\n", foundDevices[i]);
    }
    else
        appendToBuffer(buff, idx, sizeof(buff), "No I2C devices found\n");

    SerialUSB.print(buff);
    displayMessage(buff);
}

void displayLightningInfo(uint8_t dist, uint8_t percent)
{
    char buffer[128]; // Adjust size as needed

    sprintf(buffer, " L: %d km\n E: %d %%", dist, percent);

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

    SerialUSB.print(buffer);
    displayMessage(buffer);
}
