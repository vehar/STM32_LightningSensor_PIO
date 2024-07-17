#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>

#include <Menu.h>
#include <MenuItem.h>
#include <MenuManager.h>
#include <Parameter.h>

#define LED_PIN PC13

#define AS3935_IRQ_PIN PB5
#define AS3935_I2C_ADDR 0x01

#define DATA_POINTS 55
#define MAX_ENERGY 2097151
#define HISTOGRAM_HEIGHT 14 // Height of each histogram section

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

#define LIGHTNING_MAX_DETECTION_DISTANCE 63

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

struct TimeInfo
{
    unsigned long hours;
    unsigned long minutes;
    unsigned long seconds;
};

TimeInfo currentTimeInfo = { 0, 0, 0 };

struct SpikeInfo
{
    uint8_t noise;
    uint8_t disturber;
    uint8_t lightningCount;
    uint8_t lightningDistance;
    uint8_t lightningEnergyPercent;
};

SpikeInfo spikeArray[DATA_POINTS];

struct LightningData
{
    TimeInfo time;
    SpikeInfo spike;
};

LightningData lightningData;

struct Atmosphere
{
    float temperature;
    float dewPoint;
    float humidity;
    float pressure;
    float altitude;
};

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
AS3935 as3935(Wire, AS3935_IRQ_PIN, AS3935_I2C_ADDR);
AHT10Class AHT10;
Adafruit_BMP280 bmp;

void handleLightning();

void setup();
void loop();
void scanI2CDevices();
void displayMessage(const String &message, int textSize = 1, bool immediateUpdate = true);
void displayLightningInfo(uint8_t dist, uint8_t percent);
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
void actionMenuDumpDergs();
void actionManuAs3935Recalibrate();
void actionMenuExit();

// External variables to be tuned
extern int minimumLightnings;
extern int noiseFloor;
extern int watchDogTh;
extern int spikeRegection;
extern int dispAlgo;
extern int updateSeconds;

// Parameters referencing external variables
Parameter param1("minimumLightnings", minimumLightnings, 0, 3);
Parameter param2("noiseFloor", noiseFloor, 0, 7);
Parameter param3("watchDogTh", watchDogTh, 0, 15);
Parameter param4("spikeRegection", spikeRegection, 0, 15);
Parameter param5("0-sum\n1-shift", dispAlgo, 0, 1);
Parameter param6("every(s)", updateSeconds, 1, 3600);

// Menu items and menus
MenuItem item1("View settings", MENU_ITEM_ACTION, actionMenuDumpDergs);

MenuItem item2(" minimumLightnings", MENU_ITEM_PARAMETER, nullptr, &param1);
MenuItem item3(" noiseFloor", MENU_ITEM_PARAMETER, nullptr, &param2);
MenuItem item4(" watchDogTh", MENU_ITEM_PARAMETER, nullptr, &param3);
MenuItem item5(" spikeRegection", MENU_ITEM_PARAMETER, nullptr, &param4);
MenuItem item6(" dispAlgo", MENU_ITEM_PARAMETER, nullptr, &param5);
MenuItem item7(" update scale", MENU_ITEM_PARAMETER, nullptr, &param6);

MenuItem item8("Recalibrate", MENU_ITEM_ACTION, actionManuAs3935Recalibrate);
MenuItem item9("Exit", MENU_ITEM_ACTION, actionMenuExit);

MenuItem *mainMenuItems[] = {
    &item1, &item2, &item3, &item4, &item5, &item6, &item7, &item8, &item9
};

const int numberOfMenuItems = sizeof(mainMenuItems) / sizeof(mainMenuItems[0]);
Menu mainMenu("Main Menu", mainMenuItems, numberOfMenuItems);

MenuManager menuManager(display, &mainMenu);

#endif // MAIN_H