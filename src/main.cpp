/*
SCL -> PB6
SDA -> PB7
*/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <AS3935.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDR     0x3C
#define LED_PIN       PC13

#define AS3935_IRQ_PIN PA1
#define AS3935_I2C_ADDR 0x01

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
AS3935 as3935(Wire, AS3935_IRQ_PIN, AS3935_I2C_ADDR);

void AS3935Irq();
volatile int AS3935IrqTriggered = 0;

void scanI2CDevices();

void displayLightningInfo(uint8_t dist) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 10);
    display.print("Lightning");
    display.setCursor(0, 30);
    display.print(dist);
    display.print(" km");
    display.display();
}

void printAS3935Registers()
{
  int noiseFloor = as3935.getNoiseFloor();
  int spikeRejection = as3935.getSpikeRejection();
  int watchdogThreshold = as3935.getWatchdogThreshold();
  int distance = as3935.lightningDistanceKm();
  int minNumberOfLightnings = as3935.getMinimumLightnings();
  int afe = as3935.getAfe();

  SerialUSB.print("Noise floor is: ");
  SerialUSB.println(noiseFloor,DEC);
  SerialUSB.print("Spike rejection is: ");
  SerialUSB.println(spikeRejection,DEC);
  SerialUSB.print("Watchdog threshold is: ");
  SerialUSB.println(watchdogThreshold,DEC);  
  SerialUSB.print("lightningDistanceKm: ");
  SerialUSB.println(distance,DEC);  

  SerialUSB.print("minNumberOfLightnings: ");
  SerialUSB.println(minNumberOfLightnings,DEC);  
  SerialUSB.print("afe: ");
  SerialUSB.println(afe,DEC);  
}

void handleLightning() {
    AS3935IrqTriggered = 1;
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // Turn off the built-in LED

    // Initialize Serial Monitor
    SerialUSB.begin(115200);
    SerialUSB.setTimeout(10000);
    delay(3000); // Delay to allow USB connection to establish

    SerialUSB.println(F("System online."));

    // Initialize I2C on PB6 (SCL) and PB7 (SDA)
    Wire.setSCL(PB6);
    Wire.setSDA(PB7);
    Wire.begin();

    scanI2CDevices();

    // Initialize Display
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        SerialUSB.println(F("SSD1306 allocation failed"));
        for (;;);
    }
    
    display.display();
    delay(2000); // Pause for 2 seconds

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Hello, STM32!"));
    display.display();

    SerialUSB.println(F("Display initialized and message displayed."));

      // Initialize AS3935
    if (!as3935.begin()) {
        SerialUSB.println("AS3935 initialization failed!");
        while (1);
    }

    if (!as3935.calibrate()) {
        SerialUSB.println("Tuning out of range, check your wiring, your sensor and make sure physics laws have not changed!");
    }

    as3935.setOutdoors();
    as3935.disableDisturbers();
    as3935.setNoiseFloor(0);

    printAS3935Registers();
    AS3935IrqTriggered = 0;

    attachInterrupt(digitalPinToInterrupt(AS3935_IRQ_PIN), handleLightning, RISING);

    SerialUSB.println("AS3935 initialized and ready.");
}

void loop() {
    digitalWrite(LED_PIN, LOW); // Turn the LED on
    delay(500);
    digitalWrite(LED_PIN, HIGH); // Turn the LED off
    delay(500);

   if (AS3935IrqTriggered) {
        AS3935IrqTriggered = 0;

        int irqSource = as3935.interruptSource();
        if (irqSource & 0x01) {
            SerialUSB.println("Noise level too high, try adjusting noise floor");
        }
        if (irqSource & 0x04) {
            SerialUSB.println("Disturber detected");
        }
        if (irqSource & 0x08) {
            int distance = as3935.lightningDistanceKm();
            if (distance == 1) {
                SerialUSB.println("Storm overhead, watch out!");
            } else if (distance == 63) {
                SerialUSB.println("Out of range lightning detected.");
            } else {
                SerialUSB.print("Lightning detected ");
                SerialUSB.print(distance);
                SerialUSB.println(" kilometers away.");
                displayLightningInfo(distance);
            }
        }
    }

    //scanI2CDevices();
}

void scanI2CDevices() {
    byte error, address;
    int nDevices;

    SerialUSB.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            SerialUSB.print("I2C device found at address 0x");
            if (address < 16)
                SerialUSB.print("0");
            SerialUSB.print(address, HEX);
            SerialUSB.println(" !");

            nDevices++;
        } else if (error == 4) {
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
