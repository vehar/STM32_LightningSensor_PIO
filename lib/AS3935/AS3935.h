#ifndef AS3935_H
#define AS3935_H

#include <SPI.h>
#include <Wire.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <limits.h>

// register access macros - register address, bitmask
#define AS3935_AFE_GB 0x00, 0x3E
#define AS3935_PWD 0x00, 0x01
#define AS3935_NF_LEV 0x01, 0x70
#define AS3935_WDTH 0x01, 0x0F
#define AS3935_CL_STAT 0x02, 0x40
#define AS3935_MIN_NUM_LIGH 0x02, 0x30
#define AS3935_SREJ 0x02, 0x0F
#define AS3935_LCO_FDIV 0x03, 0xC0
#define AS3935_MASK_DIST 0x03, 0x20
#define AS3935_INT 0x03, 0x0F
#define AS3935_DISTANCE 0x07, 0x3F
#define AS3935_DISP_LCO 0x08, 0x80
#define AS3935_DISP_SRCO 0x08, 0x40
#define AS3935_DISP_TRCO 0x08, 0x20
#define AS3935_TUN_CAP 0x08, 0x0F

// other constants
#define AS3935_AFE_INDOOR 0x12
#define AS3935_AFE_OUTDOOR 0x0E

class AS3935
{
public:
    // Constructor for SPI
    AS3935(byte (*SPItransfer)(byte), int csPin, int irq);

    // Constructor for I2C
    AS3935(TwoWire &wire, int irq, uint8_t address = 0x03);

    // Initialization function
    bool begin();

    // Calibration function
    bool calibrate();

    // Functions to read/write registers
    void registerWrite(byte reg, byte mask, byte data);
    byte registerRead(byte reg, byte mask);

    // Other functions
    void reset();
    void powerDown();
    void powerUp();
    int interruptSource();
    void disableDisturbers();
    void enableDisturbers();
    int getMinimumLightnings();
    int setMinimumLightnings(int minlightning);
    int lightningDistanceKm();
    void setIndoors();
    void setOutdoors();
    int getAfe();
    int getNoiseFloor();
    int setNoiseFloor(int noisefloor);
    int getSpikeRejection();
    int setSpikeRejection(int srej);
    int getWatchdogThreshold();
    int setWatchdogThreshold(int wdth);
    void clearStats();

private:
    // SPI-related variables
    byte (*SPITransferFunc)(byte);
    int _CSPin;

    // I2C-related variables
    TwoWire *_wire;
    uint8_t _address;

    // Shared variables
    int _IRQPin;

    // Helper functions
    byte _SPITransfer2(byte high, byte low);
    byte _rawRegisterRead(byte reg);
    byte _ffsz(byte mask);
};

#endif
