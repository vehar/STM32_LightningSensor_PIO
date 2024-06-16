#include "AS3935.h"

// Constructor for SPI
AS3935::AS3935(byte (*SPItransfer)(byte), int csPin, int irq)
    : SPITransferFunc(SPItransfer), _CSPin(csPin), _IRQPin(irq), _wire(nullptr) {
    digitalWrite(_CSPin, HIGH);
    pinMode(_CSPin, OUTPUT);
    pinMode(_IRQPin, INPUT);
}

// Constructor for I2C
AS3935::AS3935(TwoWire &wire, int irq, uint8_t address)
    : _wire(&wire), _address(address), _IRQPin(irq), SPITransferFunc(nullptr), _CSPin(-1) {
    pinMode(_IRQPin, INPUT);
}

// Initialization function
bool AS3935::begin() {
    if (_wire) {
        _wire->begin();
        // Additional I2C initialization if needed
        return true;
    } else if (SPITransferFunc) {
        SPI.begin();
        return true;
    }
    return false;
}

// SPI helper function
byte AS3935::_SPITransfer2(byte high, byte low) {
    digitalWrite(_CSPin, LOW);
    SPITransferFunc(high);
    byte regval = SPITransferFunc(low);
    digitalWrite(_CSPin, HIGH);
    return regval;
}

// I2C helper function
byte AS3935::_rawRegisterRead(byte reg) {
    if (_wire) {
        _wire->beginTransmission(_address);
        _wire->write(reg);
        _wire->endTransmission();
        _wire->requestFrom(_address, (uint8_t)1);
        return _wire->read();
    } else {
        return _SPITransfer2((reg & 0x3F) | 0x40, 0);
    }
}

byte AS3935::_ffsz(byte mask) {
    byte i = 0;
    if (mask)
        for (i = 1; ~mask & 1; i++)
            mask >>= 1;
    return i;
}

void AS3935::registerWrite(byte reg, byte mask, byte data) {
    byte regval = _rawRegisterRead(reg);
    regval &= ~(mask);
    if (mask)
        regval |= (data << (_ffsz(mask) - 1));
    else
        regval |= data;

    if (_wire) {
        _wire->beginTransmission(_address);
        _wire->write(reg & 0x3F);
        _wire->write(regval);
        _wire->endTransmission();
    } else {
        _SPITransfer2(reg & 0x3F, regval);
    }
}

byte AS3935::registerRead(byte reg, byte mask) {
    byte regval = _rawRegisterRead(reg);
    regval = regval & mask;
    if (mask)
        regval >>= (_ffsz(mask) - 1);
    return regval;
}

void AS3935::reset() {
    if (_wire) {
        _wire->beginTransmission(_address);
        _wire->write(0x3C);
        _wire->write(0x96);
        _wire->endTransmission();
    } else {
        _SPITransfer2(0x3C, 0x96);
    }
    delay(2);
}

bool AS3935::calibrate() {
    int target = 3125, currentcount = 0, bestdiff = INT_MAX, currdiff = 0;
    byte bestTune = 0, currTune = 0;
    unsigned long setUpTime;
    int currIrq, prevIrq;
    // set lco_fdiv divider to 0, which translates to 16
    // so we are looking for 31250Hz on irq pin
    // and since we are counting for 100ms that translates to number 3125
    // each capacitor changes second least significant digit
    // using this timing so this is probably the best way to go
    registerWrite(AS3935_LCO_FDIV, 0);
    registerWrite(AS3935_DISP_LCO, 1);
    // tuning is not linear, can't do any shortcuts here
    // going over all built-in cap values and finding the best
    for (currTune = 0; currTune <= 0x0F; currTune++) {
        registerWrite(AS3935_TUN_CAP, currTune);
        // let it settle
        delay(2);
        currentcount = 0;
        prevIrq = digitalRead(_IRQPin);
        setUpTime = millis() + 100;
        while ((long)(millis() - setUpTime) < 0) {
            currIrq = digitalRead(_IRQPin);
            if (currIrq > prevIrq) {
                currentcount++;
            }
            prevIrq = currIrq;
        }
        currdiff = target - currentcount;
        // don't look at me, abs() misbehaves
        if (currdiff < 0)
            currdiff = -currdiff;
        if (bestdiff > currdiff) {
            bestdiff = currdiff;
            bestTune = currTune;
        }
    }
    registerWrite(AS3935_TUN_CAP, bestTune);
    delay(2);
    registerWrite(AS3935_DISP_LCO, 0);
    // and now do RCO calibration
    powerUp();
    // if error is over 109, we are outside allowed tuning range of +/-3.5%
    return bestdiff > 109 ? false : true;
}

void AS3935::powerDown() {
    registerWrite(AS3935_PWD, 1);
}

void AS3935::powerUp() {
    registerWrite(AS3935_PWD, 0);
    if (_wire) {
        _wire->beginTransmission(_address);
        _wire->write(0x3D);
        _wire->write(0x96);
        _wire->endTransmission();
    } else {
        _SPITransfer2(0x3D, 0x96);
    }
    delay(3);
}

int AS3935::interruptSource() {
    return registerRead(AS3935_INT);
}

void AS3935::disableDisturbers() {
    registerWrite(AS3935_MASK_DIST, 1);
}

void AS3935::enableDisturbers() {
    registerWrite(AS3935_MASK_DIST, 0);
}

int AS3935::getMinimumLightnings() {
    return registerRead(AS3935_MIN_NUM_LIGH);
}

int AS3935::setMinimumLightnings(int minlightning) {
    registerWrite(AS3935_MIN_NUM_LIGH, minlightning);
    return getMinimumLightnings();
}

int AS3935::lightningDistanceKm() {
    return registerRead(AS3935_DISTANCE);
}

void AS3935::setIndoors() {
    registerWrite(AS3935_AFE_GB, AS3935_AFE_INDOOR);
}

void AS3935::setOutdoors() {
    registerWrite(AS3935_AFE_GB, AS3935_AFE_OUTDOOR);
}

int AS3935::getAfe() {
    return registerRead(AS3935_AFE_GB);
}

int AS3935::getNoiseFloor() {
    return registerRead(AS3935_NF_LEV);
}

int AS3935::setNoiseFloor(int noisefloor) {
    registerWrite(AS3935_NF_LEV, noisefloor);
    return getNoiseFloor();
}

int AS3935::getSpikeRejection() {
    return registerRead(AS3935_SREJ);
}

int AS3935::setSpikeRejection(int srej) {
    registerWrite(AS3935_SREJ, srej);
    return getSpikeRejection();
}

int AS3935::getWatchdogThreshold() {
    return registerRead(AS3935_WDTH);
}

int AS3935::setWatchdogThreshold(int wdth) {
    registerWrite(AS3935_WDTH, wdth);
    return getWatchdogThreshold();
}

void AS3935::clearStats() {
    registerWrite(AS3935_CL_STAT, 1);
    registerWrite(AS3935_CL_STAT, 0);
    registerWrite(AS3935_CL_STAT, 1);
}
