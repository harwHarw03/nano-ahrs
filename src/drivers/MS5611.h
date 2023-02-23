#ifndef MS5611_H
#define MS5611_H
#include <Arduino.h>

#include <simple_i2c.h>
#include <delay_millis.h>

//  MS5611 Registers
#define MS5611_ADDRESS                (0x77)

#define MS5611_CMD_ADC_READ           (0x00)
#define MS5611_CMD_RESET              (0x1E)
#define MS5611_CMD_CONV_D1            (0x40)
#define MS5611_CMD_CONV_D2            (0x50)
#define MS5611_CMD_READ_PROM          (0xA2)

typedef enum
{
    MS5611_ULTRA_HIGH_RES   = 0x08,
    MS5611_HIGH_RES         = 0x06,
    MS5611_STANDARD         = 0x04,
    MS5611_LOW_POWER        = 0x02,
    MS5611_ULTRA_LOW_POWER  = 0x00
} ms5611_osr_t;


bool MS5611_Init(ms5611_osr_t osr = MS5611_HIGH_RES);
uint32_t readRawTemperature(void);
uint32_t readRawPressure(void);
void setOversampling(ms5611_osr_t osr);
ms5611_osr_t getOversampling(void);
double rp;

uint16_t C[6];
uint8_t ct;
uint8_t uosr;
int32_t TEMP2;
int64_t OFF2, SENS2;

void reset(void);
void readPROM(void);

uint16_t readRegister16(uint8_t reg);
uint32_t readRegister24(uint8_t reg);

bool MS5611_Init(ms5611_osr_t osr)
{
    Wire.begin();

    reset();

    setOversampling(osr);

    delay(100);

    readPROM();

    return true;
}

// Set oversampling value
void  setOversampling(ms5611_osr_t osr)
{
    switch (osr)
    {
	case MS5611_ULTRA_LOW_POWER:
	    ct = 1;
	    break;
	case MS5611_LOW_POWER:
	    ct = 2;
	    break;
	case MS5611_STANDARD:
	    ct = 3;
	    break;
	case MS5611_HIGH_RES:
	    ct = 5;
	    break;
	case MS5611_ULTRA_HIGH_RES:
	    ct = 10;
	    break;
    }

    uosr = osr;
}

// Get oversampling value
ms5611_osr_t getOversampling(void)
{
    return (ms5611_osr_t)uosr;
}

void reset(void)
{
    Wire.beginTransmission(MS5611_ADDRESS);

    #if ARDUINO >= 100
	Wire.write(MS5611_CMD_RESET);
    #else
	Wire.send(MS5611_CMD_RESET);
    #endif

    Wire.endTransmission();
}

void  readPROM(void)
{
    for (uint8_t offset = 0; offset < 6; offset++)
    {
	C[offset] = readRegister16(MS5611_CMD_READ_PROM + (offset * 2));
    }
}

uint32_t readRawTemperature(void)
{
    Wire.beginTransmission(MS5611_ADDRESS);

    #if ARDUINO >= 100
	Wire.write(MS5611_CMD_CONV_D2 + uosr);
    #else
	Wire.send(MS5611_CMD_CONV_D2 + uosr);
    #endif

    Wire.endTransmission();

    delay(ct);

    return readRegister24(MS5611_CMD_ADC_READ);
}

uint32_t readRawPressure(void)
{
    Wire.beginTransmission(MS5611_ADDRESS);

    #if ARDUINO >= 100
	Wire.write(MS5611_CMD_CONV_D1 + uosr);
    #else
	Wire.send(MS5611_CMD_CONV_D1 + uosr);
    #endif

    Wire.endTransmission();

    delay(ct);

    return readRegister24(MS5611_CMD_ADC_READ);
}

// Read 16-bit from register (oops MSB, LSB)
uint16_t readRegister16(uint8_t reg)
{
    uint16_t value;
    Wire.beginTransmission(MS5611_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();

    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.requestFrom(MS5611_ADDRESS, 2);
    while(!Wire.available()) {};
    #if ARDUINO >= 100
        uint8_t vha = Wire.read();
        uint8_t vla = Wire.read();
    #else
        uint8_t vha = Wire.receive();
        uint8_t vla = Wire.receive();
    #endif;
    Wire.endTransmission();

    value = vha << 8 | vla;

    return value;
}

// Read 24-bit from register (oops XSB, MSB, LSB)
uint32_t readRegister24(uint8_t reg)
{
    uint32_t value;
    Wire.beginTransmission(MS5611_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();

    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.requestFrom(MS5611_ADDRESS, 3);
    while(!Wire.available()) {};
    #if ARDUINO >= 100
        uint8_t vxa = Wire.read();
        uint8_t vha = Wire.read();
        uint8_t vla = Wire.read();
    #else
        uint8_t vxa = Wire.receive();
        uint8_t vha = Wire.receive();
        uint8_t vla = Wire.receive();
    #endif;
    Wire.endTransmission();

    value = ((int32_t)vxa << 16) | ((int32_t)vha << 8) | vla;

    return value;
}


#endif // MS5611
