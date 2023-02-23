#pragma once
#ifndef MEMORY_H
#define MEMORY_H

#include <EEPROMex.h>

class Memory
{
private:
    const uint8_t memBase          = 150;
    const uint8_t maxAllowedWrite  = 150;
public:
    EEPROMClassEx EEPROM;
    Memory();

    void setupMemory(void);

    int16_t getTypeAddress(const char* type);
    void writeToMemory(int16_t address, int16_t input, const char* type);

    int16_t readsInt(int16_t *address);
    float readsFloat(int16_t *address);
    int32_t readsLong(int16_t *address);
    int32_t readsDouble(int16_t *address);
    int8_t readsByte(int16_t *address);

    void debug(int16_t address, int16_t input);
    
    ~Memory();
};



#endif