#include "Memory.h"

Memory::Memory() {}

void Memory::setupMemory(void) {
    EEPROM.setMemPool(memBase, EEPROMSizeUno);
    EEPROM.setMaxAllowedWrites(maxAllowedWrite);
    delay(100);
    Serial.println("EEPROM is set.");
}

/**
 * @brief Get a new starting address to write to. 
 * 
 * @param[in] type type of data (int, long, float, double, byte).
 * @return return avaliable address. Adress is negative if not enough space is available
 */
int16_t Memory::getTypeAddress(const char* type) {
    if (type == "int") {
        return EEPROM.getAddress(sizeof(int));
    } else if (type == "long") {
        return EEPROM.getAddress(sizeof(long));
    } else if (type == "float") {
        return EEPROM.getAddress(sizeof(float));
    } else if (type == "double") {
        return EEPROM.getAddress(sizeof(double));
    } else if (type == "byte") {
        return EEPROM.getAddress(sizeof(byte));
    } else if (type == "bytearray") {
        return EEPROM.getAddress(sizeof(byte) * 7);
    } else if (type == "chararray") {
        return EEPROM.getAddress(sizeof(char) * 7);
    }
}

/**
 * @brief Write a bits of datatype to memory
 * 
 * @param[in] address address from result getAddress()
 * @param[in] input actual value to save in memory
 * @param[in] type type of data (int, long, float, double, byte)
 */
void Memory::writeToMemory(int16_t address, int16_t input, const char* type) {
    if (type == "int") {
        EEPROM.writeInt(address, input);
    } else if (type == "long") {
        EEPROM.writeLong(address, input);
    } else if (type == "float") {
        EEPROM.writeFloat(address, input);
    } else if (type == "double") {
        EEPROM.writeDouble(address, input);
    } else if (type == "byte") {
        EEPROM.write(address, input);
    }
    Memory::debug(address, input);
}

int16_t Memory::readsInt(int16_t *address) {
    return EEPROM.readInt(*address);
}

float Memory::readsFloat(int16_t *address) {
    return EEPROM.readFloat(*address);
}

int32_t Memory::readsLong(int16_t *address) {
    return EEPROM.readLong(*address);
}

int32_t Memory::readsDouble(int16_t *address) {
    return EEPROM.readDouble(*address);
}

int8_t Memory::readsByte(int16_t *address) {
    return EEPROM.read(*address);
}

void Memory::debug(int16_t address, int16_t value) {
    Serial.print("Address: ");
    Serial.print(address);
    Serial.print("\t Value: ");
    Serial.println(value);
}

Memory::~Memory() {}
