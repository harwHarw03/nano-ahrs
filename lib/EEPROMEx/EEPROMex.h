/*
  EEPROMEx.h - Extended EEPROM library
  Copyright (c) 2012 Thijs Elenbaas.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef EEPROMEX_h
#define EEPROMEX_h

#if ARDUINO >= 100
#include <Arduino.h> 
#else
#include <WProgram.h> 
#endif
#include <inttypes.h>
#include <avr/eeprom.h>

#define EEPROMSize1k         1024
#define EEPROMSize2k         2048
#define EEPROMSize4k         4096
#define EEPROMSize8k         8192
#define EEPROMSize16k        16384
#define EEPROMSize32k        32768
#define EEPROMSize64k        65536

#define EEPROMSizeATmega168   512     
#define EEPROMSizeATmega328   1024     
#define EEPROMSizeATmega1280  4096     
#define EEPROMSizeATmega32u4  1024
#define EEPROMSizeAT90USB1286 4096
#define EEPROMSizeMK20DX128   2048
#define EEPROMSizeMK20DX256   2048
#define EEPROMSizeATSAMD21G18 16384          

#define EEPROMSizeUno         EEPROMSizeATmega328     
#define EEPROMSizeUnoSMD      EEPROMSizeATmega328
#define EEPROMSizeLilypad     EEPROMSizeATmega328
#define EEPROMSizeDuemilanove EEPROMSizeATmega328
#define EEPROMSizePro         EEPROMSizeATmega328
#define EEPROMSizeFio         EEPROMSizeATmega328
#define EEPROMSizeMega        EEPROMSizeATmega1280
#define EEPROMSizeDiecimila   EEPROMSizeATmega168
#define EEPROMSizeNano        EEPROMSizeATmega168
#define EEPROMSizeTeensy2     EEPROMSizeATmega32u4
#define EEPROMSizeLeonardo    EEPROMSizeATmega32u4
#define EEPROMSizeMicro       EEPROMSizeATmega32u4
#define EEPROMSizeEsplora     EEPROMSizeATmega32u4
#define EEPROMSizeYun         EEPROMSizeATmega32u4
#define EEPROMSizeTre         EEPROMSizeATmega32u4
#define EEPROMSizeZero        EEPROMSizeATSAMD21G18
#define EEPROMSizeTeensy2pp   EEPROMSizeAT90USB1286
#define EEPROMSizeTeensy3     EEPROMSizeMK20DX128
#define EEPROMSizeTeensy31    EEPROMSizeMK20DX256
class EEPROMClassEx
{
	  
  public:
	EEPROMClassEx();
	bool 	 isReady();
	int 	 writtenBytes();
    void 	 setMemPool(int base, int memSize);
	void  	 setMaxAllowedWrites(int allowedWrites);
	int 	 getAddress(int noOfBytes);
    
	uint8_t  read(int);	
	bool 	 readBit(int, byte);
	uint8_t  readByte(int);
    uint16_t readInt(int);
    uint32_t readLong(int);
	float    readFloat(int);
	double   readDouble(int);
			
    bool     write(int, uint8_t);
	bool 	 writeBit(int , uint8_t, bool);
	bool     writeByte(int, uint8_t);
	bool 	 writeInt(int, uint16_t);
	bool 	 writeLong(int, uint32_t);
	bool 	 writeFloat(int, float);
	bool 	 writeDouble(int, double);

	bool     update(int, uint8_t);
	bool 	 updateBit(int , uint8_t, bool);
	bool     updateByte(int, uint8_t);
	bool 	 updateInt(int, uint16_t);
	bool 	 updateLong(int, uint32_t);
	bool 	 updateFloat(int, float);
	bool 	 updateDouble(int, double);

	
    // Use template for other data formats

	/**
	 * Template function to read  multiple items of any type of variable, such as structs
	 */
	template <class T> int readBlock(int address, const T value[], int items)
	{
		unsigned int i;
		for (i = 0; i < (unsigned int)items; i++)
			readBlock<T>(address+(i*sizeof(T)),value[i]);
		return i;
	}

	/**
	 * Template function to read any type of variable, such as structs
	 */	
	template <class T> int readBlock(int address, const T& value)
	{		
		eeprom_read_block((void*)&value, (const void*)address, sizeof(value));
		return sizeof(value);
	}
	
	/**
	 * Template function to write multiple items of any type of variable, such as structs
	 */	
	template <class T> int writeBlock(int address, const T value[], int items)
	{	
		if (!isWriteOk(address+items*sizeof(T))) return 0;
		unsigned int i;
		for (i = 0; i < (unsigned int)items; i++)
			  writeBlock<T>(address+(i*sizeof(T)),value[i]);
		return i;
	}

	/**
	 * Template function to write any type of variable, such as structs
	 */		
	template <class T> int writeBlock(int address, const T& value)
	{
		if (!isWriteOk(address+sizeof(value))) return 0;
		eeprom_write_block((void*)&value, (void*)address, sizeof(value));			  			  
		return sizeof(value);
	}
	 
	/**
	 * Template function to update multiple items of any type of variable, such as structs
	 * The EEPROM will only be overwritten if different. This will reduce wear.
	 */	 
	template <class T> int updateBlock(int address, const T value[], int items)
	{
		int writeCount=0;
		if (!isWriteOk(address+items*sizeof(T))) return 0;
		unsigned int i;
		for (i = 0; i < (unsigned int)items; i++)
			  writeCount+= updateBlock<T>(address+(i*sizeof(T)),value[i]);
		return writeCount;
	}
	
	/**
	 * Template function to update any type of variable, such as structs
	 * The EEPROM will only be overwritten if different. This will reduce wear.
	 */	 
	template <class T> int updateBlock(int address, const T& value)
	{
		int writeCount=0;
		if (!isWriteOk(address+sizeof(value))) return 0;
		const byte* bytePointer = (const byte*)(const void*)&value;
		for (unsigned int i = 0; i < (unsigned int)sizeof(value); i++) {
			if (read(address)!=*bytePointer) {
				write(address, *bytePointer);
				writeCount++;		
			}
			address++;
			bytePointer++;
		}
		return writeCount;
	}
	
	
	
private:
	//Private variables
	static int _base;
	static int _memSize;
	static int _nextAvailableaddress;	
	static int _writeCounts;
	int _allowedWrites;	
	bool checkWrite(int base,int noOfBytes);	
	bool isWriteOk(int address);
	bool isReadOk(int address);
};

// extern EEPROMClassEx EEPROM;

#endif

