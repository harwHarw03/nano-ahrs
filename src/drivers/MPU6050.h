/*
references : 
MadgwickAHRS Filter Algorithm : http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
Jeff Rowberg's MPU6050 : https://github.com/jrowberg/i2cdevlib
Davide Gironi's AVR atmega MPU6050 : http://davidegironi.blogspot.com/2013/02/avr-atmega-mpu6050-gyroscope-and.html#.W7zM7mgzaUk
*/
#ifndef MPU6050_H
#define MPU6050_H
#include <Arduino.h>
#include "MPU6050_Register.h"
#include <Wire.h>
#include "../Eeprom/rwEEPROM.h"
#include <simple_i2c.h>
#include "delay_millis.h"

// accel & gyro data
#define MPU6050_AXOFFSET 293
#define MPU6050_AYOFFSET -53
#define MPU6050_AZOFFSET -13 // AcX = 242 | AcY = -55 | AcZ = -669 | 
//#define MPU6050_AXOFFSET 0
//#define MPU6050_AYOFFSET 0
//#define MPU6050_AZOFFSET 0
//#define MPU6050_AXGAIN 16384.0 // AFS_SEL = 0, +/-2g, MPU6050_ACCEL_FS_2
//#define MPU6050_AYGAIN 16384.0 // AFS_SEL = 0, +/-2g, MPU6050_ACCEL_FS_2
//#define MPU6050_AZGAIN 16384.0 // AFS_SEL = 0, +/-2g, MPU6050_ACCEL_FS_2
//#define MPU6050_AXGAIN 8192.0 // AFS_SEL = 1, +/-4g, MPU6050_ACCEL_FS_4
//#define MPU6050_AYGAIN 8192.0 // AFS_SEL = 1, +/-4g, MPU6050_ACCEL_FS_4
//#define MPU6050_AZGAIN 8192.0 // AFS_SEL = 1, +/-4g, MPU6050_ACCEL_FS_4
#define MPU6050_AXGAIN 4096.0 // AFS_SEL = 2, +/-8g, MPU6050_ACCEL_FS_8
#define MPU6050_AYGAIN 4096.0 // AFS_SEL = 2, +/-8g, MPU6050_ACCEL_FS_8
#define MPU6050_AZGAIN 4096.0 // AFS_SEL = 2, +/-8g, MPU6050_ACCEL_FS_8
//#define MPU6050_AXGAIN 2048.0 // AFS_SEL = 3, +/-16g, MPU6050_ACCEL_FS_16
//#define MPU6050_AYGAIN 2048.0 // AFS_SEL = 3, +/-16g, MPU6050_ACCEL_FS_16
//#define MPU6050_AZGAIN 2048.0 // AFS_SEL = 3, +/-16g, MPU6050_ACCEL_FS_16
#define MPU6050_GXOFFSET -31
#define MPU6050_GYOFFSET 55
#define MPU6050_GZOFFSET 18   //GyX = -31 | GyY = -19 | GyZ = -19
//#define MPU6050_GXOFFSET 0
//#define MPU6050_GYOFFSET 0
//#define MPU6050_GZOFFSET 0
//#define MPU6050_GXGAIN 131.072 // FS_SEL = 0, +/-250degree/s, MPU6050_GYRO_FS_250
//#define MPU6050_GYGAIN 131.072 // FS_SEL = 0, +/-250degree/s, MPU6050_GYRO_FS_250
//#define MPU6050_GZGAIN 131.072 // FS_SEL = 0, +/-250degree/s, MPU6050_GYRO_FS_250
//#define MPU6050_GXGAIN 65.536 // FS_SEL = 1, +/-500degree/s, MPU6050_GYRO_FS_500
//#define MPU6050_GYGAIN 65.536 // FS_SEL = 1, +/-500degree/s, MPU6050_GYRO_FS_500
//#define MPU6050_GZGAIN 65.536 // FS_SEL = 1, +/-500degree/s, MPU6050_GYRO_FS_500
//#define MPU6050_GXGAIN 32.768 // FS_SEL = 2, +/-1000degree/s, MPU6050_GYRO_FS_1000
//#define MPU6050_GYGAIN 32.768 // FS_SEL = 2, +/-1000degree/s, MPU6050_GYRO_FS_1000
//#define MPU6050_GZGAIN 32.768 // FS_SEL = 2, +/-1000degree/s, MPU6050_GYRO_FS_1000
#define MPU6050_GXGAIN 16.384 // FS_SEL = 3, +/-2000degree/s, MPU6050_GYRO_FS_2000
#define MPU6050_GYGAIN 16.384 // FS_SEL = 3, +/-2000degree/s, MPU6050_GYRO_FS_2000
#define MPU6050_GZGAIN 16.384 // FS_SEL = 3, +/-2000degree/s, MPU6050_GYRO_FS_2000
// float SelfTest[6];

/*Variables*/
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int16_t Cal_AcX, Cal_AcY, Cal_AcZ, Cal_Tmp, Cal_GyX, Cal_GyY, Cal_GyZ;
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer
float sampleFreq = 0.0f;                              // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0;         // used to calculate integration interval
uint32_t Now = 0;                                 // used to calculate integration interval
int16_t address[6];
int16_t output[6]; //AcX, AcY, AcZ, GyX, GyY, GyZ

void MPU6050_Init(bool _calibrate, bool _use_eeprom);
void GetIMUData();
void calibrateMPU6050(bool _save_to_eeprom);
void SelfTest(float * destination);

void MPU6050_Init(bool _calibrate, bool _use_eeprom)
{
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x00);
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x01);
    // WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x00);    // MPU6050 Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV) for DMP
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x18);   //FS_SEL=3, +- 2000 ^/s
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x10);  //AFS_SEL=2, +- 8g
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, 0x00);    //Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz
    
    // manual offset
    Cal_AcX = MPU6050_AXOFFSET;
    Cal_AcY = MPU6050_AYOFFSET;
    Cal_AcZ = MPU6050_AZOFFSET;
    Cal_GyX = MPU6050_GXOFFSET;
    Cal_GyY = MPU6050_GYOFFSET;
    Cal_GyZ = MPU6050_GZOFFSET;

    if (!_use_eeprom){
        if (_calibrate) calibrateMPU6050(0);
    } else {
        if (_calibrate) calibrateMPU6050(1);
        EEPROM_Read(0, Cal_AcX);
        EEPROM_Read(10, Cal_AcY);
        EEPROM_Read(20, Cal_AcZ);
        EEPROM_Read(30, Cal_GyX);
        EEPROM_Read(40, Cal_GyY);
        EEPROM_Read(50, Cal_GyZ);  
    }
    /*
    case : 
    1. No use eeprom, no calibrate (manual edit to MPU6050_**OFFSET)
    2. No use eeprom, calibrate (auto calibrate without reading or saving to eeprom)
    3. Use eeprom, no calibrate (use previous calibration data)
    4. Use eeprom, calibrate (calibrate and save to eeprom)
    */
}

void GetIMUData()
{
    uint8_t data_raw[14];
    ReadBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 14, &data_raw[0]);
    AcX = data_raw[0] << 8 | data_raw[1];
    AcY = data_raw[2] << 8 | data_raw[3];
    AcZ = data_raw[4] << 8 | data_raw[5];
    Tmp = data_raw[6] << 8 | data_raw[7];
    GyX = data_raw[8] << 8 | data_raw[9];
    GyY = data_raw[10] << 8 | data_raw[11];
    GyZ = data_raw[12] << 8 | data_raw[13];
}

void calibrateMPU6050(bool _save_to_eeprom)
{
    Cal_AcX = 0;
    Cal_AcY = 0;
    Cal_AcZ = 0;
    Cal_GyX = 0;
    Cal_GyY = 0;
    Cal_GyZ = 0;
        // uint8_t data_raw[14];
    for (int i = 0; i < 2000; i++){
        if(i % 200 == 0) Serial.println("Calculating .....");
        GetIMUData();
        // ReadBytes(MPU6050_DEFAULT_ADDRESS, 0x3B, 14, &data_raw[0]);
        // AcX = data_raw[0] << 8 | data_raw[1];
        // AcY = data_raw[2] << 8 | data_raw[3];
        // AcZ = data_raw[4] << 8 | data_raw[5];
        // Tmp = data_raw[6] << 8 | data_raw[7];
        // GyX = data_raw[8] << 8 | data_raw[9];
        // GyY = data_raw[10] << 8 | data_raw[11];
        // GyZ = data_raw[12] << 8 | data_raw[13];
        DelayMillis(10);

        Cal_AcX += AcX;
        Cal_AcY += AcY;
        Cal_AcZ += AcZ;
        Cal_GyX += GyX;
        Cal_GyY += GyY;
        Cal_GyZ += GyZ;
    }
    Cal_AcX /= 2000;
    Cal_AcY /= 2000;
    Cal_AcZ /= 2000;
    Cal_GyX /= 2000;
    Cal_GyY /= 2000;
    Cal_GyZ /= 2000;

    if (_save_to_eeprom){
        EEPROM_Write(0, Cal_AcX);
        EEPROM_Write(20, Cal_AcY);
        EEPROM_Write(40, Cal_AcZ);
        EEPROM_Write(60, Cal_GyX);
        EEPROM_Write(80, Cal_GyY);
        EEPROM_Write(100, Cal_GyZ);
        Serial.println("Saved to eeprom");
    }
}

void SelfTest(float * destination)
{
    uint8_t rawData[4];
    uint8_t selfTest[6];
    float factoryTrim[6];
    
    // Configure the accelerometer for self-test
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
    WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    DelayMillis(250);  // Delay a while to let the device execute the self-test
    rawData[0] = ReadByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_X); // X-axis self-test results
    rawData[1] = ReadByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_Y); // Y-axis self-test results
    rawData[2] = ReadByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_Z); // Z-axis self-test results
    rawData[3] = ReadByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_A); // Mixed-axis self-test results
    // Extract the acceleration test results first
    selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
    selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2 ; // YA_TEST result is a five-bit unsigned integer
    selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) ; // ZA_TEST result is a five-bit unsigned integer
    // Extract the gyration test results first
    selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
    selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
    selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer   
    // Process results to allow final comparison with factory set values
    factoryTrim[0] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[0] - 1.0)/30.0))); // FT[Xa] factory trim calculation
    factoryTrim[1] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[1] - 1.0)/30.0))); // FT[Ya] factory trim calculation
    factoryTrim[2] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[2] - 1.0)/30.0))); // FT[Za] factory trim calculation
    factoryTrim[3] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[3] - 1.0) ));             // FT[Xg] factory trim calculation
    factoryTrim[4] =  (-25.0*131.0)*(pow( 1.046 , ((float)selfTest[4] - 1.0) ));             // FT[Yg] factory trim calculation
    factoryTrim[5] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[5] - 1.0) ));             // FT[Zg] factory trim calculation
    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get to percent, must multiply by 100 and subtract result from 100
    for (int i = 0; i < 6; i++) {
        destination[i] = 100.0 + 100.0*((float)selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
    }  
}

#endif