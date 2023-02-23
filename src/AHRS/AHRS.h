#ifndef AHRS_H
#define AHRS_H

/*Include Framework & Dep*/
#ifdef Arduino_h
#include <Arduino.h>
#endif
// #include <TeensyThread.h>

/*Include Drivers*/
#include <drivers/MPU6050.h>
#include <drivers/MS5611.h>

/*Include Modules*/
#include "Barometer.h"
#include "IMU.h"
// #include "Compass.h"

/*Include Extra Libraries*/
// #include <AP_Math.h>
#include <simple_i2c.h>

/*Calibration States*/
#define CALIBRATE_IMU false
#define IMU_USE_EEPROM true

#define CALIBRATE_COMPASS false
#define COMPASS_USE_EEPROM true

#define CALIBRATE_BARO false
#define BARO_USE_EEPROM true

#define CALIBRATE_AIRSPEED false
#define AIRSPEED_USE_EEPROM true

class AHRS
{
    private:
/*Singleton*/
        static AHRS *instance;
        AHRS() {}
        AHRS(const AHRS &) = delete;
        AHRS &operator=(const AHRS &) = delete;
/*
using : AHRS *ahrs = AHRS::getInstance(); 
ex : ahrs->getAltitude
*/        

    public:
/*Singleton Instance*/
        static AHRS *getInstance()
        {   
            if (!instance)
                instance = new AHRS();
            return instance;
        }   
/*Sensors get Instance*/

/*AHRS setup ( Sensors Init, Calib & threads )*/
        void init();
/*Update Calculation Sections*/
        void update();  //with scheduler each sensor
/*Get & Set Parameters*/
        bool have_gps = false;  
/*Get Datas*/
        void get_relative_position_D_home(float &posD);
        double getBaroAltitude(){return baro.altitude;}
        float last_roll;
        float last_pitch;
        float last_yaw;
        float baro_altitude;
        float _heading;
        float _heading_radian;
/*Get Data*/
        bool is_airspeed_sensor_enabled(){return airspeed_sensor_enabled;}
        // float get_EAS2TAS();

        int32_t get_raw_gyro_roll() { return GyX;}
        int32_t get_raw_gyro_pitch() { return GyY;}
        int32_t get_raw_acc_x() { return AcX;}
        int32_t get_raw_acc_y() { return AcY;}
        float get_accel_x(){return imu.axg;}
        float get_accel_y(){return imu.ayg;}
        float get_accel_z(){return imu.azg;}
        float get_gyro_x(){return imu.gxrs;}
        float get_gyro_y(){return imu.gyrs;}
        float get_gyro_z(){return imu.gzrs;}
        float get_roll(){return imu.roll;}
        float get_pitch(){return imu.pitch;}
        float get_yaw(){return imu.yaw;}    
        double get_baro_altitude(){return baro.altitude;}
    
    private:
        bool airspeed_sensor_enabled = false;
        uint32_t _last_ahrs_update;
};
// part of singleton
AHRS *AHRS::instance = nullptr;

/*--Implementation--*/
void AHRS::init()
{
    MPU6050_Init(CALIBRATE_IMU, IMU_USE_EEPROM);
    baro.Init();//(CALIBRATE_BARO, BARO_USE_EEPROM);
}

void AHRS::update(){
    imu.UpdateIMU();
    baro.UpdateBaro();
}

void AHRS::get_relative_position_D_home(float &posD) {
    posD = -baro.altitude;
}

#endif  // AHRS_H