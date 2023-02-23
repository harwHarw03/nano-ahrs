#ifndef ALTIMETER_H
#define ALTIMETER_H

#include <drivers/MS5611.h>
#include "Scheduler/Scheduler.h"
#include "Scheduler/Task.h"
#include "../../lib/definitions.h"
class BAROMETER
{
private:
    float _last_altitude_EAS2TAS;
    float _EAS2TAS;
    // int32_t TEMP2;
public:
    void UpdateBaro();
    void CalculateTemperature(bool compensation = false);
	int32_t readPressure(bool compensation = false);
	double getAltitude(double _pressure, double _referencePress = 101325);
	double getSeaLevel(double _pressure, double _altitude);
    double temperature, altitude, pressure;
    void Init();
};
// BAROMETER *BAROMETER::instance = nullptr;

BAROMETER baro;

void BAROMETER::CalculateTemperature(bool compensation)
{
    uint32_t D2 = readRawTemperature();
    int32_t dT = D2 - (uint32_t)C[4] * 256;
    int32_t TEMP = 2000 + ((int64_t) dT * C[5]) / 8388608;
    TEMP2 = 0;
    if (compensation){  // datasheet page 8-9
        if (TEMP < 2000){
            TEMP2 = (dT * dT) / (2 << 30);
        }
    }
    TEMP -= TEMP2;
    temperature =  ((double)TEMP/100);
}

int32_t BAROMETER::readPressure(bool compensation)
{
    uint32_t D1 = readRawPressure();

    uint32_t D2 = readRawTemperature();
    int32_t dT = D2 - (uint32_t)C[4] * 256;

    int64_t OFF = (int64_t)C[1] * 65536 + (int64_t)C[3] * dT / 128;
    int64_t SENS = (int64_t)C[0] * 32768 + (int64_t)C[2] * dT / 256;

    if (compensation)
    {
	int32_t TEMP = 2000 + ((int64_t) dT * C[5]) / 8388608;

	OFF2 = 0;
	SENS2 = 0;

	if (TEMP < 2000)
	{
	    OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
	    SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
	}

	if (TEMP < -1500)
	{
	    OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
	    SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
	}

	OFF = OFF - OFF2;
	SENS = SENS - SENS2;
    }

    uint32_t P = (D1 * SENS / 2097152 - OFF) / 32768;

    return P;
}

double BAROMETER::getAltitude(double _pressure, double _referencePress)
{
    return (44330.0f * (1.0f - pow((double)pressure / (double)_referencePress, 0.1902949f)));
}

// Calculate sea level from Pressure given on specific altitude
double BAROMETER::getSeaLevel(double _pressure, double _altitude)
{
    return ((double)pressure / pow(1.0f - ((double)altitude / 44330.0f), 5.255f));
}

void BAROMETER::Init(){
    MS5611_Init();
    rp = readPressure();
}

void BAROMETER::UpdateBaro(){
    CalculateTemperature();
    pressure = readPressure();
    altitude = getAltitude(pressure, rp);
}

/*Scheduling*/
// class BAROMETER_TASK : public TimingTask
// {
// public:
//     BAROMETER_TASK(uint32_t _rate):rate(_rate){updateTime(millis());}
//     virtual void run(uint32_t now){
//         GetBaroData();
//         _climb_rate_filter.update(baro.altitude, millis());
//         tick(rate);
//     }
// private:
//     uint32_t rate;
// };
// BAROMETER_TASK baro_task(100);

#endif