#ifndef IMU_H
#define IMU_H
#include <drivers/MPU6050.h>
// #include <AP_Math.h>
#include "../Scheduler/Scheduler.h"
#include "../Scheduler/Task.h"

#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.0f) // 2 * integral gain

class IMU
{
public : 
  float axg, ayg, azg, gxrs, gyrs, gzrs;
  float roll, pitch, yaw;
  void UpdateQuaternion();
  void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
  void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz); 
  // void RotationMatrix(Matrix3f &m)const;
  void GetRollPitchYaw();

  void UpdateIMU(); 

private:
  volatile float twoKp = twoKpDef;    // 2 * proportional gain (Kp)
  volatile float twoKi = twoKiDef;    // 2 * integral gain (Ki)
  volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // quaternion of sensor frame relative to auxiliary frame
  volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki
  float sampleFreq = 0.0f;                              // integration interval for both filter schemes
  uint32_t lastUpdate = 0, firstUpdate = 0;         // used to calculate integration interval
  uint32_t Now = 0;                                 // used to calculate integration interval

};
IMU imu;

void IMU::UpdateQuaternion()
{
    axg = (float)(AcX - Cal_AcX) / MPU6050_AXGAIN;
    ayg = (float)(AcY - Cal_AcY) / MPU6050_AYGAIN;
    azg = (float)(AcZ - Cal_AcZ) / MPU6050_AZGAIN;
    gxrs = (float)(GyX - Cal_GyX) / MPU6050_GXGAIN * 0.01745329;   //degree to radians
    gyrs = (float)(GyY - Cal_GyY) / MPU6050_GYGAIN * 0.01745329;   //degree to radians
    gzrs = (float)(GyZ - Cal_GyZ) / MPU6050_GZGAIN * 0.01745329;   //degree to radians
}

void IMU::MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
    float norm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    ax /= norm;
    ay /= norm;
    az /= norm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;
  
    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }
  
  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx); 
  
  // Normalise quaternion
  norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= norm;
  q1 /= norm;
  q2 /= norm;
  q3 /= norm;
}

void IMU::MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = sqrt(ax * ax + ay * ay + az * az);
		ax /= recipNorm;
		ay /= recipNorm;
		az /= recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = sqrt(mx * mx + my * my + mz * mz);
		mx /= recipNorm;
		my /= recipNorm;
		mz /= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 /= recipNorm;
	q1 /= recipNorm;
	q2 /= recipNorm;
	q3 /= recipNorm;
}
/*
void IMU::RotationMatrix(Matrix3f &m) const{
    const float q2q2 = q2 * q2;
    const float q2q3 = q2 * q3;
    const float q1q1 = q1 * q1;
    const float q1q2 = q1 * q2;
    const float q1q3 = q1 * q3;
    const float q0q1 = q0 * q1;
    const float q0q2 = q0 * q2;
    const float q0q3 = q0 * q3;
    const float q3q3 = q3 * q3;

    m.a.x = 1.0f-2.0f*(q2q2 + q3q3);
    m.a.y = 2.0f*(q1q2 - q0q3);
    m.a.z = 2.0f*(q1q3 + q0q2);
    m.b.x = 2.0f*(q1q2 + q0q3);
    m.b.y = 1.0f-2.0f*(q1q1 + q3q3);
    m.b.z = 2.0f*(q2q3 - q0q1);
    m.c.x = 2.0f*(q1q3 - q0q2);
    m.c.y = 2.0f*(q2q3 + q0q1);
    m.c.z = 1.0f-2.0f*(q1q1 + q2q2);
}*/

void IMU::GetRollPitchYaw()
{
    // yaw = -atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.29577951;
    // pitch = asin(2.0f * (q1 * q3 - q0 * q2)) * 57.29577951;
    // roll = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 57.29577951;
    //source wikipedia quaternion to euler angels  
    roll = atan2((2.0f * (q0 * q1 + q2 * q3)), (1.0f - 2.0f * (q1 * q1 + q2 * q2))) * 57.29577951;
    pitch = (2.0f * atan2((sqrt(1 + 2 * (q0 * q2 - q1 * q3))), (sqrt(1.0f - 2.0f * (q0 * q2 - q1 * q3)))) - M_PI / 2.0f )* 57.29577951;
    yaw = atan2(( 2.0f * (q0 * q3 + q1 * q2)), (1.0f - 2.0f * (q2 * q2 + q3 * q3))) * 57.29577951;
}

void IMU::UpdateIMU(){
    // int16_t mx, my, mz;
    // compass.getHeading(&mx, &my, &mz);
    GetIMUData();
    UpdateQuaternion();
    Now = micros();
    sampleFreq = (1000000.0f / (Now - lastUpdate));
    lastUpdate = Now;
    //get rpy from imu and magnetometer using mahony
    MahonyAHRSupdateIMU(gxrs, gyrs, gzrs, axg, ayg, azg);//, mx, my, mz);
    GetRollPitchYaw();
}

/*Scheduling*/
// class IMU_TASK : public TimingTask
// {
//   public : 
//   IMU_TASK(uint32_t _rate):rate(_rate){updateTime(millis());}
//   virtual void run (){
//     imu.UpdateIMU();
//     tick(rate);
//   }
//   private:
//   uint32_t rate;
// };
// IMU_TASK imu_task(10);



#endif