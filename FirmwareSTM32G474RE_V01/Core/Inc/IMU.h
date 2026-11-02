/*
 * Imu.h
 *
 *  Created on: Jun 5, 2026
 *      Author: HP
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include <stdint.h>
//#include "FC.h"
#define  IMU_LOOP_HZ (400.0f)
#define IMU_PERIOD	(1.0f/IMU_LOOP_HZ)
#define ONE_RAD 57.29578f
#define ONE_DEG 0.017453f
#define Pi 3.14159f
#define SCALE_GYRO (16.4f)
#define  SCALE_ACCEL (981.0f/4096.0f) //Cm/ss
#define Frq_Update (400)

//------------IMU-BARO-MAG-------------------
typedef struct
{     uint8_t SensorModel; //Raw Data
	  uint8_t Status;
	  int16_t Temp; // 0.1C
	  float accelTravel;
	  float Accel[3];  // cm/ss
  	  float AccelHover[3]; //cm/ss
	  float Gyro[3]; // Rad/s
	  float GyroOffset[3];
      float GyroRate[3]; // Rad/s
      float ZGroundRate; // Rad
      float GyroAcc[3];// Rad/ss
  	  float VAccel[3]; //cm/s
  	  float VxyAccel;
	  int16_t Mag[3];
	  uint8_t MagCalib;
	  float MagHeading;
	  uint8_t ExMagDetected;
	  uint8_t ImuUpdated;
	  uint8_t MagUpdated;
	  uint8_t BaroUpdated;
	  uint32_t ImuUpdatedTick; //ms
	  uint32_t MagExUpdatedTick; //ms
}
Imu_t;

void LowpassAccel(const float* accel,float* accelOut);
void LowpassGyro(const float* SpeedGyro,float* GyroLP);
void LowpassGyroAcc(const float* GyroLPin,float* GyroA);
uint8_t GyroOffsetLoopRaw(float *GyroRaw ,float *accel,float *GyroOffsetRaw,float *accelTravelZ);
#endif /* INC_IMU_H_ */



