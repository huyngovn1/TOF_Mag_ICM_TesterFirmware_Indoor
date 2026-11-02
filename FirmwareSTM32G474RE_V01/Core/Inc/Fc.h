/*
 * Fc.h
 *
 *  Created on: Jun 24, 2026
 *      Author: HP
 */

#ifndef APPLICATION_USER_CORE_FC_H_
#define APPLICATION_USER_CORE_FC_H_
#define VHDOP_HOME (5000)
#define THRO_MIN (1110*4)
#define THRO_MAX (1930*4)

#include <stdint.h>
void HeadSensorConvert (uint8_t Heading,float *Gyro,float *Accel);
void FIFOStabling_DualIMU(float *GyroRef);
void FlightControlLoop(void);
void FlightControlLoop2(void);
uint8_t Mixer(int16_t x_con ,int16_t y_con ,int16_t r_con,int16_t ThroCon );

typedef struct
{
    uint32_t  time_ms;		    // System time in ms
    uint32_t  distance;		    // distance(mm), 0 Indicates unavailable
    uint8_t   strength;	            // signal strength
    uint8_t   precision;	    // distance precision
    uint8_t   dis_status;	    // distance status
    uint8_t  reserved1;	            // reserved
    int16_t   flow_vel_x;	    // optical flow velocity in x
    int16_t   flow_vel_y;	    // optical flow velocity in y
    uint8_t   flow_quality;	    // optical flow quality
    uint8_t   flow_status;	    // optical flow status
    uint16_t  reserved2;	    // reserved

    float vx;
    float vy;

    int32_t moved_x;
    int32_t moved_y;
    uint8_t Updated;

} MTF_t;

typedef struct
{
    uint32_t  time_ms[6];		    // System time in ms
    uint32_t  distance[6];		    // distance(mm), 0 Indicates unavailable
    int16_t   flow_vel[6];	    // optical flow velocity in x
    int32_t speed[6];
    uint8_t Updated;
} TOF_t;


typedef struct
{
	uint8_t State;
	uint8_t ErrorFlags;
	uint8_t Armed;
	uint32_t TxIdleTime;
	float speedControl[3]; //cm/s
	float speedBodyControl;
	float Angle[3]; // Rad
    float AngleFlybar[3];
    float AngleHover[3];
	float Heading; // Compass-Rad
	uint16_t ThroHover;
	uint8_t OverThrottle;
	uint8_t EscTeleDetected;
	uint16_t EscBatVol;  //10mV
	uint8_t BatCell;
	uint8_t PercentBat;
	uint16_t BecVol;  //1mV
	uint16_t BatVolMin;  //10mV
	uint16_t BecVolMin;  //1mV
	uint16_t EscCurrent[8]; //10mA
	int8_t EscTemp;
    int32_t mAhComsum;
    uint16_t eRpm;
    uint8_t ExMagDetected; //0 Internal, 1 Ex,
    uint8_t  FlyingMode;
    uint8_t  FlyingStatus;
    uint32_t FlyingTime;
    int16_t TakeoffAngle; //0.1Deg
    float DistanceHomeCm;
    float posError[3];
	uint8_t GoingHomeStep;
} Fc_t;

typedef struct
{
	uint8_t FrameSize;
	uint8_t ImuFilterFrq;
	uint8_t RcDeadBand;
	uint8_t ImuOrient;
	uint8_t CompassOrient;
	int8_t  GpsPos; //Cm
	int8_t  FcPosY;  // Behind MainSharf Cm
	int8_t  FcPosZ;  // Under Main Blade cm
	uint8_t ThroOutType; // 0 PWM, 1 DSHOT
	uint16_t ThroMin;
	uint16_t ThroMax;
	uint8_t  ThroIdling;
	uint8_t Response[3]; //By Joystick
	uint8_t Rate[4];  //Roll,Pit,Yaw,Updown
	uint8_t GainRoll;
	uint8_t GainRollP;
	uint8_t GainRollI;
	uint8_t GainRollD;
	uint8_t GainPitch;
	uint8_t GainPitchP;
	uint8_t GainPitchI;
	uint8_t GainPitchD;
	uint8_t GainYaw;
	uint8_t GainYawP;
	uint8_t GainYawI;
	uint8_t GainYawD;
	uint8_t RollRate;
	uint8_t PitchRate;
	uint8_t YawRate;
} FcConfig_t;


#endif /* APPLICATION_USER_CORE_FC_H_ */
