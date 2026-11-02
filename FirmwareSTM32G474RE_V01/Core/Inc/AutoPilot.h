/*
 * AutoPilot.h
 *
 *  Created on: Jun 24, 2026
 *      Author: HP
 */

#ifndef APPLICATION_USER_CORE_AUTOPILOT_H_
#define APPLICATION_USER_CORE_AUTOPILOT_H_

#include "main.h"
#include <stdbool.h>
#include <Rx.h>
#include "IMU.h"
#include "Gps.h"

uint8_t NAV_POSLLH_Decoder(uint8_t *Buf);
void AutoPilot(void);
float  GotoOnePoint(int32_t Lon, int32_t Lat,float SpeedCms,float Acc,uint32_t HoldMs);



typedef struct {
  uint32_t Tof[6];// mm
  int16_t  TofSpeed[3]; //mm/s
  uint32_t OpticalDistance;// mm
  int16_t  OpticalMove[3];
  int32_t LonHome;
  int32_t LatHome;
  double LonSet;
  double LatSet;
  float AltitudeCm;
  float AltitudeOffsetCm;
  uint32_t HDOP;
  uint32_t VDOP;
  int16_t Speed; //Cm/s
  int16_t Course;
  uint32_t TimeoutGPS;
  uint32_t FrameUpdateMs;
} Sensors_t;

#include "main.h"
#include <stdbool.h>
#include <math.h>

#define  GYRO_MODE 1
#define  ALTI_ANGLE_MODE 2
#define  GPS_MODE 3
#define  INDOOR_MODE 4
#define  WAYPOINT_MODE 5
#define  GOHOME_MODE 6

#define  WAYPOINT_BANK 2
#define  MAX_POINT_OF_BANK 512
#define  LIMIT_ACELL_BODY  (150.0f/1.11f) // 50Hz cms/s
#define  LIMIT_SPEED  1600.0f   // 16m/s
#define  ONE_DEG  0.017453f

#define  NOT_GPS_NOT_ALTI  0
#define  NOT_ALTI  1
#define  NOT_GPS  2
#define  GPS_HOVERING 3
#define  GPS_MOVING 4
#define  GOHOME_UP 5
#define  GOHOME_MOVING 6
#define  GOHOME_DOWN 7
#define  WAYPOINT_MOVING 8
#define  WAYPOINT_DONE 9

#define  POS_ERROR 15.0f
#define  BARO_NOISE 0.0f

#define  RADAR_NOISE 8.0f

#define  LIMIT_ALTITUDE 6000.0f //cm
//#define LOWBAT_BY_THROTTLE_LEVEL
//#define RADAR_ALTI_HOVER

#define GPS_BASE_MODE 2  // 1: Trimmer Only, 2: Follow

uint8_t NAV_POSLLH_Decoder(uint8_t *Buf);
void AutoPilot(void);
float  GotoOnePoint(int32_t Lon, int32_t Lat,float SpeedCms,float Acc,uint32_t HoldMs);



typedef struct {
  uint8_t HomeFixed;//
  uint32_t TimeWeek;
  int32_t Lon;
  int32_t Lat;
  int32_t LonLast;
  int32_t LatLast;
  float AltitudeCm;
  float AltitudeCmLast;
  float VLonGPSBase;
  float VLatGPSBase;
  float VZ_GPS_Base;

  uint32_t HDOP;
  uint32_t VDOP;
  uint32_t TimeoutGPS;
  int16_t xMag;
  int16_t yMag;
  int16_t zMag;
  int32_t AltitudemmBaro;
} GPS_Base_t;

typedef struct {
  float SpeedCms[MAX_POINT_OF_BANK];
  //uint16_t Heading[MAX_POINT_OF_BANK];
  uint16_t NumberPoints;
  float Hold[MAX_POINT_OF_BANK];
  float SparaySpeed[MAX_POINT_OF_BANK];
  int16_t AUX1[MAX_POINT_OF_BANK];
  int16_t AUX2[MAX_POINT_OF_BANK];
  float Yaw[MAX_POINT_OF_BANK];
  int32_t Lon[MAX_POINT_OF_BANK];
  int32_t Lat[MAX_POINT_OF_BANK];
  float TakeOffAltitude; //Cm
  float Altitude[MAX_POINT_OF_BANK]; //Cm

  int16_t AUX3[MAX_POINT_OF_BANK];
  int16_t AUX4[MAX_POINT_OF_BANK];
  int16_t Active1[MAX_POINT_OF_BANK];
  int16_t Active2[MAX_POINT_OF_BANK];

  int8_t Active3[MAX_POINT_OF_BANK];
  int8_t Active4[MAX_POINT_OF_BANK];

  uint8_t ON1[MAX_POINT_OF_BANK];
  uint8_t ON2[MAX_POINT_OF_BANK];
  uint8_t ON3[MAX_POINT_OF_BANK];
  uint8_t ON4[MAX_POINT_OF_BANK];

  uint16_t Distance[MAX_POINT_OF_BANK];
  uint8_t  Shutter[MAX_POINT_OF_BANK];
  uint8_t  Trigger[MAX_POINT_OF_BANK];
  uint8_t  Empty[MAX_POINT_OF_BANK];
  uint8_t  Empty2[MAX_POINT_OF_BANK];
  uint8_t  Empty3[MAX_POINT_OF_BANK];
  uint8_t  Empty4[MAX_POINT_OF_BANK];

  float  Speed;  //Cms
  uint16_t  Cmd[MAX_POINT_OF_BANK];// Testonly
  uint8_t Repeat;
  uint16_t PointFlying;
} WayPoint;

uint8_t GPS_NAV_POSLLH_Decoder(GPS_t* GPSin, uint8_t *Buf);


#endif /* APPLICATION_USER_CORE_AUTOPILOT_H_ */
