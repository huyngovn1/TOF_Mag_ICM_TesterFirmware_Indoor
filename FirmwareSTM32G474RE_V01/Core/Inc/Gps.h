/*
 * Gps.h
 *
 *  Created on: Oct 1, 2026
 *      Author: HP
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "main.h"
#include <stdint.h>
#include <string.h>
// 1. Khai báo struct payload chuẩn UBX-NAV-PVT (packed để tránh padding bộ nhớ)
typedef struct __attribute__((packed)) {
    uint32_t iTOW;       // GPS time of week (ms)
    uint16_t year;       // Year
    uint8_t  month;      // Month
    uint8_t  day;        // Day
    uint8_t  hour;       // Hour
    uint8_t  min;        // Minute
    uint8_t  sec;        // Second
    uint8_t  valid;      // Validity flags
    uint32_t tAcc;       // Time accuracy estimate (ns)
    int32_t  nano;       // Fraction of second (-1e9 .. 1e9 ns)
    uint8_t  fixType;    // GNSS fix type (0: no fix, 2: 2D, 3: 3D)
    uint8_t  flags;      // Fix status flags (RTK info ở bit 6-7)
    uint8_t  flags2;     // Additional flags
    uint8_t  numSV;      // Number of satellites used
    int32_t  lon;        // Longitude (deg * 1e7)
    int32_t  lat;        // Latitude (deg * 1e7)
    int32_t  height;     // Height above ellipsoid (mm)
    int32_t  hMSL;       // Height above mean sea level (mm)
    uint32_t hAcc;       // Horizontal accuracy estimate (mm)
    uint32_t vAcc;       // Vertical accuracy estimate (mm)
    int32_t  velN;       // Ned north velocity (mm/s)
    int32_t  velE;       // Ned east velocity (mm/s)
    int32_t  velD;       // Ned down velocity (mm/s)
    int32_t  gSpeed;     // Ground speed (2D) (mm/s)
    int32_t  headMot;    // Heading of motion (2D) (deg * 1e-5)
    uint32_t sAcc;       // Speed accuracy estimate (mm/s)
    uint32_t headAcc;    // Heading accuracy estimate (deg * 1e-5)
    uint16_t pDOP;       // Position DOP (scale 0.01)
    uint8_t  reserved[6];
    int32_t  headVeh;    // Heading of vehicle (2D)
} UBX_NAV_PVT_t;


typedef struct {
  uint8_t Status;
  uint8_t HomeFixed;
  uint32_t TimeWeek;
  double LonSet;
  double LatSet;
  int32_t Lon;
  int32_t Lat;
  int32_t LonOld;
  int32_t LatOld;
  int32_t LonHome;
  int32_t LatHome;
  int32_t DLon;
  int32_t DLat;
  float AltitudeCm; //Cm
  float AltitudeDoppler; //Cm
  float AltiKitCenter; //Cm
  float AltitudeLast; //Cm
  float vx; //cm/s
  float vy; //cm/s
  float vUp; //cm/s
  float VxGPSBody; //cm/s
  float VyGPSBody; //cm/s
  float Speed; //cm/s
  uint32_t HDOP;
  uint32_t VDOP;
  uint16_t pDOP;
  uint8_t Stopped;
  int16_t Course; //0.01Deg
  int16_t Heading; //0.01Deg
  uint32_t TimeoutGPS;
  uint32_t FrameRateMs;
  uint32_t VDOP_Last;
  __IO uint8_t Updated;
} GPS_t;


typedef struct {
  uint8_t Id;
  uint8_t Cmd;
  int32_t Lon;
  int32_t Lat;
  float VLon;
  float VLat;
  float AltitudeCm;
  int16_t Heading; //0.01Deg
  uint8_t Updated;
  uint32_t Timeout; //400Hz Tick
  uint32_t FrameRate;
} AirLink_t;



void M10N_GPS_Config(UART_HandleTypeDef *huart) ;
uint8_t  GPS_NAV_POSLLH_Decoder(GPS_t* GPSin, uint8_t *Buf);
uint8_t Parse_And_Update_PVT( const uint8_t *uart_buf, UBX_NAV_PVT_t *pvt_dest);
uint8_t AirLinkEcoder(const GPS_t *gps,uint8_t ID,uint8_t cmd,uint8_t *buf);
uint8_t AirLinkDecoder(const uint8_t *buf, AirLink_t *gps);
uint8_t AntiDiftGps(const float vRef,const float vNoise,float *Error, uint32_t dt);
uint8_t AntiDropGps(const float vRef,const float vNoise,float *ErrorPos, uint32_t dt);
uint8_t CheckEndRtkPacket(uint8_t *Buf);
void GpsConfig(UART_HandleTypeDef *huart);
#endif /* INC_GPS_H_ */
