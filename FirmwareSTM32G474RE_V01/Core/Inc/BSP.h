/*
 * BSP.h
 *
 *  Created on: Oct 28, 2026
 *      Author: HP
 */

#ifndef INC_BSP_H_
#define INC_BSP_H_

#define  NOTGOOD_GPS_SIGNAL (GPS.HDOP>500||GPS.VDOP>800 ||UBX_NAV_PVT.pDOP>140)
float getAltitude(double press, double temp);

#endif /* INC_BSP_H_ */
