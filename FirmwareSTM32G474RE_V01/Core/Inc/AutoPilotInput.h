/*
 * AutoPilotInput.h
 *
 *  Created on: Jul 6, 2026
 *      Author: HP
 */

#ifndef INC_AUTOPILOTINPUT_H_
#define INC_AUTOPILOTINPUT_H_
#include "main.h"
void CompassCab(int16_t * MagOffset );
void MagUpdateAngle(int16_t* MagRaw);
uint8_t XyPosition3Sensors2(uint32_t *mmIn,float *VxAccel, float *VyAccel);

#endif /* INC_AUTOPILOTINPUT_H_ */
