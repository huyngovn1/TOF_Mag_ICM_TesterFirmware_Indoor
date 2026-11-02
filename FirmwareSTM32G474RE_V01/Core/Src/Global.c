/*
 * Global.cpp
 *
 *  Created on: May 3, 2026
 *      Author: HP
 */

#include "Global.h"
#include <math.h>

float getAltitude(double press, double temp) {
  return ((pow((1013.25 / press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
}

uint8_t RingLimit(float *x, float *y,float Limit,float *RingOut){
                    float tmpring;
                    tmpring=((*x)*(*x)) + ((*y)*(*y));
                    tmpring=sqrtf(tmpring);
                    *RingOut=tmpring;
                    if(tmpring>Limit){
                            *x=(*x/tmpring)*Limit;
                            *y=(*y/tmpring)*Limit;
                            *RingOut=Limit;
                            return 1;
                    }
                    return 0;
}
