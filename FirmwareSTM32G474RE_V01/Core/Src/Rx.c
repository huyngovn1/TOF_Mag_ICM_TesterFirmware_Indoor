/*
 * Rx.c
 *
 *  Created on: Jun 24, 2026
 *      Author: HP
 */

#include "Rx.h"
#include "crsf.h"
uint8_t SBUS_Decoder(uint8_t *RxBuffer,uint16_t *RxVal){ //https://github.com/uzh-rpg/rpg_quadrotor_control/wiki/SBUS-Protocol

                                                    //   USART1->TDR = RxBuffer[23];
                                                       uint8_t Sbus_failsafe=RxBuffer[23]& (1<<3);
                                                       uint8_t Sbus_FrameLost=RxBuffer[23]& (1<<2);
//                                                       uint8_t CH17=RxBuffer[23]&0x01;
//                                                       uint8_t CH18=(RxBuffer[23]>>1)&0x01;

							if(Sbus_failsafe==0 &&Sbus_FrameLost==0 &&RxBuffer[0]==0x0F){
                                                          uint16_t SbusVal[8];
                                                          SbusVal[0]=RxBuffer[1]|((uint16_t)RxBuffer[2]<<8);
                                                          SbusVal[1]=(RxBuffer[2]>>3)|((uint16_t)RxBuffer[3]<<5);
                                                          SbusVal[2]=(RxBuffer[3]>>6)|((uint16_t)RxBuffer[4]<<2)|((uint16_t)RxBuffer[5]<<10);
                                                          SbusVal[3]=(RxBuffer[5]>>1)|((uint16_t)RxBuffer[6]<<7);
                                                          SbusVal[4]=(RxBuffer[6]>>4)|((uint16_t)RxBuffer[7]<<4);
                                                          SbusVal[5]=(RxBuffer[7]>>7)|((uint16_t)RxBuffer[8]<<1)|((uint16_t)RxBuffer[9]<<9);
                                                          SbusVal[6]=(RxBuffer[9]>>2)|((uint16_t)RxBuffer[10]<<6);
                                                          SbusVal[7]=(RxBuffer[10]>>5)|((uint16_t)RxBuffer[11]<<3);

                                                          for(uint8_t i8=0;i8<8;i8++){
                                                            SbusVal[i8]&=0x7FF;
                                                            RxVal[i8+1]=(SbusVal[i8]*5)/2 +(6080-2560);
                                                          }
                                                          return 1;
                                                        }
                                                        else return 0;

}

uint8_t RxUpdateChannel (uint8_t *RxUpdated, uint8_t *payload,uint16_t *RxVal ){
	uint8_t ReVal=0;
	  if(*RxUpdated==2) {
		if(SBUS_Decoder((uint8_t*)payload,RxVal)){
			  ReVal=1;
	    }
	  }
	  if(*RxUpdated==4) {
		 decode_crsf_channels((uint8_t *)payload, RxVal);
	  }
	   *RxUpdated=0;
	   return ReVal;
}
