/*
 * Rx.h
 *
 *  Created on: Jun 24, 2026
 *      Author: HP
 */

#ifndef APPLICATION_USER_CORE_RX_H_
#define APPLICATION_USER_CORE_RX_H_
#include "main.h"
#define RX_CENTRE (6080)

typedef struct  {
	uint8_t CenterDone;
	uint16_t ChannelValue[16];
	uint16_t ChannelCenter[6];
	uint16_t ChannelValueLast[16];
	int16_t TxMove[4];
	int16_t TxMove2[4];
	float TxMoveDelay[4];
	uint8_t  UpdatedFlag;
	uint32_t UpdatedTick;
	uint32_t UpdatedCount;
	uint32_t Rate; //ms
}
Rx_t;

uint8_t SBUS_Decoder(uint8_t *RxBuffer,uint16_t *RxVal);
uint8_t RxUpdateChannel (uint8_t *RxUpdated, uint8_t *payload,uint16_t *RxVal );
#endif /* APPLICATION_USER_CORE_RX_H_ */
