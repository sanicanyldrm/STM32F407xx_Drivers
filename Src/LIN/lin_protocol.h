/*
 * lin_protocol.h
 *
 *  Created on: Mar 17, 2025
 *      Author: can.yildirim
 */

#include "stdint.h"

#ifndef LIN_LIN_PROTOCOL_H_
#define LIN_LIN_PROTOCOL_H_

#define CHECK_PARITY_REQUEST 				0
#define CALCULATE_PARITY_REQUEST			1


#define PARITY_BITS 						0xC0U
#define ID_BITS								0x3FU

/***************************************************************************
 * Global Type Definitions
 **************************************************************************/
typedef enum
{
	FRAME_IDLE = 0,
	FRAME_BREAK_RECEIVED = 1,
	FRAME_SYNC_RECEIVED = 2,
	FRAME_PID = 3,
	FRAME_RX_DATA = 4,
	FRAME_RX_CHECKSUM = 5,
	FRAME_TX_DATA = 6,
	FRAME_TX_CHECKSUM = 7,
}LinFrameStateType;


extern uint8_t LIN_Process_PID(uint8_t pid, uint8_t process_type);



/***************************************************************************
 * Global Functions Prototypes
 **************************************************************************/
void Lin_SetFrameState(LinFrameStateType CurrentFrameState);

void Lin_StateMachine(uint8_t ReceivedData);


/***************************************************************************
 * Static Functions Prototypes
 **************************************************************************/

LinFrameStateType Lin_GetFrameState(void);
#endif /* LIN_LIN_PROTOCOL_H_ */
