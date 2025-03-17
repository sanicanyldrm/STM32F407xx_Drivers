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
#define CHECK_PARITY(A) 					( ( ((A >> 0) & 0x01) ^ ((A >> 1) & 0x01) ^ ((A >> 2) & 0x01) ^ ((A >> 4) & 0x01) << 6 ) | \
											  ~( ((A >> 1) & 0x01) ^ ((A >> 3) & 0x01) ^ ((A >> 4) & 0x01) ^ ((A >> 5) & 0x01) << 7 ) )
#define PARITY_BITS 						0xC0U
#define ID_BITS							0x3FU

typedef enum
{
	LIN_UNINIT = 0,
	LIN_IDLE,
	LIN_SEND_BREAK,
	LIN_SEND_SYN,
	LIN_SEND_PID,
	LIN_RECEIVE_SYNC,
	LIN_RECEIVE_PID,
	LIN_IGNORE_DATA,
	LIN_RECEIVE_DATA,
	LIN_SEND_DATA,
	LIN_SEND_DATA_COMPLETED,
	LIN_SLEEP_MODE
}lin_driver_state_t;



extern uint8_t LIN_Process_PID(uint8_t pid, uint8_t process_type);

#endif /* LIN_LIN_PROTOCOL_H_ */
