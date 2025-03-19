/*
 * lin_protocol.c
 *
 *  Created on: Mar 17, 2025
 *      Author: can.yildirim
 */

#include "lin_protocol.h"

static inline uint8_t Check_Parity(uint8_t pid);


uint8_t LIN_Process_PID(uint8_t pid, uint8_t process_type)
{
	uint8_t parity;
	uint8_t ret_val;

	parity = Check_Parity(pid);

	if(process_type == CHECK_PARITY_REQUEST)
	{
		if( (pid & PARITY_BITS) != parity)
		{
			ret_val = 0xFF;
		}
		else
		{
			ret_val = (uint8_t)(pid & ID_BITS);
		}
	}
	else if(CALCULATE_PARITY_REQUEST)
	{
		ret_val = (uint8_t)(pid | parity);
	}
	else
	{
		//do nothing
	}

	return ret_val;
}


static inline uint8_t Check_Parity(uint8_t pid)
{
	uint8_t ID_Buf[6];
	uint8_t P0;
	uint8_t P1;
	uint8_t parity;


	for(int i = 0; i < 6; i++)
	{
		ID_Buf[i] = ((pid >> i) & (uint8_t)0x01);
	}

	P0 = ( (ID_Buf[0] ^ ID_Buf[1] ^ ID_Buf[2] ^ ID_Buf[4]) & (uint8_t)0x01 );
	P1 = ~( (ID_Buf[1] ^ ID_Buf[3] ^ ID_Buf[4] ^ ID_Buf[5]) & (uint8_t)0x01 );

	parity =( P0 << 6 ) | (P1 << 7 );


	return parity;
}
