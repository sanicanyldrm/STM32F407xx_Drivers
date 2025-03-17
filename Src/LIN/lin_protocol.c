/*
 * lin_protocol.c
 *
 *  Created on: Mar 17, 2025
 *      Author: can.yildirim
 */
#include "lin_protocol.h"




uint8_t LIN_Process_PID(uint8_t pid, uint8_t process_type)
{
	uint8_t parity;
	uint8_t ret_val;

	parity = CHECK_PARITY(pid);

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
