/*
 * lin_hal.c
 *
 *  Created on: Mar 17, 2025
 *      Author: can.yildirim
 */


#include "lin_hal.h"

static lin_driver_state_t LIN_driver_state = LIN_RECEIVE_PID;
static uint8_t received_frame_id;
static uint8_t received_frame_pid;
USART_Handle_t LIN_Handle_Config;

typedef enum
{
	USART_READY_FOR_RX = 0,
	USART_BUSY_FOR_RX
}Usart_State_t;

Usart_State_t Usart_State;


INLINE uint8_t LIN_Get_Flag_Status(uint32_t flag)
{
	uint8_t flag_status;
	if(USART2->SR & flag)
	{
		flag_status = SET;
	}
	else
	{
		flag_status = RESET;
	}
	return flag_status;
}

INLINE void LIN_Clear_Flag_Status(uint32_t flag)
{
	USART2->SR &= ~(flag);
}



INLINE void LIN_Get_Rx_Data(uint8_t *pRxBuffer)
{
	*pRxBuffer = (uint8_t)USART2->DR;

	//USART2->CR2 |= ~(1 << USART_CR1_RXNEIE);
}




INLINE void LIN_GoTo_Idle_State(void)
{
	//will be implemented
}






INLINE void LIN_Process_Data(uint8_t data)
{
	uint8_t temp_byte = data;

	switch(LIN_driver_state)
	{
		case LIN_RECEIVE_SYNC:
			if(temp_byte == LIN_SYNC_FIELD)
			{
				LIN_driver_state = LIN_RECEIVE_PID;
			}
			else
			{
				LIN_GoTo_Idle_State();
			}
			break;
		case LIN_RECEIVE_PID:
			received_frame_id = LIN_Process_PID(temp_byte, CHECK_PARITY_REQUEST);
			received_frame_pid = temp_byte;
			if(received_frame_id != 0xFF)
			{
				LIN_driver_state = LIN_RECEIVE_DATA;
			}
			else
			{
				//if received frame id is 0xFF, invalid frame is received

			}
			break;
		case LIN_RECEIVE_DATA:
			rx_buf[index++]

			break;

		default:
			break;
	}
}



void LIN_Init(void)
{
	LIN_Handle_Config.pUSARTx = USART2;
	LIN_Handle_Config.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	LIN_Handle_Config.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	LIN_Handle_Config.USART_Config.USART_Mode = USART_MODE_TXRX;
#if LIN_MODE == 0
	LIN_Handle_Config.USART_Config.USART_LINMode = USART_LIN_MODE_DISABLE;
#elif LIN_MODE == 1
	LIN_Handle_Config.USART_Config.USART_LINMode = USART_LIN_MODE_ENABLE;
#endif
	LIN_Handle_Config.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	LIN_Handle_Config.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	LIN_Handle_Config.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;


	USART_Init(&LIN_Handle_Config);
	USART_PeripheralControl(USART2, ENABLE);

}

void LIN_DeInit(void)
{
	USART_DeInit(USART2);
	USART_PeripheralControl(USART2, RESET);
	LIN_driver_state = LIN_UNINIT;
}

void LIN_ISR(void)
{
	uint8_t received_data;

	//An interrupt is generated when LBD=1 if LBDIE=1
	if(LIN_Get_Flag_Status(LIN_FLAG_LBD) != 0x00U)
	{
		LIN_Clear_Flag_Status(LIN_FLAG_LBD);

#if FEATURE_SLEEP_MODE == 1
		if(LIN_driver_state == LIN_SLEEP_MODE)
		{
		LIN_GoTo_Idle_State();
		}
#endif
		LIN_driver_state = LIN_RECEIVE_SYNC;
	}
	else if(LIN_Get_Flag_Status(LIN_FLAG_RXNE))
	{
		LIN_Clear_Flag_Status(LIN_FLAG_RXNE);
		LIN_Get_Rx_Data(&received_data);
		LIN_Process_Data(received_data);
	}





}




