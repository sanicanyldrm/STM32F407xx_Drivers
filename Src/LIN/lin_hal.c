/*
 * lin_hal.c
 *
 *  Created on: Mar 17, 2025
 *      Author: can.yildirim
 */


#include "lin_hal.h"
#include "lin_protocol.h"
#include "lin_config.h"
#include "stm32f407xx.h"
#include "stm32_f407xx_USART_driver.h"
#include "stm32f407xx_gpio_driver.h"



USART_Handle_t LIN_Handle_Config;

uint8_t RxData[11];
uint8_t frame_index = 0;
uint8_t frame_count = 0;




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
}



INLINE void LIN_GoTo_Idle_State(void)
{
	//will be implemented
}

INLINE void LIN_USART_Set_Break(void)
{

	static uint8_t Data;

	while(!LIN_Get_Flag_Status(USART_FLAG_RXNE));
	Data = LIN_USART_BUFFER;

	//GPIO_WriteToOutputPin(GPIOD, 12, SET);
	//GPIO_WriteToOutputPin(GPIOD, 12, RESET);
	//Data = LIN_USART_BUFFER;
	uint8_t Data_2;


	USART2->CR2 &= ~(1 << USART_CR2_LBDIE);
	LIN_Clear_Flag_Status(LIN_FLAG_LBD);
	Lin_SetFrameState(FRAME_BREAK_RECEIVED);

	if(Lin_GetFrameState() == FRAME_BREAK_RECEIVED)
	{

		while(!LIN_Get_Flag_Status(USART_FLAG_RXNE))
		{
			GPIO_WriteToOutputPin(GPIOD, 12, SET);
		}
		GPIO_WriteToOutputPin(GPIOD, 12, RESET);

		Data_2 = LIN_USART_BUFFER;
		RxData[frame_index] = Data_2;
		frame_index++;

		if(Data_2 == LIN_SYNC_FIELD)
		{
			Lin_SetFrameState(FRAME_SYNC_RECEIVED);
			Lin_StateMachine(Data_2);
		}
	}
	USART2->CR1 |= (1 << USART_CR1_RXNEIE);


}


void LIN_ISR(void)
{

	uint32_t temp1 = USART2->CR2 & (1 << USART_CR2_LBDIE);
	uint32_t temp2 = USART2->CR1 & ( 1 << USART_CR1_RXNEIE);
	//An interrupt is generated when LBD=1 if LBDIE=1
	if(LIN_Get_Flag_Status(LIN_FLAG_LBD) != 0x00U && temp1)
	{
		LIN_USART_Set_Break();
	}
	else if(LIN_Get_Flag_Status(LIN_FLAG_RXNE) && temp2)
	{

		RxData[frame_index] = USART2->DR;
		frame_index++;

		if(frame_index > 10)
		{
			USART2->CR1 &= ~(1 << USART_CR1_RXNEIE);
			//USART2->CR2 |= (1 << USART_CR2_LINEN);
			USART2->CR2 |= (1 << USART_CR2_LBDIE);
			Lin_SetFrameState(FRAME_IDLE);
			frame_index = 0;

		}

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
	LIN_Handle_Config.USART_Config.USART_LINModeBreakDetectionLength = USART_LIN_MODE_BRK_DETECTION_11;
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

}



