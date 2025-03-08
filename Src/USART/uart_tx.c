/*
 * uart_tx.c
 *
 *  Created on: Mar 8, 2025
 *      Author: sanic
 */


//PA9 	TX
//PA10 	RX
#include "stm32_f407xx_USART_driver.h"
#include "string.h"

//USART1 Handle structure
USART_Handle_t USART1_Handle;

//Data
uint8_t Data[1024];
void USART1_GPIO_Init();
void USART1_Init();


void sw_delay(void)
{
	uint32_t i;
	for(i = 0; i <500000/2; i++);
}
int main()
{

	Data[0] = 0x3C;
	Data[1] = 0xA5;
	Data[2] = 0x5A;
	Data[3] = 0xA5;
	Data[4] = 0x5A;
	Data[5] = 0xA5;
	Data[6] = 0x5A;
	Data[7] = 0xFF;

	USART1_GPIO_Init();


	USART1_Init();


	USART_PeripheralControl(USART2, ENABLE);

	while(1)
	{
		USART_SendData(&USART1_Handle, Data, 8);
		sw_delay();
	}


	return 0;
}

void USART1_GPIO_Init(void)
{
	GPIO_Handle_t USART1IO_Handle;
	USART1IO_Handle.pGPIOx = GPIOA;
	USART1IO_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USART1IO_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USART1IO_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	USART1IO_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	USART1IO_Handle.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

	USART1IO_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&USART1IO_Handle);

	USART1IO_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&USART1IO_Handle);
}


void USART1_Init()
{
	USART1_Handle.pUSARTx = USART2;
	USART1_Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USART1_Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART1_Handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	USART1_Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART1_Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART1_Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&USART1_Handle);
}
