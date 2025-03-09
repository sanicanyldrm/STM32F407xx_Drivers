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
USART_Handle_t USART2_Handle;

//Data
char msg[1024] = "Ajkummmm Seni Cok Seviyorum\n\r";
void USART2_GPIO_Init();
void USART2_Init();


void sw_delay(void)
{
	uint32_t i;
	for(i = 0; i <500000/2; i++);
}
int main()
{


	USART2_GPIO_Init();


	USART2_Init();


	USART_PeripheralControl(USART2, ENABLE);

	while(1)
	{
		USART_SendData(&USART2_Handle, msg, strlen(msg));
		sw_delay();
	}


	return 0;
}

void USART2_GPIO_Init(void)
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


void USART2_Init()
{
	USART2_Handle.pUSARTx = USART2;
	USART2_Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USART2_Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART2_Handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	USART2_Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART2_Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART2_Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&USART2_Handle);
}
