/*
 * usart_rx_interrupt.c
 *
 *  Created on: Mar 15, 2025
 *      Author: sanic
 */

#include "stm32_f407xx_USART_driver.h"
#include "string.h"

void USART2_GPIO_Init(void);
void USART2_Init(void);

USART_Handle_t usart2_handle;

#define SLAVE_NAD 	0x63
#define B0			0x22


typedef struct
{
	uint8_t PID;
	uint8_t NAD;
	uint8_t PCI;
	uint8_t SID;
	uint8_t D1;
	uint8_t D2;
	uint8_t D3;
	uint8_t D4;
	uint8_t D5;
}RxData;

typedef struct
{
	uint8_t PID;
	uint8_t NAD;
	uint8_t PCI;
	uint8_t SID;
	uint8_t D1;
	uint8_t D2;
	uint8_t D3;
	uint8_t D4;
	uint8_t D5;
}Diag_Response;

Diag_Response DiagResponseData;
RxData Master_RxData;

void sw_delay(void)
{
	uint32_t i;
	for(i = 0; i <500000/2; i++);
}


int main(void)
{

	USART2_GPIO_Init();
	USART2_Init();

	USART_IRQInterruptConfig(IRQ_NO_USART2,ENABLE);

	USART_PeripheralControl(USART2,ENABLE);

	while(1)
	{
		while ( USART_ReceiveDataIT(&usart2_handle,&Master_RxData,9) != USART_READY );


	}






	return 0;
}



void USART2_GPIO_Init(void)
{
	GPIO_Handle_t usart_gpios;
	GPIO_Handle_t GPIO_Led_Orange;

	usart_gpios.pGPIOx = GPIOA;
	usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode =7;

	usart_gpios.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NO_2;
	GPIO_Init(&usart_gpios);

	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart_gpios);

	GPIO_Led_Orange.pGPIOx = GPIOD;
	GPIO_Led_Orange.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	//alternate function is not necessary
	GPIO_Led_Orange.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_Led_Orange.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Led_Orange.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_Led_Orange.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIO_Led_Orange);

	GPIO_Handle_t GPIO_Led_Red;
	GPIO_Led_Red.pGPIOx = GPIOD;
	GPIO_Led_Red.GPIO_PinConfig.GPIO_PinNumber = 14;
	//alternate function is not necessary
	GPIO_Led_Red.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_Led_Red.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Led_Red.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_Led_Red.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIO_Led_Red);
}

void USART2_Init(void)
{
	usart2_handle.pUSARTx = USART2;
	usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_9600;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart2_handle);
}


void USART2_IRQHandler(void)
{
	USART_IRQHandling(&usart2_handle);
}

void USART_ApplicationEventCallback( USART_Handle_t *pUSARTHandle, uint8_t ApEv)
{
	uint8_t PID;
	uint8_t NAD;
	uint8_t PCI;
	uint8_t SID;
	uint8_t pData[5];
	if(ApEv == USART_EVENT_RX_CMPLT)
	{
		PID = Master_RxData.PID;
		NAD = Master_RxData.NAD;
		PCI = Master_RxData.PCI;

		pData[0] = Master_RxData.D1;
		pData[1] = Master_RxData.D2;
		pData[2] = Master_RxData.D3;
		pData[3] = Master_RxData.D4;
		pData[4] = Master_RxData.D5;

		if(PID = 0x3C)
		{
			if(NAD = SLAVE_NAD)
			{
				if(PCI = 0x06)
				{
					if(SID = B0)
					{
						GPIO_WriteToOutputPin(GPIOD, 13, SET);
						GPIO_WriteToOutputPin(GPIOD, 14, RESET);
						DiagResponseData.PID = 0x3C;
						DiagResponseData.NAD = SLAVE_NAD;
						DiagResponseData.PCI = 0x06;
						DiagResponseData.SID = B0;
						DiagResponseData.D1 = 0xAA;
						DiagResponseData.D2 = 0xAA;
						DiagResponseData.D3 = 0xAA;
						DiagResponseData.D4 = 0xAA;
						DiagResponseData.D5 = 0xAA;
						USART_SendData(&usart2_handle, &DiagResponseData, 9);
					}
					else
					{
						GPIO_WriteToOutputPin(GPIOD, 13, RESET);
						GPIO_WriteToOutputPin(GPIOD, 14, SET);
					}
				}
				else
				{
					//ignore message
				}
			}
			else
			{
				//ignore frame
			}
		}
		else
		{
			//handle normal frame
		}


	}
}



