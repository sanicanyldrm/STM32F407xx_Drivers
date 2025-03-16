/*
 * uart_tx.c
 *
 *  Created on: Mar 8, 2025
 *      Author: sanic
 */
#define LIN_USART_MODE 			0



//PA9 	TX
//PA10 	RX
#include "stm32_f407xx_USART_driver.h"
#include "string.h"


#if LIN_USART_MODE == 0

//USART1 Handle structure
USART_Handle_t USART2_Handle;

//Data
char msg[1024] = "Ajkummmm Seni Cok Seviyorum\n\r";

#endif


#if LIN_USART_MODE == 1

USART_Handle_t USART_LIN_Handle;

uint8_t TxData[20];

uint8_t PID_Calculate(uint8_t ID);
uint8_t Checksum_Calculate(uint8_t PID, uint8_t *pData, uint8_t size);


#endif


void USART2_GPIO_Init(void);
void USART2_Init(void);


void sw_delay(void)
{
	uint32_t i;
	for(i = 0; i <500000/2; i++);
}
int main()
{

#if LIN_USART_MODE == 0
	USART2_GPIO_Init();


	USART2_Init();


	USART_PeripheralControl(USART2, ENABLE);

	while(1)
	{
		USART_SendData(&USART2_Handle, msg, strlen(msg));
		sw_delay();
	}

#endif


#if LIN_USART_MODE == 1

	USART2_GPIO_Init();

	USART2_Init();

	USART_PeripheralControl(USART2, ENABLE);

	TxData[0] = 0x55; //sync field
	TxData[1] = PID_Calculate(0x34);

	for(int i= 0; i < 8; i++)
	{
		TxData[i + 2] = i;
	}

	TxData[10] = Checksum_Calculate(TxData[1], TxData+2, 8);

	while(1)
	{
		USART_SendData(&USART_LIN_Handle, TxData, 10);
		sw_delay();
	}


#endif


	return 0;
}

void USART2_GPIO_Init(void)
{

#if LIN_USART_MODE == 0

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

#endif

#if LIN_USART_MODE == 1

	GPIO_Handle_t USART_LIN_GPIO_Handle;

	USART_LIN_GPIO_Handle.pGPIOx = GPIOA;
	USART_LIN_GPIO_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USART_LIN_GPIO_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USART_LIN_GPIO_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	USART_LIN_GPIO_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	USART_LIN_GPIO_Handle.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

	USART_LIN_GPIO_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&USART_LIN_GPIO_Handle);

	USART_LIN_GPIO_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&USART_LIN_GPIO_Handle);


#endif

}


void USART2_Init(void)
{
#if LIN_USART_MODE == 0
	USART2_Handle.pUSARTx = USART2;
	USART2_Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USART2_Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART2_Handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	USART2_Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART2_Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART2_Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&USART2_Handle);
#endif

#if LIN_USART_MODE == 1

	USART_LIN_Handle.pUSARTx = USART2;
	USART_LIN_Handle.USART_Config.USART_Baud = USART_STD_BAUD_9600;
	USART_LIN_Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART_LIN_Handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	USART_LIN_Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART_LIN_Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART_LIN_Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&USART_LIN_Handle);

#endif


}


#if LIN_USART_MODE == 1
uint8_t PID_Calculate(uint8_t ID)
{
	uint8_t ID_Buf[6];
	uint8_t P0;
	uint8_t P1;

	if(ID > 0x3F)
	{
		//do nothing
	}
	else
	{
		for(int i = 0; i < 6; i++)
		{
			ID_Buf[i] = ((ID >> 1) & (uint8_t)0x01);
		}

	P0 = ( (ID_Buf[0] ^ ID_Buf[1] ^ ID_Buf[2] ^ ID_Buf[4]) & (uint8_t)0x01 );
	P1 = ~( (ID_Buf[1] ^ ID_Buf[3] ^ ID_Buf[4] ^ ID_Buf[5]) & (uint8_t)0x01 );

	ID = ID | ( P0 << 6 ) | (P1 << 7 );
	}

	return ID;
}



uint8_t Checksum_Calculate(uint8_t PID, uint8_t *pData, uint8_t size)
{
	uint8_t buffer[size + 2];
	uint16_t sum = 0;

	buffer[0] = PID;

	for(int i = 0; i < size; i++)
	{
		buffer[i + 1] = pData[i];
	}

	for(int i = 0; i < size + 1; i++)
	{
		sum = sum + buffer[i];

		if(sum > 0xFF)
		{
			sum = sum - 0xFF;
		}
	}

	sum = 0xFF - sum;

	return sum;


}

#endif
