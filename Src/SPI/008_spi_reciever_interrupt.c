/*
 * 008_spi_reciever_interrupt.c
 *
 *  Created on: Sep 23, 2024
 *      Author: sanic
 */

#include "stm32f407xx.h"
#include <string.h>
#include <stdio.h>

SPI_Handle_t SPI2handle;

#define MAX_LEN 		500

char RcvBuff[MAX_LEN];

volatile char ReadByte;

volatile uint8_t rcvStop = 0;

volatile uint8_t dataAvaliable;

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}


void Slave_GPIO_InterruptPinInit(void);
void SPI2_GPIOInits(void);
void SPI2_Inits(void);







int main(void)
{
	uint8_t dummy = 0xFF;

	Slave_GPIO_InterruptPinInit();

	SPI2_GPIOInits();

	SPI2_Inits();

	SPI_SSOEConfig(SPI2, ENABLE);

	SPI_IRQInterruptConfig(IRQ_NO_SPI_2, ENABLE);

	while(1)
	{
		rcvStop = 0;

		//wait untill data avaliable interrupt from transmitter device(slave)
		while(!dataAvaliable);

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, DISABLE);

		//enable SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		while(!rcvStop)
		{
			//fetch the data from the SPI peripheral byte by byte in iterrupt mode
			while( SPI_SendDataIT(&SPI2handle, &dummy, 1) == SPI_BUSY_IN_TX );
			while( SPI_ReceiveDataIT(&SPI2handle, &ReadByte, 1) == SPI_BUSY_IN_RX);
		}

		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		SPI_PeripheralControl(SPI2, DISABLE);

		printf("Rcvd data = %s\n", RcvBuff);

		dataAvaliable = 0;

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	}




	return 0;
}

//This function configures the GPIO pin over which SPI peripheral issues data avaliable interrupt
void Slave_GPIO_InterruptPinInit(void)
{
	GPIO_Handle_t SPIInterruptPin;
	memset(&SPIInterruptPin, 0, sizeof(SPIInterruptPin));

	SPIInterruptPin.pGPIOx = GPIOD;
	SPIInterruptPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	SPIInterruptPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	SPIInterruptPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	SPIInterruptPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&SPIInterruptPin);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

}

void SPI2_GPIOInits(void)
{

	GPIO_Handle_t SPI2Pins;

	SPI2Pins.pGPIOx = GPIOB;
	SPI2Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI2Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPI2Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI2Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI2Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;


	//SCLK
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPI2Pins);

	//MOSI
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPI2Pins);

	//MISO
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPI2Pins);

	//NSS
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPI2Pins);

}

void SPI2_Inits(void)
{


	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2handle);

}

void SPI2_IRQHandler(void)
{
	SPI_IRQHandling(&SPI2handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	static uint32_t i = 0;

	if(AppEv == SPI_EVENT_RX_CMPLT)
	{
		RcvBuff[i++] = ReadByte;
		if(ReadByte == '\0' || (i == MAX_LEN))
		{
			rcvStop = 1;
			RcvBuff[i-1] = '\0';
			i = 0;
		}
	}
}

void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_6);
	dataAvaliable = 1;
}

