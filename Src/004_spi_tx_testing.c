/*
 * 004_spi_tx_testing.c
 *
 *  Created on: Sep 4, 2024
 *      Author: sanic
 */


/*
 *PB9 --> SPI_2 NSS
 *PB13 --> SPI_2 SCLK
 *PB15 --> SPI_2 MOSI
 *PB14 --> SPI_2 MISO
 *ALT function mode : 5
 */

#include "stm32f407xx.h"
#include <string.h>

int g_counter = 5000;

void delay(void)
{
	for(uint32_t i = 0; i < g_counter; i++);
}



void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // output type has to be PP
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;



	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&SPIPins);



}


void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV256;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_16BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // software slave management


	SPI_Init(&SPI2handle);
}




int main(void)
{

	char user_data[] = "A";
	uint16_t user_data_2 = 16000;
	uint8_t len = strlen(user_data);




	SPI2_GPIOInits();

	SPI2_Inits();

	//this makes NSS signal internally high and avoid MODF error
	SPI_SSIConfig(SPI2, DISABLE);

	//enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);


	while(1)
	{
		//SPI_SendData(SPI2, (uint8_t*)user_data, len);
		SPI_SendData(SPI2, &user_data_2, 1);

		delay();

	}


	return 0;

}
