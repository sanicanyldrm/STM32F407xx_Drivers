/*
 * 005_spi_rx_testing.c
 *
 *  Created on: Sep 4, 2024
 *      Author: sanic
 */


/*
 * Tx Configuration SPI2
 * PB12 --> SPI_2 NSS
 * PB13 --> SPI_2 SCLK
 * PB15 --> SPI_2 MOSI
 * PB14 --> SPI_2 MISO
 * ALT function mode : 5
 */

/*
 * Rx Configuration
 * PA4 --> SPI_1 NSS
 * PA5 --> SPI_1 SCK
 * PA6 --> SPI_1 MISO
 * PA7 --> SPI_1 MOSI
 */

#include "stm32f407xx.h"
#include <string.h>


int g_counter = 150000;

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

	GPIO_Handle_t SPI_Rx_Pins;
	SPI_Rx_Pins.pGPIOx = GPIOA;
	SPI_Rx_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI_Rx_Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPI_Rx_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // output type has to be PP
	SPI_Rx_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI_Rx_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;



	/**
	 * GPIO led green config structure
	*/
	GPIO_Handle_t GPIO_Led_Green;

	GPIO_Led_Green.pGPIOx = GPIOD;
	GPIO_Led_Green.GPIO_PinConfig.GPIO_PinNumber = 12;
	//alternate function is not necessary
	GPIO_Led_Green.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_Led_Green.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Led_Green.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_Led_Green.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIO_Led_Green);


	/*
	 * SPI2 Tx pins configurations
	 */

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	//NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	//GPIO_Init(&SPIPins);

	/*
	 * SPI1 Rx pins configurations
	 */
	SPI_Rx_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&SPI_Rx_Pins);


}


void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;
	SPI_Handle_t SPI1_handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV256;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN; // software slave management



	SPI2handle.pSPIx = SPI1;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_SLAVE;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV256;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;





	SPI_Init(&SPI2handle);
	SPI_Init(&SPI1_handle);


}




int main(void)
{




	return 0;

}
