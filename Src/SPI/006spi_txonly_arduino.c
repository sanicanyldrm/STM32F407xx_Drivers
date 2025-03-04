/*
 * 006spi_txonly_arduino.c
 *
 *  Created on: Sep 4, 2024
 *      Author: sanic
 */


#include <string.h>
#include "stm32f407xx.h"


/*
 * PB14 -> SPI2 MISO
 * PB15 -> SPI2 MOSI
 * PB13 -> SPI2 SCLK
 * PB12 -> SPI2 NSS
 * ALT function mode: 5
 */

/*
 * GLOBAL VARIABLES SECTION
 */
int g_counter = 5000;






void delay(void);
void SPI2_GPIOInits(void);
void SPI2_Inits(void);
void GPIO_ButtonInit(void);



int main(void)
{

	char data[] = "Askim Seni Cok Seviyorum";

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//this function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	//initialize the user button
	GPIO_ButtonInit();

	/*
	 * making SSOE1 does NSS output enable.
	 * the NSS pin is automatically managed by the hardware
	 * i.e when SPE = 1, NSS will be pulled to low
	 * and NSS pin will be high when SPE = 0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);






	//enable spi2 peripheral


	while(1)
	{



		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );

		delay();

		SPI_PeripheralControl(SPI2, ENABLE);



		//first send length information
		uint8_t data_length = strlen(data);

		SPI_SendData(SPI2, &data_length, 1);

		SPI_SendData(SPI2, (uint8_t*)data, strlen(data));

		while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );


		SPI_PeripheralControl(SPI2, DISABLE);





	}
}






void delay(void)
{
	for(uint32_t i = 0; i < g_counter; i++);
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
	//SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPI2Pins);

	//NSS
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPI2Pins);

}


void SPI2_Inits(void)
{
	SPI_Handle_t SPI2_Handle;

	SPI2_Handle.pSPIx = SPI2;
	SPI2_Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2_Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2_Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //SCLK is 2 Mhz
	SPI2_Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2_Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;//hardware slave management

	SPI_Init(&SPI2_Handle);

}

void GPIO_ButtonInit(void)
{

	GPIO_Handle_t GPIOUserButton;

	GPIOUserButton.pGPIOx = GPIOA;
	GPIOUserButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOUserButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOUserButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOUserButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOUserButton);
}






