/*
 * 007spi_cmd_handling.c
 *
 *  Created on: Sep 7, 2024
 *      Author: sanic
 *
 *
 * PB14 -> SPI2 MISO
 * PB15 -> SPI2 MOSI
 * PB13 -> SPI2 SCLK
 * PB12 -> SPI2 NSS
 * ALT function mode: 5
 */

#include <string.h>
#include "stm32f407xx.h"

/*
 * Macros Section
 * Command Codes
 */

#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

/*
 * Led commands
 */
#define LED_ON					1
#define LED_OFF					0

/*
 * Analog pins
 */

#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3
#define ANALOG_PIN4				4
#define ANALOG_PIN5				5

/*
 * Led pins
 */
#define LED_PIN					9
/*
 * GLOBAL VARIABLES SECTION
 */
int g_counter = 5000;


/*
 * Function prototypes
 */
void delay(void);
void SPI2_GPIOInits(void);
void SPI2_Inits(void);
void GPIO_ButtonInit(void);
uint8_t SPI_VerifyResponse(ackbyte);




int main(void)
{

	uint8_t dummy_write = 0xFF;

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


		//wait untill button pressed
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );

		//to avoid de-bouncing related issues 200ms of delay
		delay();

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);


		//1. CMD_LED_CTRL <pin_no> <value>

		uint8_t cmd_code = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];
		uint8_t dummy_read;

		//send command
		SPI_SendData(SPI2, &cmd_code, 1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);


		//send some dummy bits(1 bytes) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if ( SPI_VerifyResponse(ackbyte) )
		{
			//send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2, args, 2);
		}



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
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPI2Pins);

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


uint8_t SPI_VerifyResponse(ackbyte)
{
	if(ackbyte == 0xF5)
	{
		return 1;
	}
	return 0;
}

