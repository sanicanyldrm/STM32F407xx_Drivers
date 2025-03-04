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
int g_counter = 500000;


/*
 * Function prototypes
 */
void delay(void);
void SPI2_GPIOInits(void);
void SPI2_Inits(void);
void GPIO_ButtonInit(void);
uint8_t SPI_VerifyResponse(uint8_t ackbyte);
void Command_Control(uint8_t cmd_code);




int main(void)
{



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


		Command_Control(COMMAND_LED_CTRL);

		//wait untill button pressed
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );

		//to avoid de-bouncing related issues 200ms of delay
		delay();

		Command_Control(COMMAND_SENSOR_READ);

		//wait untill button pressed
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );

		//to avoid de-bouncing related issues 200ms of delay
		delay();

		Command_Control(COMMAND_LED_READ);

		//wait untill button pressed
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );

		//to avoid de-bouncing related issues 200ms of delay
		delay();

		Command_Control(COMMAND_PRINT);

		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );

				//to avoid de-bouncing related issues 200ms of delay
		delay();

		Command_Control(COMMAND_ID_READ);

		while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );


		SPI_PeripheralControl(SPI2, DISABLE);





	}
}


void delay(void)
{
	for(uint32_t i = 0; i < (g_counter/2); i++);
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


uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
	{
		return 1;
	}
	return 0;
}


void Command_Control(uint8_t cmd_code)
{



			uint8_t ackbyte = 0;
			uint8_t args[2] = {0, 0};
			uint8_t dummy_read = 0;
			uint8_t dummy_write = 0xFF;


			switch(cmd_code)
			{
			case COMMAND_LED_CTRL:

				//send command COMMAND_LED_CTRL
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
				break;

			case COMMAND_SENSOR_READ:
				//send command
				SPI_SendData(SPI2,&cmd_code,1);

				//do dummy read to clear off the RXNE
				SPI_ReceiveData(SPI2,&dummy_read,1);

				//Send some dummy byte to fetch the response from the slave
				SPI_SendData(SPI2,&dummy_write,1);

				//read the ack byte received
				SPI_ReceiveData(SPI2,&ackbyte,1);

				if( SPI_VerifyResponse(ackbyte))
				{
					args[0] = ANALOG_PIN0;

					//send arguments
					SPI_SendData(SPI2,args,1); //sending one byte of

					//do dummy read to clear off the RXNE
					SPI_ReceiveData(SPI2,&dummy_read,1);

					//insert some delay so that slave can ready with the data
					delay();

					//Send some dummy bits (1 byte) fetch the response from the slave
					SPI_SendData(SPI2,&dummy_write,1);

					uint8_t analog_read;
					SPI_ReceiveData(SPI2,&analog_read,1);

				}
				break;

			case COMMAND_LED_READ:
				//send command
				SPI_SendData(SPI2,&cmd_code,1);

				//do dummy read to clear off the RXNE
				SPI_ReceiveData(SPI2,&dummy_read,1);

				//Send some dummy byte to fetch the response from the slave
				SPI_SendData(SPI2,&dummy_write,1);

				//read the ack byte received
				SPI_ReceiveData(SPI2,&ackbyte,1);

				if( SPI_VerifyResponse(ackbyte))
				{
					args[0] = LED_PIN;

					//send arguments
					SPI_SendData(SPI2,args,1); //sending one byte of

					//do dummy read to clear off the RXNE
					SPI_ReceiveData(SPI2,&dummy_read,1);

					//insert some delay so that slave can ready with the data
					delay();

					//Send some dummy bits (1 byte) fetch the response from the slave
					SPI_SendData(SPI2,&dummy_write,1);

					uint8_t led_status;
					SPI_ReceiveData(SPI2,&led_status,1);

				}
				break;

			case COMMAND_PRINT:
				//send command
				SPI_SendData(SPI2,&cmd_code,1);

				//do dummy read to clear off the RXNE
				SPI_ReceiveData(SPI2,&dummy_read,1);

				//Send some dummy byte to fetch the response from the slave
				SPI_SendData(SPI2,&dummy_write,1);

				//read the ack byte received
				SPI_ReceiveData(SPI2,&ackbyte,1);

				uint8_t message[] = "Hello ! How are you ??";
				if( SPI_VerifyResponse(ackbyte))
				{
					args[0] = strlen((char*)message);

					//send arguments
					SPI_SendData(SPI2,args,1); //sending length

					//send message
					SPI_SendData(SPI2,message,args[0]);

				}
				break;
			case COMMAND_ID_READ:
				//send command
				SPI_SendData(SPI2,&cmd_code,1);

				//do dummy read to clear off the RXNE
				SPI_ReceiveData(SPI2,&dummy_read,1);

				//Send some dummy byte to fetch the response from the slave
				SPI_SendData(SPI2,&dummy_write,1);

				//read the ack byte received
				SPI_ReceiveData(SPI2,&ackbyte,1);

				uint8_t id[11];
				uint32_t i=0;
				if( SPI_VerifyResponse(ackbyte))
				{
					//read 10 bytes id from the slave
					for(  i = 0 ; i < 10 ; i++)
					{
						//send dummy byte to fetch data from slave
						SPI_SendData(SPI2,&dummy_write,1);
						SPI_ReceiveData(SPI2,&id[i],1);
					}

					id[11] = '\0';

				}
			}
}
