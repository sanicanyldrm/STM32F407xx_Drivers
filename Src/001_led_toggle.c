/*
 * 001_led_toggle.c
 *
 *  Created on: Sep 4, 2024
 *      Author: sanic
 */


#include "stm32f407xx.h"

int g_counter = 150000;

void delay(void)
{
	for(uint32_t i = 0; i < g_counter; i++);
}





int main()
{

	GPIO_Handle_t GPIO_Led_Orange;
	GPIO_Handle_t GPIO_Led_Green;
	GPIO_Handle_t GPIO_Led_Red;
	GPIO_Handle_t GPIO_Led_Blue;


	/**
	 * GPIO led green config structure
	 */
	GPIO_Led_Green.pGPIOx = GPIOD;
	GPIO_Led_Green.GPIO_PinConfig.GPIO_PinNumber = 12;
	//alternate function is not necessary
	GPIO_Led_Green.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_Led_Green.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Led_Green.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_Led_Green.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	/*
	 * GPIO led orange config structure
	 */
	GPIO_Led_Orange.pGPIOx = GPIOD;
	GPIO_Led_Orange.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	//alternate function is not necessary
	GPIO_Led_Orange.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_Led_Orange.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Led_Orange.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_Led_Orange.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	/*
	 * GPIO led red config structure
	 */
	GPIO_Led_Red.pGPIOx = GPIOD;
	GPIO_Led_Red.GPIO_PinConfig.GPIO_PinNumber = 14;
	//alternate function is not necessary
	GPIO_Led_Red.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_Led_Red.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Led_Red.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_Led_Red.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	/*
	 * GPIO led blue config structure
	 */
	GPIO_Led_Blue.pGPIOx = GPIOD;
	GPIO_Led_Blue.GPIO_PinConfig.GPIO_PinNumber = 15;
	//alternate function is not necessary
	GPIO_Led_Blue.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_Led_Blue.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Led_Blue.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_Led_Blue.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GPIO_Led_Orange);
	GPIO_Init(&GPIO_Led_Green);
	GPIO_Init(&GPIO_Led_Red);
	GPIO_Init(&GPIO_Led_Blue);

	while(1)
	{
		GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_13, SET);
		delay();
		GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_13, RESET);
		GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, SET);
		delay();
		GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, RESET);
		GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_15, SET);
		delay();
		GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_15, RESET);
		GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_14, SET);
		delay();
		GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_14, RESET);


		g_counter = g_counter - 5000;

		if(g_counter <= 0)
		{
			g_counter = 500000;
		}


	}


}
