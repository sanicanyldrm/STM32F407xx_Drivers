/*
 * 002_led_toggle.c
 *
 *  Created on: Sep 4, 2024
 *      Author: sanic
 */


#include "stm32f407xx.h"

void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}


int main()
{

	GPIO_Handle_t GPIO_Led_Green;
	GPIO_Handle_t GPIO_Led_Orange;


	/**
	 * GPIO led green config structure
	 */
	GPIO_Led_Green.pGPIOx = GPIOD;
	GPIO_Led_Green.GPIO_PinConfig.GPIO_PinNumber = 12;
	//alternate function is not necessary
	GPIO_Led_Green.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_Led_Green.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Led_Green.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GPIO_Led_Green.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	/*
	 * GPIO led orange config structure
	 */
	GPIO_Led_Orange.pGPIOx = GPIOD;
	GPIO_Led_Orange.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	//alternate function is not necessary
	GPIO_Led_Orange.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_Led_Orange.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Led_Orange.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GPIO_Led_Orange.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;


	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GPIO_Led_Orange);
	GPIO_Init(&GPIO_Led_Green);



	while(1)
	{
		GPIO_ToogleOutputPin(GPIOD, 12);
		delay();
		GPIO_ToogleOutputPin(GPIOD, 13);
		delay();
	}
}
