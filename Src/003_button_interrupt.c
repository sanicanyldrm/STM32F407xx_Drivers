/*
 * 003_button_interrupt.c
 *
 *  Created on: Sep 4, 2024
 *      Author: sanic
 */

#include "stm32f407xx.h"
#include <string.h>
#include <stdint.h>

void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}



int main()
{
	GPIO_Handle_t GPIO_Led_Green, GPIO_User_Button, GPIO_Led_Orange;

	memset(&GPIO_Led_Green, 0, sizeof(GPIO_Led_Green));
	memset(&GPIO_User_Button, 0, sizeof(GPIO_User_Button));
	memset(&GPIO_Led_Orange, 0, sizeof(GPIO_Led_Orange));

	GPIO_Led_Green.pGPIOx = GPIOD;
	GPIO_Led_Green.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Led_Green.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_Led_Green.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Led_Green.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_Led_Green.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_User_Button.pGPIOx = GPIOA;
	GPIO_User_Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_User_Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GPIO_User_Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_User_Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Led_Orange.pGPIOx = GPIOD;
	GPIO_Led_Orange.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Led_Orange.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_Led_Orange.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GPIO_Led_Orange.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_Led_Green.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;



	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GPIO_Led_Green);
	GPIO_Init(&GPIO_User_Button);
	GPIO_Init(&GPIO_Led_Orange);


	//IRQ configurations
	//1. enable irq
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

	while(1)
	{
		GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_13, SET);
		delay();
		GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_13, RESET);
		delay();
	}



	return 0;
}

void EXTI0_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_ToogleOutputPin(GPIOD, GPIO_PIN_NO_12);
	delay();
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_13, RESET);


}

