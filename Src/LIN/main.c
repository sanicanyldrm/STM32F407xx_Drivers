/*
 * main.c
 *
 *  Created on: Mar 17, 2025
 *      Author: can.yildirim
 */
#include "stm32f407xx.h"


void GPIO_Usart_Pins_Init(void)
{
	GPIO_Handle_t USART2_Tx_GPIO_Config;
	GPIO_Handle_t USART2_Rx_GPIO_Config;

	USART2_Tx_GPIO_Config.pGPIOx = GPIOA;
	USART2_Tx_GPIO_Config.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	USART2_Tx_GPIO_Config.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USART2_Tx_GPIO_Config.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USART2_Tx_GPIO_Config.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	USART2_Tx_GPIO_Config.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	USART2_Tx_GPIO_Config.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

	USART2_Rx_GPIO_Config.pGPIOx = GPIOA;
	USART2_Rx_GPIO_Config.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	USART2_Rx_GPIO_Config.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USART2_Rx_GPIO_Config.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USART2_Rx_GPIO_Config.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	USART2_Rx_GPIO_Config.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	USART2_Rx_GPIO_Config.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

	GPIO_Init(&USART2_Tx_GPIO_Config);
	GPIO_Init(&USART2_Rx_GPIO_Config);

}


void System_Init(void)
{
	GPIO_Usart_Pins_Init();
}























int main(void)
{


	return 0;
}
