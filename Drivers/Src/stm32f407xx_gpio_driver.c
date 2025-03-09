/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Sep 4, 2024
 *      Author: sanic
 */

#include "stm32f407xx_gpio_driver.h"




/******************************************************************************
 * @fn				- GPIO_Init
 *
 * @brief			- GPIO Initialization function
 *
 * @param[in]		- GPIO_Handle_t *pGPIOHandle
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

	uint32_t temp = 0; //temp register


	//enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. configure the mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//non-interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing the related bit fields
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}
	else
	{
		//interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. configure the falling trigger selection register
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding rising trigger selection register
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. configure the rising trigger selection register
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding falling trigger selection register
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. configure both falling trigger selection register and rising trigger selection register
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		//3. enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}

	temp = 0;
	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing the related bit fields
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;
	//3. configure the pupd setting
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing the related bit fields
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;
	//4. configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing the related bit fields
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//5. configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alternate function registers
		uint8_t temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) / 8;
		uint8_t temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 8;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~( 0xF<< (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2);

	}

}

/******************************************************************************
 * @fn				- GPIO_DeInit
 *
 * @brief			- GPIO De-Initialization function
 *
 * @param[in]		- GPIO_RegDef_t *pGPIOx
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else
	{
		GPIOI_REG_RESET();
	}
}


/******************************************************************************
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- GPIO Peripheral clock control function
 *
 * @param[in]		- GPIO_RegDef_t *pGPIOx
 * @param[in]		- uint8_t EnorDi
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else
		{
			GPIOI_PCLK_DI();
		}

	}
}



/******************************************************************************
 * @fn				- GPIO_ReadFromInputPin
 *
 * @brief			- GPIO Read from input pin function
 *
 * @param[in]		- GPIO_RegDef_t *pGPIOx
 * @param[in]		- uint8_t PinNumber
 *
 * @return			- uint8_t Value
 *
 * @notes			-
 *****************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t Value;
	Value = (uint8_t)( (pGPIOx->IDR >> PinNumber) &  0x00000001 );

	return Value;
}



/******************************************************************************
 * @fn				- GPIO_ReadFromInputPort
 *
 * @brief			- GPIO Read from input port function
 *
 * @param[in]		- GPIO_RegDef_t *pGPIOx
 *
 * @return			- uint16_t Value
 *
 * @notes			-
 *****************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{

	uint16_t Value;
	Value = (uint16_t)(pGPIOx->IDR);

	return Value;
}


/******************************************************************************
 * @fn				- GPIO_WriteToOutputPin
 *
 * @brief			- GPIO Write to output pin function
 *
 * @param[in]		- GPIO_RegDef_t *pGPIOx
 * @param[in]		- uint8_t PinNumber
 * @param[in]		- uint8_t Value
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}




/******************************************************************************
 * @fn				- GPIO_WriteToOutputPort
 *
 * @brief			- GPIO Write to output port function
 *
 * @param[in]		- GPIO_RegDef_t *pGPIOx
 * @param[in]		- uint16_t Value
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR |= Value;
}



/******************************************************************************
 * @fn				- GPIO_ToogleOutputPin
 *
 * @brief			- GPIO Toggle output pin function
 *
 * @param[in]		- GPIO_RegDef_t *pGPIOx
 * @param[in]		- uint8_t PinNumber
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void GPIO_ToogleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);//toggle the pin
}




/******************************************************************************
 * @fn				- GPIO_IRQInterruptConfig
 *
 * @brief			- GPIO IRQ Interrupt configuration function
 *
 * @param[in]		- uint8_t IRQNumber
 * @param[in]		- uint8_t EnorDi
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	 if(EnorDi == ENABLE)
	 {
		 if(IRQNumber <= 31)
		 {
			 //program ISER0 register
			 *NVIC_ISER0 |= (1 << IRQNumber);
		 }
		 else if(IRQNumber > 31 && IRQNumber < 64)
		 {
			 //program ISER1 register
			 *NVIC_ISER1 |= (1 << (IRQNumber % 32) );

		 }
		 else if(IRQNumber >= 64 && IRQNumber < 96)
		 {
			 //program ISER2 register
			 *NVIC_ISER3 |= (1 << (IRQNumber % 64) );
		 }
	 }
	 else
	 {
		 if(IRQNumber <= 31)
		 {
			 //program ICER0 register
			 *NVIC_ICER0 |= (1 << IRQNumber );
		 }
		 else if(IRQNumber > 31 && IRQNumber < 64)
		 {
			 //program ICER1 register
			 *NVIC_ICER1 |= (1 << (IRQNumber % 32) );
		 }
		 else if(IRQNumber >= 64 && IRQNumber < 96)
		 {
			 //program ICER2 register
			 *NVIC_ICER3 |= (1 << (IRQNumber % 64) );
		 }

	 }
}

/******************************************************************************
 * @fn				- GPIO_IRQPriorityConfig
 *
 * @brief			- GPIO IRQ Priority configuration function
 *
 * @param[in]		- uint8_t IRQPriority

 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

	//1. find out ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED );
	*( NVIC_PR_BASE_ADDR + iprx ) |= ( IRQPriority << shift_amount );

}



/******************************************************************************
 * @fn				- GPIO_IRQHandling
 *
 * @brief			- GPIO IRQ Handling function
 *
 * @param[in]		- uint8_t PinNumber

 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{

	//clear the EXTI Pending register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber) )
	{
		//clear the pending bit
		EXTI->PR &= (1 << PinNumber);
	}
}
