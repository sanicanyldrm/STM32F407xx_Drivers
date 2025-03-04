/*
 * stm32f407_spi_driver.c
 *
 *  Created on: Sep 4, 2024
 *      Author: sanic
 */


#include "stm32f407xx_spi_driver.h"
/*
 * Helper Function Prototypes for Interrupt Handling Operation
 */
static void SPI_TXE_Interrupt_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_Interrupt_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_OVR_ERR_Interrupt_Handle(SPI_Handle_t *pSPIHandle);


/******************************************************************************
 * @fn				- SPI_PeripheralControl
 *
 * @brief			- SPI peripheral control function
 *
 * @param[in]		- SPI_RegDef_t *pSPIx
 * @param[in]		- uint8_t EnorDi
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}

}

/******************************************************************************
 * @fn				- SPI_SSIConfig
 *
 * @brief			- SPI peripheral SSI bit control function
 *
 * @param[in]		- SPI_RegDef_t *pSPIx
 * @param[in]		- uint8_t EnorDi
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}

}

/******************************************************************************
 * @fn				- SPI_SSOEConfig
 *
 * @brief			- SPI peripheral SSOE bit control function
 *
 * @param[in]		- SPI_RegDef_t *pSPIx
 * @param[in]		- uint8_t EnorDi
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}

}







/******************************************************************************
 * @fn				- SPI_PeriClockControl
 *
 * @brief			- SPI peripheral clock control function
 *
 * @param[in]		- SPI_RegDef_t *pSPIx
 * @param[in]		- uint8_t EnorDi
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else
		{
			SPI4_PLCK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else
		{
			SPI4_PLCK_DI();
		}
	}


}






/******************************************************************************
 * @fn				- SPI_Init
 *
 * @brief			- SPI Initialization function
 *
 * @param[in]		- SPI_Handle_t *pSPIHandle
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{


	//first lets configure the SPI1_CR register
	uint32_t tempreg = 0;

	//peripheral clock control
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//1. configure the device mode
	tempreg |= ( pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR );

	//2. configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDI mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}
	else
	{
		//do nothing
	}

	//3. configure the SPI serial clock speed
	tempreg |= ( pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	//4. configure the DFF
	tempreg |= ( pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	//5. configure the CPOL
	tempreg |= ( pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	//6. configure CPHA
	tempreg |= ( pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	//7. configure SSM
	tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);


	pSPIHandle->pSPIx->CR1 = tempreg;

}



/******************************************************************************
 * @fn				- SPI_DeInit
 *
 * @brief			- SPI De-Initialization function
 *
 * @param[in]		- SPI_RegDef_t *pSPIx
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	//to do
}



uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{


	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;

}

/******************************************************************************
 * @fn				- SPI_SendData
 *
 * @brief			- SPI Sending data function
 *
 * @param[in]		- SPI_RegDef_t *pSPIx
 * @param[in]		- uint8_t *pTxBuffer
 * @param[in]		- uint32_t Len
 *
 * @return			- void
 *
 * @notes			- This is blocking call
 *****************************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{


	while(Len > 0)
	{
		//1. wait until TXE i set
		while( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET );

		//2. check the DFF bit in CR1
		if( pSPIx->CR1 & (1 << SPI_CR1_DFF) )
		{
			//data format is 16 bit
			//1. load the data in to DR
			pSPIx->DR = *(uint16_t*)pTxBuffer;

			pTxBuffer += sizeof(uint16_t);//increment the address of pTxBuffer
			Len--;


		}
		else
		{
			//data format is 8 bit
			//1. load the data in to DR
			pSPIx->DR = *pTxBuffer;

			pTxBuffer += sizeof(uint8_t);//increment the address of pTxBuffer
			Len--;//decrement the length variable


		}

	}

	while(SPI_GetFlagStatus(pSPIx, SPI_BUSY_FLAG) == FLAG_SET);

}


/******************************************************************************
 * @fn				- SPI_ReceiveData
 *
 * @brief			- SPI Receiving data function
 *
 * @param[in]		- SPI_RegDef_t *pSPIx
 * @param[in]		- uint8_t *pTxBuffer
 * @param[in]		- uint32_t Len
 *
 * @return			- void
 *
 * @notes			- This is blocking call
 *****************************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
		{
			//1. wait until RXE i set
			while( SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET );//when data recieved rx buffer set

			//2. check the DFF bit in CR1
			if( pSPIx->CR1 & (1 << SPI_CR1_DFF) )
			{
				//data format is 16 bit
				//1. load the data from DR to Rx buffer address
				*((uint16_t*)pRxBuffer) = pSPIx->DR;
				pRxBuffer += sizeof(uint16_t);
				Len--;//decrement the length variable
				//increment the address of pRxBuffer


			}
			else
			{
				//data format is 8 bit
				//1. load the data in to DR
				*(pRxBuffer) = pSPIx->DR;
				pRxBuffer += sizeof(uint8_t);//increment the address of pRxBuffer
				Len--;//decrement the length variable

			}

		}



}


/******************************************************************************
 * @fn				- SPI_IRQInterruptConfig
 *
 * @brief			- SPI IRQ Interrupt configuration function
 *
 * @param[in]		- uint8_t IRQNumber
 * @param[in]		- uint8_t EnorDi
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn				- SPI_IRQPriorityConfig
 *
 * @brief			- SPI IRQ Priority configuration function
 *
 * @param[in]		- uint8_t IRQPriority

 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

	//1. find out ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED );
	*( NVIC_PR_BASE_ADDR + iprx ) |= ( IRQPriority << shift_amount );

}

/******************************************************************************
 * @fn				- SPI_SendDataIT
 *
 * @brief			- SPI Send Data with Interrupt Mode Function
 *
 * @param[in]		- SPI_Handle_t *pSPIHandle
 * @param[in]		- uint8_t *pTxBuffer
 * @param[in]		- uint32_t Len
 *
 * @return			- uint8_t state
 *
 * @notes			-
 *****************************************************************************/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. Mark the SPI state as busy in transmission so that
		//no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is SET in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE);
	}

	//4. Data transmission will be handled by the ISR code

	return state;
}

/******************************************************************************
 * @fn				- SPI_ReceiveDataIT
 *
 * @brief			- SPI Receive Data with Interrupt Mode Function
 *
 * @param[in]		- SPI_Handle_t *pSPIHandle
 * @param[in]		- uint8_t *pTxBuffer
 * @param[in]		- uint32_t Len
 *
 * @return			- uint8_t state
 *
 * @notes			-
 *****************************************************************************/
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark the SPI state as busy in transmission so that
		//no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is SET in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE);
	}

	//4. Data transmission will be handled by the ISR code

	return state;

}


/******************************************************************************
 * @fn				- SPI_IRQHandling
 *
 * @brief			- SPI IRQ Handling Function
 *
 * @param[in]		- SPI_Handle_t *pSPIHandle
 *
 * @return			-
 *
 * @notes			-
 *****************************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t Temp_1_Flag = 0; 		/*to store TXE status in local variable*/
	uint8_t Temp_2_Flag = 0;		/*to store TXEIE status in local variable*/

	//Check TXE flag and write on local TXE_Flag variable
	Temp_1_Flag = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);

	//Check TXEIE flag and write on local TXEIE_Flag variable
	Temp_2_Flag = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if( Temp_1_Flag && Temp_2_Flag)
	{
		//handle TXE
		SPI_TXE_Interrupt_Handle(pHandle);

	}

	//Check RXNE flag and write on local TXE_Flag variable
	Temp_1_Flag = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);

	//Check RXNEIE flag and write on local TXEIE_Flag variable
	Temp_2_Flag = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if( Temp_1_Flag && Temp_2_Flag)
	{
		//handle RXNE
		SPI_RXNE_Interrupt_Handle(pHandle);

	}

	//Check OVR flag and write on local TXE_Flag variable
	Temp_1_Flag = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);

	//Check ERRIE flag and write on local TXEIE_Flag variable
	Temp_2_Flag = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if( Temp_1_Flag && Temp_2_Flag)
	{
		//handle RXNE
		SPI_OVR_ERR_Interrupt_Handle(pHandle);

	}

}


/*
 * Helper Function Prototypes for Interrupt Handling Operation
 */
/******************************************************************************
 * @fn				- SPI_TXE_Interrupt_Handle
 *
 * @brief			- SPI TXE Interrupt Handle Helper Function
 *
 * @param[in]		- SPI_Handle_t *pSPIHandle
 *
 * @return			-
 *
 * @notes			-
 *****************************************************************************/
static void SPI_TXE_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
	//check the DFF bit in CR1
	if( pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) )
	{
		//data format is 16 bit
		//1. load the data in to DR
		pSPIHandle->pSPIx->DR = *(uint16_t*)pSPIHandle->pTxBuffer;

		pSPIHandle->pTxBuffer += sizeof(uint16_t);//increment the address of pTxBuffer
		pSPIHandle->TxLen--;


	}
	else
	{
		//data format is 8 bit
		//1. load the data in to DR
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;

		pSPIHandle->pTxBuffer += sizeof(uint8_t);//increment the address of pTxBuffer
		pSPIHandle->TxLen--;//decrement the length variable
	}
	if( !pSPIHandle->TxLen )
	{
		//TxLen is zero, so close the SPI transmission and inform the appliation that
		//TX is over
		//this prevents interrupts from setting up of TXE Flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_TX_CMPLT);


	}
}
/******************************************************************************
 * @fn				- SPI_RXNE_Interrupt_Handle
 *
 * @brief			- SPI RXNE Interrupt Handle Helper Function
 *
 * @param[in]		- SPI_Handle_t *pSPIHandle
 *
 * @return			-
 *
 * @notes			-
 *****************************************************************************/
static void SPI_RXNE_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
	//check the DFF bit in CR1
	if( pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) )
	{
		//data format is 16 bit
		//1. load the data from DR to Rx buffer address
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->pRxBuffer += sizeof(uint16_t);
		pSPIHandle->RxLen--;//decrement the length variable
		//increment the address of pRxBuffer
	}
	else
	{
		//data format is 8 bit
		//1. load the data in to DR
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->pRxBuffer += sizeof(uint8_t);//increment the address of pRxBuffer
		pSPIHandle->RxLen--;//decrement the length variable
	}
	if( !pSPIHandle->RxLen )
	{
		//receptions is completed
		//lets turn of the RXNEIE interrupt
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}
/******************************************************************************
 * @fn				- SPI_OVR_ERR_Interrupt_Handle
 *
 * @brief			- SPI Overrun Error Interrupt Handle Helper Function
 *
 * @param[in]		- SPI_Handle_t *pSPIHandle
 *
 * @return			-
 *
 * @notes			-
 *****************************************************************************/
static void SPI_OVR_ERR_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//clear the OVR Flag
	if( pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//inform the application
	SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_OVR_ERR);

}
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

__attribute__((weak)) void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	//This is a weak impletementation. The application may overwrite on this function.
}



