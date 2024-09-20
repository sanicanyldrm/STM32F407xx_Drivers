/*
 * stm32f407_spi_driver.c
 *
 *  Created on: Sep 4, 2024
 *      Author: sanic
 */


#include "stm32f407xx_spi_driver.h"


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
