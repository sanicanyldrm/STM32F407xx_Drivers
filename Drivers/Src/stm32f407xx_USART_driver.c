#include "stm32_f407xx_USART_driver.h"








/******************************************************************************
 * @fn				- USART_PeriClockControl
 *
 * @brief			- USART peripheral clock control function
 *
 * @param[in]		- USART_RegDef_t *pUSARTx
 * @param[in]		- uint8_t EnorDi
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PLCK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else
		{
			USART6_PCLK_EN();
		}
	}
	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PLCK_DI();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else
		{
			USART6_PCLK_DI();
		}
	}
}



/******************************************************************************
 * @fn				- USART_PeripheralControl
 *
 * @brief			- USART peripheral control function
 *
 * @param[in]		- USART_RegDef_t *pUSARTx
 * @param[in]		- uint8_t EnorDi
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
			{
				pUSARTx->CR1 |= (1 << USART_CR1_UE);
			}
			else
			{
				pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
			}
}



/******************************************************************************
 * @fn				- USART_GetFlagStatus
 *
 * @brief			- USART peripheral control function
 *
 * @param[in]		- SPI_RegDef_t *pSPIx
 * @param[in]		- uint8_t EnorDi
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
	uint8_t ret_val;
	if(pUSARTx->SR & FlagName)
	{
		ret_val = FLAG_SET;
	}
	else
	{
		ret_val = FLAG_RESET;
	}
	return ret_val;




}


/******************************************************************************
 * @fn				- USART_ClearFlag
 *
 * @brief			- USART clear flag function
 *
 * @param[in]		- USART_RegDef_t *pUSARTx
 * @param[in]		- uint16_t StatusFlagName
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	pUSARTx->SR = ~(1 << StatusFlagName);
}



/******************************************************************************
 * @fn				- USART_IRQPriorityConfig
 *
 * @brief			- USART IRQ Priority configuration function
 *
 * @param[in]		- uint8_t IRQPriority

 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. find out ipr register
		uint8_t iprx = IRQNumber / 4;
		uint8_t iprx_section = IRQNumber % 4;

		uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED );
		*( NVIC_PR_BASE_ADDR + iprx ) |= ( IRQPriority << shift_amount );
}





/******************************************************************************
 * @fn				- usart_IRQInterruptConfig
 *
 * @brief			- USART SPI IRQ Interrupt configuration function
 *
 * @param[in]		- uint8_t IRQNumber
 * @param[in]		- uint8_t EnorDi
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn				- USART_Init
 *
 * @brief			- USART Initialization function
 *
 * @param[in]		- USART_Handle_t *pUSARTHandle
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	/******************************** Configuration of CR1******************************************/
	//Temp Register
	uint32_t tempRegister = 0;

	//Enables the Clock for given USART peripheral
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//Enables USART Tx and Rx engines according to the USART_Mode configuration item
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		tempRegister |= (1 << USART_CR1_TE);
	}
	else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		tempRegister |= (1 << USART_CR1_RE);
	}
	else
	{
		tempRegister |= ((1 << USART_CR1_TE) | (1 << USART_CR1_RE));
	}

	//Code to configure the Word length configuration item
	tempRegister |= (pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M);

	//Configure of parity control bit fields
	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		tempRegister |= (1 << USART_CR1_PCE);


	}
	else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
	{
		tempRegister |= (1 << USART_CR1_PCE);

		tempRegister |= (1 << USART_CR1_PS);
	}
	else
	{
		//do nothing
	}

	pUSARTHandle->pUSARTx->CR1 = tempRegister;

	/******************************** Configuration of CR2******************************************/
	tempRegister = 0;

	//Code to configure the number of stop bits inserted during USART frame transmission
	tempRegister |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	pUSARTHandle->pUSARTx->CR2 = tempRegister;

	/******************************** Configuration of CR3******************************************/

	tempRegister = 0;
	//Configuration of USART hardware flow control
	if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		tempRegister |= (1 << USART_CR3_CTSE);
	}
	if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		tempRegister |= (1 << USART_CR3_RTSE);
	}
	else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		tempRegister |= (1 << USART_CR3_CTSE);
		tempRegister |= (1 << USART_CR3_RTSE);
	}
	else
	{
		//do nothing
	}

	pUSARTHandle->pUSARTx->CR3 = tempRegister;

	/******************************** Configuration of BRR(Baudrate register)******************************************/

	USART_SetBaudRate(pUSARTHandle->pUSARTx,pUSARTHandle->USART_Config.USART_Baud);
}





/******************************************************************************
 * @fn				- USART_SendData
 *
 * @brief			- USART send data function
 *
 * @param[in]		- USART_Handle_t *pUSARTHandle
 * @param[in]		- uint8_t *pTxBuffer
 * @param[in]		- uint32_t Len
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pData = NULL;

	//Send data until all data is transferred
	uint32_t i;
	for(i = 0; i < Len; i++)
	{
		//Wait until TXE is set
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

		//Check is 9 bits or 8 bits
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			*pData = (uint16_t*)pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pData & (uint16_t)0x01FF);

			//Check parity control
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//8bit data transfer
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;


		}
	}

	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}





/******************************************************************************
 * @fn				- USART_ReceiveData
 *
 * @brief			- USART receive data function
 *
 * @param[in]		- USART_Handle_t *pUSARTHandle
 * @param[in]		- uint8_t *pTxBuffer
 * @param[in]		- uint32_t Len
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	//Receive data until all data is transferred
	uint32_t i;
	for(i = 0; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

		//Check word lenght is 8 bits or 9 bits
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//9 bit data will be received

			//Check parity control bit
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//parity bit is not used all 9 bits are user data
				*((uint16_t*)pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);

				//increment rx buffer
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
				pRxBuffer++;
			}

		}
		else
		{
			//8 bit data will be received
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
			}
			else
			{
				//parit is used so we have to extract parity bit from user data
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
			}
			//increment rx buffer
			pRxBuffer++;
		}
	}
}



/******************************************************************************
 * @fn				- USART_SendDataIT
 *
 * @brief			- USART send data interrupt function
 *
 * @param[in]		- USART_Handle_t *pUSARTHandle
 * @param[in]		- uint8_t *pTxBuffer
 * @param[in]		- uint32_t Len
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t TxState = pUSARTHandle->TxBusyState;

	if(TxState != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;


		//enable the intterupt
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		//enable transmission completed interrupt
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}
	return TxState;
}


/******************************************************************************
 * @fn				- USART_ReceiveDataIT
 *
 * @brief			- USART receive data interrupt function
 *
 * @param[in]		- USART_Handle_t *pUSARTHandle
 * @param[in]		- uint8_t *pRxBuffer
 * @param[in]		- uint32_t Len
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t RxState = pUSARTHandle->RxBusyState;

	if(RxState != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_RX;

		//enable interrupt
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}

	return RxState;
}





/******************************************************************************
 * @fn				- USART_SetBaudRate
 *
 * @brief			- USART baud rate configuration function
 *
 * @param[in]		- USART_RegDef_t *pUSARTx
 * @param[in]		- uint32_t BaudRate
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

	uint32_t tempreg=0;

	//Get the value of APB bus clock in to the variable PCLKx
	if(pUSARTx == USART1 || pUSARTx == USART6)
	{
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2Value();
	}else
	{
	   PCLKx = RCC_GetPCLK1Value();
	}

	//Check for OVER8 configuration bit
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
	}else
	{
	   //over sampling by 16
	   usartdiv = ((25 * PCLKx) / (4 *BaudRate));
	}

	//Calculate the Mantissa part
	M_part = usartdiv/100;

	//Place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= M_part << 4;

	//Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	//Calculate the final fractional
	if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
	{
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

	}else
	{
	   //over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

	}

	//Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= F_part;

	//copy the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;
}




/******************************************************************************
 * @fn				- USART_IRQHandling
 *
 * @brief			- USART interrupt handling function
 *
 * @param[in]		- USART_Handle_t *pHandle
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t temp1;
	uint32_t temp2;
	uint32_t temp3;

	uint16_t *pData;

	/*************************Check for TC flag ********************************************/
	//checks the state of TC bit in the SR
	temp1 = ( pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC) );

	//Checks the state of TCEIE bit
	temp2 = ( pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE) );

	if(temp1 & temp2)
	{
		//interrupt happening due to transmission completed interrupt
		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//if TxLen is 0 close the transmission
			if(! pUSARTHandle->TxLen)
			{
				//clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);

				//clear the TCIE control bit
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TCIE);

				//reset the application state
				pUSARTHandle->TxBusyState = USART_READY;

				//resets the address buffer to NULL
				pUSARTHandle->pTxBuffer = NULL;

				//resets the length
				pUSARTHandle->TxLen = 0;

				//call the application callback with event USART_EVENT_TX_CMPLT
				//USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}


}







//__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event)
//{

//}










