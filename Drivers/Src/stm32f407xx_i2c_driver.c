/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Oct 14, 2024
 *      Author: sanic
 */


#include "stm32f407xx_i2c_driver.h"


/*
 * Private Functions prototype
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);




/**************************************************************************************************************************************************************
 *																API's helper functions
 **************************************************************************************************************************************************************/

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}



/******************************************************************************
 * @fn				- I2C_GetFlagStatus
 *
 * @brief			- This function gets realted flag status
 *
 * @param[in]		- I2C_RegDef_t *pI2Cx
 * @param[in]		- uint32_t FlagName
 *
 * @return			- uint8_t flag_val
 *
 * @notes			-
 *****************************************************************************/
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{

	uint8_t flag_val;



	if(pI2Cx->SR1 & FlagName)
	{
		flag_val = FLAG_SET;
	}
	else
	{
		flag_val = FLAG_RESET;
	}

	return flag_val;

}




/******************************************************************************
 * @fn				- I2C_ExecuteAddressPhaseWrite
 *
 * @brief			- This function executes the address phase of transmisson for write operation
 *
 * @param[in]		- I2C_RegDef_t *pI2Cx
 * @param[in]		- uint8_t SlaveAddr
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = ( SlaveAddr << 1 );
	SlaveAddr &= ~(0x1);

	//SlaveAddr = Slave address + R/W bit(0)
	pI2Cx->DR = SlaveAddr;

}




/******************************************************************************
 * @fn				- I2C_ExecuteAddressPhaseRead
 *
 * @brief			- This function executes the address phase of transmisson for read operation
 *
 * @param[in]		- I2C_RegDef_t *pI2Cx
 * @param[in]		- uint8_t SlaveAddr
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = ( SlaveAddr << 1 );
	SlaveAddr |= (0x1);

	//SlaveAddr = Slave address + R/W bit(0)
	pI2Cx->DR = SlaveAddr;

}







/******************************************************************************
 * @fn				- I2C_ClearADDRFlag
 *
 * @brief			- This function clears ADDR flag
 *
 * @param[in]		- I2C_RegDef_t *pI2Cx
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead;
	//the ADDR flag will be cleared, when software is read SR1 and SR2 register
	dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}





/******************************************************************************
 * @fn				- I2C_GenerateStopCondition
 *
 * @brief			- This function generates STOP condition
 *
 * @param[in]		- I2C_RegDef_t *pI2Cx
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}




/**************************************************************************************************************************************************************
 *																API's supported by this driver
 **************************************************************************************************************************************************************/





/******************************************************************************
 * @fn				- I2C_PeriClockControl
 *
 * @brief			- I2C peripheral clock control function
 *
 * @param[in]		- I2C_RegDef_t *pI2Cx
 * @param[in]		- uint8_t EnorDi
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else
		{
			I2C3_PCLK_DI();
		}
	}
}









/******************************************************************************
 * @fn				- I2C_Init
 *
 * @brief			- I2C Initialization function
 *
 * @param[in]		- I2C_Handle_t *pI2CHandle
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;
	uint16_t ccr_value;
	uint8_t trise;


	//Enable the clock for I2C peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//Ack control bit
	tempreg |= ( pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK );

	//Configure the freq field in CR2
	tempreg = 0;
	tempreg |= ( RCC_GetPCLK1Value() / 1000000U );
	pI2CHandle->pI2Cx->CR2 = ( tempreg & 0x3F);

	//program the device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= ( 1 << 14 );
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//Mode is standart mode
		ccr_value = ( RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		tempreg |= (ccr_value & ( 0xFFF ) );
	}
	else
	{
		//Mode is fast mode
		tempreg |= (1 << 15);
		tempreg |= ( pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14 );

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = ( RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed) );
		}
		else
		{
			ccr_value = ( RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed) );
		}
		tempreg |= (ccr_value & 0xFFF );
	}
	pI2CHandle->pI2Cx->CCR = tempreg;


	//TRISE configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//Standard Mode
		trise = ( ( RCC_GetPCLK1Value() / 1000000 ) + 1 );
	}
	else
	{
		//Fast Mode
		trise = ( ( ( RCC_GetPCLK1Value() * 300 ) / 1000000000 ) + 1 );
	}

	pI2CHandle->pI2Cx->TRISE |= ( trise & 0x3F );
}






/******************************************************************************
 * @fn				- I2C_DeInit
 *
 * @brief			- I2C De-initialization function
 *
 * @param[in]		- SPI_RegDef_t *pSPIx
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else
	{
		I2C3_REG_RESET();
	}
}





/******************************************************************************
 * @fn				- I2C_MasterSendData
 *
 * @brief			- I2C function to send data from Master to Slave
 * 					  Blocking function. Use interrupt handling functions for
 * 					  sending data to slave.
 *
 * @param[in]		- I2C_Handle_t *pI2CHandle
 * @param[in]		- uint8_t *pTxBuffer
 * @param[in]		- uint32_t Len
 * @param[in]		- uint8_t SlaveAddr
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	//1. Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking SB flag in the SR1
	//Note: Until SB is cleared SCL will be streched (pulled to low)
	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB) );

	//3. Send the address of the slave with r/w bit set to w(0) (total bits = 8 bit)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) );

	//5. Clear the ADDR flag according to its software sequence
	//Note: Until ADDR is cleared SCL will be streched (pulled to low)
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	//6. Send the data until Len becomes zero
	while( Len > 0)
	{
		while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE) );//wait until TXE is set
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//7. When Len becomes zero wait for TXE=1 and BTF=1 before generating the stop condition
	//Note: TXE=1, BTF=1, means that both shift register and data register are empty and next tranmission should begin
	//When BTF=1, SCL will be streched (pulled to low)
	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE) );

	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF) );

	//8. Generate STOP condition and master need not to wait for the completion of stop condition
	//Note: generating STOP, automatically clears the BTF
	if (Sr == I2C_NO_SR)
	{
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
	else
	{
		//do nothing(due toMISRA-C Coding Standards)
	}

}


/******************************************************************************
 * @fn				- I2C_MasterReceiveData
 *
 * @brief			- I2C function to receive data from Slave to Master
 * 					  Blocking function. Use interrupt handling functions for
 * 					  receiving data to slave.
 *
 * @param[in]		- I2C_Handle_t *pI2CHandle
 * @param[in]		- uint8_t *pRxBuffer
 * @param[in]		- uint32_t Len
 * @param[in]		- uint8_t SlaveAddr
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr)
{
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in SR1
	//Note: Until SB is cleared SCL will be stretched (pulled to low)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/w bit set to R(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Wait until address phase is completed by checking the ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR))

	if(Len == 1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

		//Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//Wait until RXNE becomes 1
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		//Generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//Read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;


	}

	//procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//read the data until Len becomes zero
		for(uint32_t i = Len; i > 0; i--)
		{
			//wait until RXNE becomes 1
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if(i == 2)
			{
				//clear the ACK bit
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//generate the STOP condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

			}

			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;

		}
	}

	//re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_ACKControl == ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}




/******************************************************************************
 * @fn				- I2C_MasterSendDataIT
 *
 * @brief			- This function is used for sending data from Master to Slave
 * 					  This is interrupt function so this function does not block
 * 					  the software.
 *
 * @param[in]		- I2C_Handle_t *pI2CHandle
 * @param[in]		- uint8_t *pTxBuffer
 * @param[in]		- uint32_t Len
 * @param[in]		- uint8_t SlaveAddr
 * @param[in]		- uint8_t Sr
 *
 * @return			- uint8_t busystate
 *
 * @notes			-
 *****************************************************************************/
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = ( pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_BUSY ) );

	if((busystate != I2C_BUSY_IN_RX) && (busystate != I2C_BUSY_IN_TX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;
}


/******************************************************************************
 * @fn				- I2C_MasterReceiveDataIT
 *
 * @brief			- This function is used for receiving data from Slave to Master
 * 					  This is interrupt function so this function does not block
 * 					  the software.
 *
 * @param[in]		- I2C_Handle_t *pI2CHandle
 * @param[in]		- uint8_t *pTRBuffer
 * @param[in]		- uint32_t Len
 * @param[in]		- uint8_t SlaveAddr
 * @param[in]		- uint8_t Sr
 *
 * @return			- uint8_t busystate
 *
 * @notes			-
 *****************************************************************************/
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = ( pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_BUSY ) );

	if((busystate != I2C_BUSY_IN_RX) && (busystate != I2C_BUSY_IN_TX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
				I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		 pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;
}













/******************************************************************************
 * @fn				- I2C_IRQInterruptConfig
 *
 * @brief			- I2C SPI IRQ Interrupt configuration function
 *
 * @param[in]		- uint8_t IRQNumber
 * @param[in]		- uint8_t EnorDi
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn				- I2C_IRQPriorityConfig
 *
 * @brief			- I2C IRQ Priority configuration function
 *
 * @param[in]		- uint8_t IRQPriority

 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

	//1. find out ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED );
	*( NVIC_PR_BASE_ADDR + iprx ) |= ( IRQPriority << shift_amount );

}




/******************************************************************************
 * @fn				- I2C_EV_IRQHandling
 *
 * @brief			- I2C event interrupt handling function
 *
 * @param[in]		- I2C_Handle_t *pI2CHandle

 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	//Interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);

	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if(temp1 && temp3)
	{
		//interrupt generated due to SB flag
		//this code will not be executed in slave mode because in slave mode SB flag is always zero
		//start condition generated succesfully so execute address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	if(temp1 && temp3)
	{
		//ADDR flag is set
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//make sure that TXE flag is set
			if(pI2CHandle->pI2Cx->SR1 && (1 << I2C_SR1_TXE))
			{
				if(pI2CHandle->TxLen == 0)
				{
					//BTF and TXE are set
					//close the transmission, generate stop condition
					//if(pI2CHandle->Sr == I2C_DISABLE_SR)
					//{
					//I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
					//}

					////reset all the member of handle structure
					//I2C_CloseSendData();

					//notify the application about transmission is completed
					//I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);

				}

			}
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			//do nothing
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	if(temp1 & temp3)
	{
		//STOPF flag is set
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (I2C_SR1_TXE);
	//5. Handle For interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		//TxE flag is set
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	//6. Handle For interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		//RxNE flag is set
	}

}







/******************************************************************************
 * @fn				- I2C_ER_IRQHandling
 *
 * @brief			- I2C error interrupt handling function
 *
 * @param[in]		- I2C_Handle_t *pI2CHandle

 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

}










/******************************************************************************
 * @fn				- I2C_PeripheralControl
 *
 * @brief			- I2C peripheral control function
 *
 * @param[in]		- I2C_RegDef_t *pSPIx
 * @param[in]		- uint8_t EnorDi
 *
 * @return			- void
 *
 * @notes			-
 *****************************************************************************/
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= ( 1 << 0 );
	}
	else
	{
		pI2Cx->CR1 &= ~( 1 << 0 );
	}
}



void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		//enable the ACK
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		//disable the ACK
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}








