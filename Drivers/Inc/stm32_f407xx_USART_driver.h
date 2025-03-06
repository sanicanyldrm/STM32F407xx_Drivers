#ifndef STM32F407XX_USART_DRIVER_
#define STM32F407XX_USART_DRIVER_




/*
 * @brief Configuration structure for USARTx peripheral
 */
typedef struct
{
	uint8_t USART_Mode;
	uint8_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;



/*
 * Handle structure for USARTx peripheral
 */
typedef struct
{
	USART_RegDef_t  *pUSARTx;
	USART_Config_t  USART_Config;
}USART_Handle_t;
























#endif
