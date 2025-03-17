/*
 * lin_hal.h
 *
 *  Created on: Mar 17, 2025
 *      Author: can.yildirim
 */

#ifndef LIN_LIN_HAL_H_
#define LIN_LIN_HAL_H_

#include "stm32f407xx.h"
#include "stm32_f407xx_USART_driver.h"
#include "lin_config.h"
#include "lin_protocol.h"


USART_Handle_t LIN_Handle_Config;



extern void LIN_Init(void);
extern void LIN_DeInit(void);
extern void LIN_ISR(void);


#endif /* LIN_LIN_HAL_H_ */
