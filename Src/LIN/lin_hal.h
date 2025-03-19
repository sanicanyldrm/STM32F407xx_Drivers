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

uint8_t buffer_index = 0;
uint8_t RxBuffer[9];



void LIN_Init(void);
void LIN_DeInit(void);
void LIN_ISR(void);


#endif /* LIN_LIN_HAL_H_ */
