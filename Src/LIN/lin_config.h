/*
 * lin_config.h
 *
 *  Created on: Mar 17, 2025
 *      Author: can.yildirim
 */

#ifndef LIN_LIN_CONFIG_H_
#define LIN_LIN_CONFIG_H_

#include "stm32f407xx.h"


#define LIN_MODE 						0
#define FEATURE_SLEEP_MODE				0



#define LIN_FLAG_LBD					(1 << USART_SR_LBD)
#define LIN_FLAG_RXNE					(1 << USART_SR_RXNE)


#define LIN_SYNC_FIELD					0x55U

#endif /* LIN_LIN_CONFIG_H_ */
