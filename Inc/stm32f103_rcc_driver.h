/*
 * stm32f103_rcc_driver.h
 *
 *  Created on: 13 may. 2021
 *      Author: Mario
 */

#ifndef INC_STM32F103_RCC_DRIVER_H_
#define INC_STM32F103_RCC_DRIVER_H_

#include "stm32f103xx.h"


uint16_t RCC_GetAHB_PresValue(void);

uint32_t RCC_GetAPB1_CLKValue(uint32_t SysCLK_Value);

uint32_t RCC_GetAPB2_CLKValue(uint32_t SysCLK_Value);


#endif /* INC_STM32F103_RCC_DRIVER_H_ */
