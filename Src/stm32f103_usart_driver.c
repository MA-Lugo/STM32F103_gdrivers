/*
 * stm32f103_usart_driver.c
 *
 *  Created on: 7 may. 2021
 *      Author: Mario
 */



#include "stm32f103_usart_driver.h"


void USART_CLK_Control(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
		{
			if (pUSARTx == USART1)
			{
				USART1_CLK_ENABLE();
			}
			else if (pUSARTx == USART2)
			{
				USART2_CLK_ENABLE();
			}
			else if (pUSARTx == USART3)
			{
				USART3_CLK_ENABLE();
			}
			else if (pUSARTx == UART4)
			{
				UART4_CLK_ENABLE();
			}
			else if (pUSARTx == UART5)
			{
				UART5_CLK_ENABLE();
			}


		}

		else
		{
			if (pUSARTx == USART1)
			{
				USART1_CLK_DISABLE();
			}
			else if (pUSARTx == USART2)
			{
				USART2_CLK_DISABLE();
			}
			else if (pUSARTx == USART3)
			{
				USART3_CLK_DISABLE();
			}
			else if (pUSARTx == UART4)
			{
				UART4_CLK_DISABLE();
			}
			else if (pUSARTx == UART5)
			{
				UART5_CLK_DISABLE();
			}


		}
}
