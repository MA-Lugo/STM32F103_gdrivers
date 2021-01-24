/*
 * stm32f103_gpio_driver.c
 *
 *  Created on: Jan 14, 2021
 *  Author: Mario
 */

#include "stm32f103_gpio_driver.h"


/*
 *				Pheriperal SETUP
 */

/**********************************************************
 * @fn				- GPIO_CLK_Control
 * @brief			- This function enable or disable the peripheral clock for given GPIO port
 *
 * @param[in]		- Base addres of the GPIO Port
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/
void GPIO_CLK_Control(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_CLK_ENABLE();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_CLK_ENABLE();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_CLK_ENABLE();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_CLK_ENABLE();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_CLK_ENABLE();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_CLK_ENABLE();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_CLK_ENABLE();
		}


	}

	else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_CLK_DISABLE();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_CLK_DISABLE();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_CLK_DISABLE();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_CLK_DISABLE();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_CLK_DISABLE();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_CLK_DISABLE();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_CLK_DISABLE();
		}

	}
}

/*
 *				Pheriperal SETUP
 */

/**********************************************************
 * @fn				- GPIO_InitPin
 * @brief			- This function initialize the given GPIO Port PIN
 *
 * @param[in]		- GPIO handle structure
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/

void GPIO_InitPin(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0; // temporal register

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 7)	//CRL register
	{
		temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_Mode)  << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->CRL &= ~(0xF << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));		//clear
		pGPIOHandle->pGPIOx->CRL |= temp;					//set
	}
	else												//CRH register
	{
		temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_Mode)  << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8)));
		pGPIOHandle->pGPIOx->CRH &= ~(0xF << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8)));		//clear
		pGPIOHandle->pGPIOx->CRH |= temp;		//set

	}
}


/**********************************************************
 * @fn				- GPIO_InitPort
 * @brief			- This function initialize all Pins
 * 					  of the given GPIO port
 *
 * @param[in]		- Base addres of the GPIO port
 * @param[in]		- GPIO MODE macro
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/

void GPIO_InitPort(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_MODE)
{
	uint32_t temp = 0; // temporal register

	for (uint8_t q = 0; q <= 28 ; q+=4)
	{
		temp |= (GPIO_MODE  << q);
	}

	pGPIOx->CRL = 0;			//clear
	pGPIOx->CRH = 0;

	pGPIOx->CRL = temp;			//set
	pGPIOx->CRH = temp;

}



/**********************************************************
 * @fn				- GPIO_DeInitPort
 * @brief			- This function de-initialize the given GPIO port
 *
 * @param[in]		- Base addres of the GPIO port
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/
void GPIO_DeInitPort(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}


}

/**********************************************************
 * @fn				- GPIO_SetPull_UorD
 * @brief			- This function configure pull-up or pull-down resistors
 *
 * @param[in]		- GPIO handle structure
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/

void GPIO_SetPull_UorD(GPIO_Handle_t *pGPIOHandle)
{

}



/*
 * Pheriperal READ
 */

/**********************************************************
 * @fn				- GPIO_ReadPin
 * @brief			- This function read the pin of the given GPIO Port
 *
 * @param[in]		- Base addres of the GPIO port
 * @param[in]		- PIN number
 *
 * @return			- 0 or 1
 *
 * @note			- none
 *********************************************************/

uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value =  (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/**********************************************************
 * @fn				- GPIO_ReadPort
 * @brief			- This function read the given GPIO Port
 *
 * @param[in]		- Base addres of the GPIO port
 *
 * @return			- 16 bit number
 *
 * @note			- none
 *********************************************************/
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = pGPIOx->IDR;
	return value;
}

/*
 * Pheriperal WRITE
 */


/**********************************************************
 * @fn				- GPIO_WitePin
 * @brief			- This function write the given GPIO Port Pin
 *
 * @param[in]		- Base addres of the GPIO port
 * @param[in]		- PIN number
 * @param[in]		- Value GPIO_PIN_SET or GPIO_PIN_RESET
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/
void GPIO_WitePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1  << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1  << PinNumber);
	}
}

/**********************************************************
 * @fn				- GPIO_WritePort
 * @brief			- This function write the given GPIO Port
 *
 * @param[in]		- Base addres of the GPIO port
 * @param[in]		- uint16_t value
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/

void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}


/**********************************************************
 * @fn				- GPIO_TogglePin
 * @brief			- This function toggle the given GPIO Port Pin
 *
 * @param[in]		- Base addres of the GPIO port
 * @param[in]		- PIN number
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1  << PinNumber);
}

/*
 * IRQ configuration and ISR handle
 */

/**********************************************************
 * @fn				- GPIO_INT_Config
 * @brief			- This function enable/desable and configure the external interrupt type
 *
 * @param[in]		- Base addres of the GPIO port
 * @param[in]		- PIN number
 * @param[in]		- External interrupt type
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/

void GPIO_INT_Config(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNumber, uint8_t GPIO_INT_TYPE)
{
	//Configure falling or/and Rising edge interrupt
	if(GPIO_INT_TYPE == GPIO_INT_TYPE_FT)
	{
		EXTI->FTSR |= (1 << GPIO_PinNumber);
		EXTI->RTSR &= ~(1 << GPIO_PinNumber);

	}

	else if(GPIO_INT_TYPE == GPIO_INT_TYPE_RT)
	{
		EXTI->RTSR |= (1 << GPIO_PinNumber);
		EXTI->FTSR &= ~(1 << GPIO_PinNumber);
	}

	else if(GPIO_INT_TYPE == GPIO_INT_TYPE_FRT)
	{
		EXTI->FTSR |= (1 << GPIO_PinNumber);
		EXTI->RTSR |= (1 << GPIO_PinNumber);
	}


	//Configure the GPIO port selection in AFIO EXTICR
	uint8_t temp1 = GPIO_PinNumber / 4;
	uint8_t temp2 = GPIO_PinNumber % 4;
	uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOx);
	AFIO_CLK_ENABLE();
	AFIO->EXTICR[temp1] = portcode << (temp2*4);


	//Enable the exti interrupt delivery using IMR
	EXTI->IMR |= (1 << GPIO_PinNumber);


}

/**********************************************************
 * @fn				- GPIO_IRQInterrupt_Config
 * @brief			- This function configure the given IRQ Number
 *
 * @param[in]		- IRQ Number
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/

void GPIO_IRQInterrupt_Config(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)						//ISER0 register
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 &&  IRQNumber < 64)//ISER1 register
		{
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)//ISER2 register
		{
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}

	}

	else
	{
		if (IRQNumber <= 31)						//ICER0 register
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 &&  IRQNumber < 64)//ISER1 register
		{
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)//ISER2 register
		{
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		}
	}
}

/**********************************************************
 * @fn				- GPIO_IRQPiority_Config
 * @brief			- This function set the priority of the
 * 					- given IRQ number
 *
 * @param[in]		- IRQ number
 * @param[in]		- Priority value
 *
 * @return			- none
 *
 * @note			- none
 ********/

void GPIO_IRQPiority_Config(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENT);

	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);


}

/**********************************************************
 * @fn				- GPIO_IRQHandling
 * @brief			- This function clear the pending bit of
 * 					- the EXTI
 *
 * @param[in]		- Pin number of the interruption
 *
 * @return			- none
 *
 * @note			- none
 ********/
void GPIO_IRQHandling(uint8_t GPIO_PinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & (1 << GPIO_PinNumber))
	{
		EXTI->PR |= (1 << GPIO_PinNumber);
	}

}

