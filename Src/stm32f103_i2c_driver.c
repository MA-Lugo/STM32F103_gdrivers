/*
 * stm32f103_i2c_driver.c
 *
 *  Created on: 6 mar. 2021
 *      Author: Mario
 */

#include "stm32f103_i2c_driver.h"


/**********************************************************
 * @fn				- I2C_CLK_Control
 * @brief			- This function enable or disable the peripheral clock
 * 					  for the given I2Cx
 *
 * @param[in]		- Base addres of the I2C port
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/

void I2C_CLK_Control(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_CLK_ENABLE();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_CLK_ENABLE();
		}

	}

	else
	{
		if (pI2Cx == I2C1)
		{
			I2C1_CLK_DISABLE();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_CLK_DISABLE();
		}

	}

}

/**********************************************************
 * @fn				- I2C_DeInit
 * @brief			- This function De-Init the given I2C Port
 *
 *
 * @param[in]		- Base addres of the I2Cx
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{

	if (pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if (pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}

}

/**********************************************************
 * @fn				- I2C_PeripheralControl
 * @brief			- This function enable or disable the peripheral
 *
 *
 * @param[in]		- Base addres of the I2Cx
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @note			-
 *********************************************************/
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}
