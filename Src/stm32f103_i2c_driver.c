/*
 * stm32f103_i2c_driver.c
 *
 *  Created on: 6 mar. 2021
 *      Author: Mario
 */

#include "stm32f103_i2c_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};

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

uint32_t RCC_GetPCLK1Value(uint32_t CLK_Source)
{
	uint32_t pclk1;
	uint8_t temp,ahbp,apb1p;

	//For AHB
	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	//For APB1
	temp = ((RCC->CFGR >> 8) & 0x7);
	if(temp < 4)
	{
		apb1p = 1;
	}
	else
	{
		apb1p = APB1_PreScaler[temp-4];
	}

	pclk1 = (CLK_Source/ahbp)/ apb1p;

	return pclk1;
}

/**********************************************************
 * @fn				- I2C_Init
 * @brief			- This function Initialize the given I2C
 * 					  Port
 *
 *
 * @param[in]		- Base addres of the I2Cx
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg;

	//Configure AckControl
	tempreg = pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 = tempreg;
	//Configure the FREQ field of CR2
	tempreg = RCC_GetPCLK1Value(pI2CHandle->SYSTEM_CLK) / 1000000U;
	pI2CHandle->pI2Cx->CR2 |= (tempreg & 0x3F);
	//Configure the device own address
	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviveAddres << 1);
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;
	//Configurate the CCR register
	uint16_t ccr_value;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		ccr_value = (RCC_GetPCLK1Value(pI2CHandle->SYSTEM_CLK) / (2*pI2CHandle->I2C_Config.I2C_SCLSpeed) );
		tempreg = (ccr_value & 0xFFF);
	}
	else
	{
		tempreg = (1 <<15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle <<14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value(pI2CHandle->SYSTEM_CLK) / (3*pI2CHandle->I2C_Config.I2C_SCLSpeed) );

		}
		else
		{
			ccr_value = (RCC_GetPCLK1Value(pI2CHandle->SYSTEM_CLK) / (25*pI2CHandle->I2C_Config.I2C_SCLSpeed) );

		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

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
