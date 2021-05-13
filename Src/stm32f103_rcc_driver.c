/*
 * stm32f103_rcc_driver.c
 *
 *  Created on: 13 may. 2021
 *      Author: Mario
 */


#include "stm32f103_rcc_driver.h"



uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};
uint8_t APB2_PreScaler[4] = {2,4,8,16};





/**********************************************************
 * @fn				- RCC_GetAHB_PreesValue
 * @brief			- This function gets the frequency of the
 * 					  AHB system bus.
 *
 *
 * @return			- Preescaler value of the AHB system bus.
 *
 * @note			- none
 *********************************************************/
uint16_t RCC_GetAHB_PresValue(void)
{
	uint8_t temp;
	//For AHB
	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8)
	{
		return 1;
	}
	else
	{
		return AHB_PreScaler[temp-8];
	}
}

/**********************************************************
 * @fn				- RCC_GetAPB1_CLKValue
 * @brief			- This function gets the frequency of the
 * 					  APB1 bus.
 *
 * @param[in]		- System Clock Value.
 *
 * @return			- Frequency of the APB1.
 *
 * @note			- none
 *********************************************************/
uint32_t RCC_GetAPB1_CLKValue(uint32_t SysCLK_Value)
{
	uint32_t pclk1;
	uint8_t temp,ahbp,apb1p;

	ahbp = RCC_GetAHB_PresValue();
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

	pclk1 = (SysCLK_Value/ahbp)/ apb1p;

	return pclk1;
}

/**********************************************************
 * @fn				- RCC_GetAPB2_CLKValue
 * @brief			- This function gets the frequency of the
 * 					  APB2 bus.
 *
 * @param[in]		- System Clock Value.
 *
 * @return			- Frequency of the APB2.
 *
 * @note			- none
 *********************************************************/
uint32_t RCC_GetAPB2_CLKValue(uint32_t SysCLK_Value)
{
	uint32_t pclk2;
	uint8_t temp,ahbp,apb2p;

	ahbp = RCC_GetAHB_PresValue();
	//For APB2
	temp = ((RCC->CFGR >> 11) & 0x7);
	if(temp < 4)
	{
		apb2p = 1;
	}
	else
	{
		apb2p = APB2_PreScaler[temp-4];
	}

	pclk2 = (SysCLK_Value/ahbp)/ apb2p;

	return pclk2;
}
