/*
 * stm32f103_spi_driver.c
 *
 *  Created on: 25 ene. 2021
 *      Author: Mario
 */
#include "stm32f103_spi_driver.h"

/**********************************************************
 * @fn				- SPI_CLK_Control
 * @brief			- This function enable or disable the peripheral clock
 * 					  for the given SPIx
 *
 * @param[in]		- Base addres of the SPI port
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/

void SPI_CLK_Control(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_CLK_ENABLE();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_CLK_ENABLE();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_CLK_ENABLE();
		}

	}

	else
	{
		if (pSPIx == SPI1)
		{
			SPI1_CLK_DISABLE();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_CLK_DISABLE();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_CLK_DISABLE();
		}

	}

}

void SPI_Init(SPI_Handle_t *pSPIHandle)
{

}


/**********************************************************
 * @fn				- SPI_DeInit
 * @brief			- This function De-Init the given SPI Port
 *
 *
 * @param[in]		- Base addres of the SPIx
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

	if (pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}


}



void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
}
void SPI_ReceiveDAta(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len)
{

}



void SPI_IRQInterrupt_Config(uint8_t IRQNumber, uint8_t EnorDi)
{

}
void SPI_IRQPiority_Config(uint8_t IRQNumber, uint32_t IRQPriority){

}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{

}

