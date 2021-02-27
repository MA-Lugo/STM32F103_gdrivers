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

/**********************************************************
 * @fn				- SPI_Init
 * @brief			- This function initialize the given
 * 					  spi port
 *
 * @param[in]		- SPI handle structure
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t temp_reg = 0;

	//1. Configure the devie mode
	temp_reg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONF_FD)
	{
		temp_reg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONF_HD)
	{
		temp_reg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONF_SIMPLEX_RxONLY)
	{
		temp_reg &= ~(1 << SPI_CR1_BIDIMODE);
		temp_reg |= (1 << SPI_CR1_RXONLY);
	}

	//3. Configure the spi serial clock speed
	temp_reg |= (pSPIHandle->SPIConfig.SPI_CLKSpeed << SPI_CR1_BR);

	//4 Configure the DFF
	temp_reg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	//5. Configure the CPOL
	temp_reg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	//6. Configure the CPHA
	temp_reg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	//7. Configure the SSM
	temp_reg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);



	pSPIHandle->pSPIx->CR1 = temp_reg;
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

/**********************************************************
 * @fn				- SPI_PeripheralControl
 * @brief			- This function enable or disable the peripheral
 *
 *
 * @param[in]		- Base addres of the SPIx
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @note			-
 *********************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		pSPIx->CR1 |= (1 << 6);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << 6);
	}
}

/**********************************************************
 * @fn				- SPI_SendData
 * @brief			- This function send data from the
 * 					  given SPI port
 *
 *
 * @param[in]		- Base addres of the SPIx
 * @param[in]		- user Tx buffer pointer
 * @param[in]		- length of data (number of bytes)
 *
 * @return			- none
 *
 * @note			- This is blocking call
 *********************************************************/

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until TX is set (Tx reg buff empty)
		while( !(pSPIx->CR2 & (1 << SPI_SR_TXE)) );

		//2. check the DFF bit
		if( (pSPIx->CR1 & (1 << SPI_CR1_DFF)) )
		{
			//16 bit DFF
			//1. load the data in to the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len -=2;
			(uint16_t*) pTxBuffer ++;
		}
		else
		{
			//8 bit DFF
			//1. load the data in to the DR
			pSPIx->DR = *(pTxBuffer);
			Len --;
			pTxBuffer ++;
		}

	}
}
void SPI_ReceiveDAta(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len)
{

}


/**********************************************************
 * @fn				- SPI_SSIConfig
 * @brief			- This function enable or disbale
 * 					  the Internal slave select (SSI) bit
 *
 *
 * @param[in]		- Base addres of the SPIx
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @note			-
 *********************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		pSPIx->CR1 |= (1 << 8);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << 8);
	}
}

void SPI_IRQInterrupt_Config(uint8_t IRQNumber, uint8_t EnorDi)
{

}
void SPI_IRQPiority_Config(uint8_t IRQNumber, uint32_t IRQPriority){

}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{

}

