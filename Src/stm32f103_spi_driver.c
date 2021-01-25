/*
 * stm32f103_spi_driver.c
 *
 *  Created on: 25 ene. 2021
 *      Author: Mario
 */
#include "stm32f103_spi_driver.h"



void SPI_CLK_Control(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{

}

void SPI_Init(SPI_Handle_t *pSPIHandle)
{

}
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

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

