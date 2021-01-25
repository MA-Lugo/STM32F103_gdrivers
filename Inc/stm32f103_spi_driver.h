/*
 * stm32f103_spi_driver.h
 *
 *  Created on: 25 ene. 2021
 *      Author: Mario
 */

#ifndef INC_STM32F103_SPI_DRIVER_H_
#define INC_STM32F103_SPI_DRIVER_H_

#include "stm32f103xx.h"


/*
 * Configutation structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_CLKSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_PinConfig_t;


/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_PinConfig_t SPIConfig;

}SPI_Handle_t;


/****************************************************************************************************************
 * 						APIs supported by this driver
 *****************************************************************************************************************/


/*
 * Pheriperal SETUP
 */
void SPI_CLK_Control(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/*
 * Send and Receive Data
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveDAta(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len);



/*
 * IRQ configuration and ISR handling
 */

void SPI_IRQInterrupt_Config(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPiority_Config(uint8_t IRQNumber, uint32_t IRQPriority);

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);



#endif /* INC_STM32F103_SPI_DRIVER_H_ */
