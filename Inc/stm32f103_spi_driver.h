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



/*
 * Device Mode
 */

#define SPI_DEVICE_MODE_SLAVE		0
#define SPI_DEVICE_MODE_MASTER		1

/*
 * Bus Config
 */

#define SPI_BUS_CONF_FD					1
#define SPI_BUS_CONF_HD					2
#define SPI_BUS_CONF_SIMPLEX_RxONLY		3

/*
 * Clock Speed
 */

#define SPI_CLK_SPEED_DIV2				0
#define SPI_CLK_SPEED_DIV4				1
#define SPI_CLK_SPEED_DIV8				2
#define SPI_CLK_SPEED_DIV16				3
#define SPI_CLK_SPEED_DIV32				4
#define SPI_CLK_SPEED_DIV64				5
#define SPI_CLK_SPEED_DIV128			6
#define SPI_CLK_SPEED_DIV256			7

/*
 * SPI Data frame format (DFF)
 */

#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1

/*
 * SPI Clock Polarity (CPOL)
 */

#define SPI_CPOL_LOW					0
#define SPI_CPOL_HIGH					1

/*
 * SPI Clock Phase (CPHA)
 */

#define SPI_CPHA_LOW					0
#define SPI_CPHA_HIGH					1

/*
 * SPI Software Slave Management
 */

#define SPI_SSM_DISABLE						0
#define SPI_SSM_ENABLE						1


/****************************************************************************************************************
 * 						APIs supported by this driver
 *****************************************************************************************************************/


/*
 * Pheriperal SETUP
 */
void SPI_CLK_Control(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDis);

/*
 * Send and Receive Data
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveDAta(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len);

/*
 * Others
 */

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDis);

/*
 * IRQ configuration and ISR handling
 */

void SPI_IRQInterrupt_Config(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPiority_Config(uint8_t IRQNumber, uint32_t IRQPriority);

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);



#endif /* INC_STM32F103_SPI_DRIVER_H_ */
