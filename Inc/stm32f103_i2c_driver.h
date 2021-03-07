/*
 * stm32f103_i2c_driver.h
 *
 *  Created on: 6 mar. 2021
 *      Author: Mario
 */

#ifndef INC_STM32F103_I2C_DRIVER_H_
#define INC_STM32F103_I2C_DRIVER_H_

#include "stm32f103xx.h"


typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviveAddres;
	uint8_t  I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;


/*
 * Handle structure for I2Cx peripheral
 */

typedef struct
{
	uint32_t	 SYSTEM_CLK;
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
}I2C_Handle_t;


/*
 * I2C_SCLSpeed macros
 */
#define I2C_SCL_SPEED_SM			100000
#define I2C_SCL_SPEED_FM400K		400000
#define I2C_SCL_SPEED_FM200K		200000

/*
 * I2C_ACKControl macros
 */

#define I2C_ACK_ENABLE				1
#define I2C_ACK_DISABLE				0

/*
 * I2C_FMDutyCycle macros
 */
#define I2C_FM_DUTY_2				0
#define I2C_FM_DUTY_16_9			1



/****************************************************************************************************************
 * 						APIs supported by this driver
 *****************************************************************************************************************/


/*
 * Pheriperal SETUP
 */
void I2C_CLK_Control(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

uint32_t RCC_GetPCLK1Value(uint32_t CLK_Source);
void I2C_Init(I2C_Handle_t *pI2CHandle);

void I2C_DeInit(I2C_RegDef_t *pI2Cx);

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis);

/*
 * Send and Receive Data
 */

/*
 * IRQ configuration and ISR handling
 */

void I2C_IRQInterrupt_Config(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPiority_Config(uint8_t IRQNumber, uint32_t IRQPriority);

/*
 * Others
 */

#endif /* INC_STM32F103_I2C_DRIVER_H_ */
