/*
 * stm32f103_gpio_driver.h
 *
 *  Created on: Jan 14, 2021
 *      Author: Mario
 */

#ifndef INC_STM32F103_GPIO_DRIVER_H_
#define INC_STM32F103_GPIO_DRIVER_H_

#include "stm32f103xx.h"


/*
 * Pin configuration structure for a GPIO Pin
 */

typedef struct
{
	uint8_t GPIO_PinNumber;			/*!< possible values from @GPIO PIN NOMBERS  >*/
	uint8_t GPIO_Mode;				/*!< possible values from @GPIO PIN MODES  >*/
	//uint8_t GPIO_IntControl;
}GPIO_PinConfig_t;

/*
 * @GPIO PIN NOMBERS
 * Gpio pin numbers
 */

#define GPIO_Pin_0						0
#define GPIO_Pin_1						1
#define GPIO_Pin_2						2
#define GPIO_Pin_3						3
#define GPIO_Pin_4						4
#define GPIO_Pin_5						5
#define GPIO_Pin_6						6
#define GPIO_Pin_7						7
#define GPIO_Pin_8						8
#define GPIO_Pin_9						9
#define GPIO_Pin_10						10
#define GPIO_Pin_11						11
#define GPIO_Pin_12						12
#define GPIO_Pin_13						13
#define GPIO_Pin_14						14
#define GPIO_Pin_15						15

/*
 * @GPIO PIN MODES
 * Gpio pin possible modes
 */

//										CNF&MODE bits
#define GPIO_MODE_IN_ANALOG				0b0000U
#define GPIO_MODE_IN_FLOAT				0b0100U
#define GPIO_MODE_IN_PUPDOWN			0b1000U

#define GPIO_MODE_OUT_PPUSH				0b0010U
#define GPIO_MODE_OUT_ODRAIN			0b0110U
#define GPIO_MODE_OUT_AFPPUSH			0b1010U
#define GPIO_MODE_OUT_AFODRAIN			0b1110U

#define GPIO_MODE_OUT_PPUSH_10M			0b0001U
#define GPIO_MODE_OUT_ODRAIN_10M		0b0101U
#define GPIO_MODE_OUT_AFPPUSH_10M		0b1001U
#define GPIO_MODE_OUT_AFODRAIN_10M		0b1101U

#define GPIO_MODE_OUT_PPUSH_50M			0b0011U
#define GPIO_MODE_OUT_ODRAIN_50M		0b0111U
#define GPIO_MODE_OUT_AFPPUSH_50M		0b1011U
#define GPIO_MODE_OUT_AFODRAIN_50M		0b1111U


/*
#define GPIO_MODE_INT_FT
#define GPIO_MODE_INT_RT
#define GPIO_MODE_INT_FRT
*/


/*
 * @GPIO PIN INTERRUPT
 * Gpio pin interrup
 */





/*
 * A Handle structure for a GPIO pin
 */

typedef struct
{
	GPIO_RegDef_t 		*pGPIOx;		/*!<Holds the base address of the GPIO port to wich the pin belongs >*/
	GPIO_PinConfig_t	GPIO_PinConfig; /*!<Holds GPIO pin configuration settings >*/
}GPIO_Handle_t;




/****************************************************************************************************************
 * 						APIs supported by this driver
 *****************************************************************************************************************/


/*
 * Pheriperal SETUP
 */
void GPIO_CLK_Control(GPIO_RegDef_t *GPIOx, uint8_t EnOrDi);

void GPIO_InitPin(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInitPort(GPIO_RegDef_t *GPIOx);

void GPIO_SetPull_UorD(GPIO_Handle_t *pGPIOHandle);

/*
 * Pheriperal READ
 */
uint8_t  GPIO_ReadPin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadPort(GPIO_RegDef_t *GPIOx);

/*
 * Pheriperal WRITE
 */
void GPIO_WitePin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WritePort(GPIO_RegDef_t *GPIOx, uint16_t Value);
void GPIO_TogglePin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber);

/*
 * IRQ configuration and ISR handle
 */

#endif /* INC_STM32F103_GPIO_DRIVER_H_ */
