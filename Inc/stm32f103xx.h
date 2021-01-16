/*
 * stm32f103xx.h
 *
 *  Created on: Jan 14, 2021
 *      Author: Mario
 */
#include <stdint.h>


#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

/*
 * base address of the Flash and SRAM memories
 */
#define FLASH_BASEADDR				0x08000000U
#define SRAM_BASEADDR				0x20000000U
#define ROM_BASEADDR				0x1FFFF000U

/*
 * APBx Bus Peripheral and AHB System Bus base addreses
 */
#define PERIPH_BASEADDR				0x40000000U
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR 		0x40010000U
#define AHBPERIPH_BASEADDR			0x40018000U

/*
 * Base adresses of peripherals which are hangin
 * on a APB1 bus
 */

#define TIM2_BASEADDR				(APB1PERIPH_BASEADDR )
#define TIM3_BASEADDR				(APB1PERIPH_BASEADDR + 0x0400)
#define TIM4_BASEADDR				(APB1PERIPH_BASEADDR + 0x0800)
#define TIM5_BASEADDR				(APB1PERIPH_BASEADDR + 0x0C00)
#define TIM6_BASEADDR				(APB1PERIPH_BASEADDR + 0x1000)
#define TIM7_BASEADDR				(APB1PERIPH_BASEADDR + 0x1400)
#define TIM12_BASEADDR				(APB1PERIPH_BASEADDR + 0x1800)
#define TIM13_BASEADDR				(APB1PERIPH_BASEADDR + 0x1C00)
#define TIM14_BASEADDR				(APB1PERIPH_BASEADDR + 0x2000)
#define RTC_BASEADDR				(APB1PERIPH_BASEADDR + 0x2800)
#define WWDG_BASEADDR				(APB1PERIPH_BASEADDR + 0x2C00)
#define IWDT_BASEADDR				(APB1PERIPH_BASEADDR + 0x3000)
#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000)
#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800)
#define BXCAN1_BASEADDR				(APB1PERIPH_BASEADDR + 0x6400)
#define BXCAN2_BASEADDR				(APB1PERIPH_BASEADDR + 0x6800)
#define DAC_BASEADDR				(APB1PERIPH_BASEADDR + 0x7400)

/*
 * Base adresses of peripherals which are hangin
 * on a APB2 bus
 */
#define AFIO_BASEADDR				APB2PERIPH_BASEADDR
#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x0400)
#define GPIOA_BASEADDR				(APB2PERIPH_BASEADDR + 0x0800)
#define GPIOB_BASEADDR				(APB2PERIPH_BASEADDR + 0x0C00)
#define GPIOC_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000)
#define GPIOD_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400)
#define GPIOE_BASEADDR				(APB2PERIPH_BASEADDR + 0x1800)
#define GPIOF_BASEADDR				(APB2PERIPH_BASEADDR + 0x1C00)
#define GPIOG_BASEADDR				(APB2PERIPH_BASEADDR + 0x2000)
#define ADC1_BASEADDR				(APB2PERIPH_BASEADDR + 0x2400)
#define ADC2_BASEADDR				(APB2PERIPH_BASEADDR + 0x2800)
#define TIM1_BASEADDR				(APB2PERIPH_BASEADDR + 0x2C00)
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)
#define TIM8_BASEADDR				(APB2PERIPH_BASEADDR + 0x3400)
#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800)
#define ADC3_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)
#define TIM9_BASEADDR				(APB2PERIPH_BASEADDR + 0x4C00)
#define TIM10_BASEADDR				(APB2PERIPH_BASEADDR + 0x5000)
#define TIM11_BASEADDR				(APB2PERIPH_BASEADDR + 0x5400)

/*
 * Base adresses of peripherals which are hangin
 * on a AHB bus
 */

#define SDIO_BASEADDR				(AHBPERIPH_BASEADDR)
#define DMA1_BASEADDR				0x40020000U
#define DMA2_BASEADDR				0x40020400U
#define RCC_BASEADDR				0x40021000U



/*
 **************************Peripheral register definition  structures********************
 */

typedef struct
{
	volatile uint32_t CRL;				//Port configuration register low	OFSET:0x00
	volatile uint32_t CRH;				//Port configuration register high	OFSET:0x04
	volatile uint32_t IDR;				//Port input data register			OFSET:0x08
	volatile uint32_t ODR;				//Port output data register			OFSET:0x0C
	volatile uint32_t BSRR;				//Port bit set/reset register		OFSET:0x10
	volatile uint32_t BRR;				//Port bit reset register			OFSET:0x14
	volatile uint32_t LCKR;				//Port configuration lock register	OFSET:0x18

}GPIO_RegDef_t;

typedef struct
{
	volatile uint32_t CR;			//Clock control register				OFSET:0x00
	volatile uint32_t CFGR;			//Clock configuration register			OFSET:0x04
	volatile uint32_t CIR;			//Clock interrupt register				OFSET:0x08
	volatile uint32_t APB2RSTR;		//APB2 peripheral reset register		OFSET:0x0C
	volatile uint32_t APB1RSTR;		//APB1 peripheral reset register		OFSET:0x10
	volatile uint32_t AHBENR;		//AHB peripheral clock enable register	OFSET:0x14
	volatile uint32_t APB2ENR;		//APB2 peripheral clock enable register	OFSET:0x18
	volatile uint32_t APB1ENR;		//APB1 peripheral clock enable register	OFSET:0x1C
	volatile uint32_t BDCR;			//Backup domain control register		OFSET:0x20
	volatile uint32_t CSR;			//Control/status register				OFSET:0x24
}RCC_RegDef_t;



/*
 * Peripheral definitions (peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA						((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB						((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC						((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD						((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE						((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF						((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG						((GPIO_RegDef_t*)GPIOG_BASEADDR)

#define RCC							((RCC_RegDef_t*)RCC_BASEADDR)


/*
 * * * CLOCK ENABLE MACROS
 */

//GPIOs
#define GPIOA_CLK_ENABLE()			RCC->APB2ENR |= ( 1 << 2 )
#define GPIOB_CLK_ENABLE()			RCC->APB2ENR |= ( 1 << 3 )
#define GPIOC_CLK_ENABLE()			RCC->APB2ENR |= ( 1 << 4 )
#define GPIOD_CLK_ENABLE()			RCC->APB2ENR |= ( 1 << 5 )
#define GPIOE_CLK_ENABLE()			RCC->APB2ENR |= ( 1 << 6 )
#define GPIOF_CLK_ENABLE()			RCC->APB2ENR |= ( 1 << 7 )
#define GPIOG_CLK_ENABLE()			RCC->APB2ENR |= ( 1 << 8 )

//USART & UART
#define USART1_CLK_ENABLE()			RCC->APB2ENR |= ( 1 << 14 )
#define USART2_CLK_ENABLE()			RCC->APB1ENR |= ( 1 << 17 )
#define USART3_CLK_ENABLE()			RCC->APB1ENR |= ( 1 << 18 )
#define UART4_CLK_ENABLE()			RCC->APB1ENR |= ( 1 << 19 )
#define UART5_CLK_ENABLE()			RCC->APB1ENR |= ( 1 << 20 )

//SPI
#define SPI1_CLK_ENABLE()			RCC->APB2ENR |= ( 1 << 12 )
#define SPI2_CLK_ENABLE()			RCC->APB1ENR |= ( 1 << 14 )
#define SPI3_CLK_ENABLE()			RCC->APB1ENR |= ( 1 << 15 )
//I2C
#define I2C1_CLK_ENABLE()			RCC->APB1ENR |= ( 1 << 21 )
#define I2C2_CLK_ENABLE()			RCC->APB1ENR |= ( 1 << 22 )


/*
 * * * CLOCK DISABLE MACROS
 */

//GPIOs
#define GPIOA_CLK_DISABLE()			RCC->APB2ENR &= ~( 1 << 2 )
#define GPIOB_CLK_DISABLE()			RCC->APB2ENR &= ~( 1 << 3 )
#define GPIOC_CLK_DISABLE()			RCC->APB2ENR &= ~( 1 << 4 )
#define GPIOD_CLK_DISABLE()			RCC->APB2ENR &= ~( 1 << 5 )
#define GPIOE_CLK_DISABLE()			RCC->APB2ENR &= ~( 1 << 6 )
#define GPIOF_CLK_DISABLE()			RCC->APB2ENR &= ~( 1 << 7 )
#define GPIOG_CLK_DISABLE()			RCC->APB2ENR &= ~( 1 << 8 )

//USART & UART
#define USART1_CLK_DISABLE()		RCC->APB2ENR &= ~( 1 << 14 )
#define USART2_CLK_DISABLE()		RCC->APB1ENR &= ~( 1 << 17 )
#define USART3_CLK_DISABLE()		RCC->APB1ENR &= ~( 1 << 18 )
#define UART4_CLK_DISABLE()			RCC->APB1ENR &= ~( 1 << 19 )
#define UART5_CLK_DISABLE()			RCC->APB1ENR &= ~( 1 << 20 )

//SPI
#define SPI1_CLK_DISABLE()			RCC->APB2ENR &= ~( 1 << 12 )
#define SPI2_CLK_DISABLE()			RCC->APB1ENR &= ~( 1 << 14 )
#define SPI3_CLK_DISABLE()			RCC->APB1ENR &= ~( 1 << 15 )
//I2C
#define I2C1_CLK_DISABLE()			RCC->APB1ENR &= ~( 1 << 21 )
#define I2C2_CLK_DISABLE()			RCC->APB1ENR &= ~( 1 << 22 )


/*
 * * * PERIPHERALS RESET REGISTER macros
 */

#define GPIOA_REG_RESET()			do{(RCC->APB2RSTR |= ( 1 << 2 )); (RCC->APB2RSTR |= ( 1 << 2 ));} while(0)
#define GPIOB_REG_RESET()			do{(RCC->APB2RSTR |= ( 1 << 3 )); (RCC->APB2RSTR |= ( 1 << 3 ));} while(0)
#define GPIOC_REG_RESET()			do{(RCC->APB2RSTR |= ( 1 << 4 )); (RCC->APB2RSTR |= ( 1 << 4 ));} while(0)
#define GPIOD_REG_RESET()			do{(RCC->APB2RSTR |= ( 1 << 5 )); (RCC->APB2RSTR |= ( 1 << 5 ));} while(0)
#define GPIOE_REG_RESET()			do{(RCC->APB2RSTR |= ( 1 << 6 )); (RCC->APB2RSTR |= ( 1 << 6 ));} while(0)
#define GPIOF_REG_RESET()			do{(RCC->APB2RSTR |= ( 1 << 7 )); (RCC->APB2RSTR |= ( 1 << 7 ));} while(0)
#define GPIOG_REG_RESET()			do{(RCC->APB2RSTR |= ( 1 << 8 )); (RCC->APB2RSTR |= ( 1 << 8 ));} while(0)



/*
 * Generic macros
 */
#define ENABLE 			1
#define DISABLE			0
#define SET				ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET

#endif /* INC_STM32F103XX_H_ */
