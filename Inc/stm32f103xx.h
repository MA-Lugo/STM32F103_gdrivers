/*
 * stm32f103xx.h
 *
 *  Created on: Jan 14, 2021
 *      Author: Mario
 */
#include <stdint.h>
#include <stddef.h>

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

/*
 * ARM Cortex M3 NVIC ISERx register Addresses
 */

#define NVIC_ISER0				( (volatile uint32_t*)0xE000E100 )
#define NVIC_ISER1				( (volatile uint32_t*)0xE000E104 )
#define NVIC_ISER2				( (volatile uint32_t*)0xE000E108 )
#define NVIC_ISER3				( (volatile uint32_t*)0xE000E10C )
#define NVIC_ISER4				( (volatile uint32_t*)0xE000E110 )
#define NVIC_ISER5				( (volatile uint32_t*)0xE000E114 )
#define NVIC_ISER6				( (volatile uint32_t*)0xE000E118 )
#define NVIC_ISER7				( (volatile uint32_t*)0xE000E11C )

/*
 * ARM Cortex M3 NVIC ICERx register Addresses
 */

#define NVIC_ICER0				( (volatile uint32_t*)0xE000E180 )
#define NVIC_ICER1				( (volatile uint32_t*)0xE000E184 )
#define NVIC_ICER2				( (volatile uint32_t*)0xE000E188 )
#define NVIC_ICER3				( (volatile uint32_t*)0xE000E18C )
#define NVIC_ICER4				( (volatile uint32_t*)0xE000E190 )
#define NVIC_ICER5				( (volatile uint32_t*)0xE000E194 )
#define NVIC_ICER6				( (volatile uint32_t*)0xE000E198 )
#define NVIC_ICER7				( (volatile uint32_t*)0xE000E19C )


#define NVIC_PR_BASEADDR 		( (volatile uint32_t*)0xE000E400 )

#define NO_PR_BITS_IMPLEMENT		4
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

typedef struct
{
	volatile uint32_t IMR;			//Interrupt mask register				OFSET:0x00
	volatile uint32_t EMR;			//Event mask register					OFSET:0x04
	volatile uint32_t RTSR;			//Rising trigger selection register		OFSET:0x08
	volatile uint32_t FTSR;			//Falling trigger selection register	OFSET:0x0C
	volatile uint32_t SWIER;		//Software interrupt event register		OFSET:0x10
	volatile uint32_t PR;			//Pending register	OFSET:0x14
}EXTI_RegDef_t;

typedef struct
{
	volatile uint32_t EVCR;			//Event control register							OFSET:0x00
	volatile uint32_t MAPR;			//AF remap and debug I/O configuration register		OFSET:0x04
	volatile uint32_t EXTICR[4];	//External interrupt configuration register 1-4		OFSET:0x08
	volatile uint32_t RESERVED;		//External interrupt configuration register 4		OFSET:0x18
	volatile uint32_t MAPR2;		//AF remap and debug I/O configuration register2	OFSET:0x1C

}AFIO_RegDef_t;

typedef struct
{
	volatile uint32_t CR1;			//SPI control register 1							OFSET:0x00
	volatile uint32_t CR2;			//SPI control register 2							OFSET:0x04
	volatile uint32_t SR;			//SPI status register								OFSET:0x08
	volatile uint32_t DR;			//SPI data register									OFSET:0x0C
	volatile uint32_t CRCPR;		//SPI CRC polynomial register						OFSET:0x10
	volatile uint32_t RXCRCR;		//SPI RX CRC register								OFSET:0x14
	volatile uint32_t TXCRCR;		//SPI TX CRC register								OFSET:0x18
	volatile uint32_t I2SCFGR;		//SPI_I2S configuration register					OFSET:0x1C
	volatile uint32_t I2SPR;		//SPI_I2S prescaler register						OFSET:0x20


}SPI_RegDef_t;

typedef struct
{
	volatile uint32_t CR1;			//I2C control register 1							OFSET:0x00
	volatile uint32_t CR2;			//I2C control register 2							OFSET:0x04
	volatile uint32_t OAR1;			//I2C Own address register 1						OFSET:0x08
	volatile uint32_t OAR2;			//I2C Own address register 2						OFSET:0x0C
	volatile uint32_t DR;			//I2C Data register									OFSET:0x10
	volatile uint32_t SR1;			//I2C Status register 1								OFSET:0x14
	volatile uint32_t SR2;			//I2C Status register 2								OFSET:0x18
	volatile uint32_t CCR;			//I2C Clock control register						OFSET:0x1C
	volatile uint32_t TRISE;		//I2C TRISE register								OFSET:0x20


}I2C_RegDef_t;



typedef struct
{
	volatile uint32_t	USART_SR;	//Status register									OFSET:0x00
	volatile uint32_t	USART_DR;	//Data register										OFSET:0x04
	volatile uint32_t	USART_BRR;	//Baud rate register								OFSET:0x08
	volatile uint32_t	USART_CR1;	//Control register 1								OFSET:0x0C
	volatile uint32_t	USART_CR2;	//Control register 2								OFSET:0x10
	volatile uint32_t	USART_CR3;	//Control register 3								OFSET:0x14
	volatile uint32_t	USART_GTPR;	//Guard time and prescaler register					OFSET:0x18
}USART_RegDef_t;



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

#define SPI1						((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2						((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3						((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1						((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2						((I2C_RegDef_t*)I2C2_BASEADDR)

#define USART1						((USART_RegDef_t*)USART1_BASEADDR)
#define USART2						((USART_RegDef_t*)USART2_BASEADDR)
#define USART3						((USART_RegDef_t*)USART3_BASEADDR)
#define UART4						((USART_RegDef_t*)UART4_BASEADDR)
#define UART5						((USART_RegDef_t*)UART5_BASEADDR)


#define RCC							((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI						((EXTI_RegDef_t*)EXTI_BASEADDR)

#define AFIO						((AFIO_RegDef_t*)AFIO_BASEADDR)
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

//AFIO
#define AFIO_CLK_ENABLE()			RCC->APB2ENR |= ( 1 << 0 )

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

#define GPIOA_REG_RESET()			do{(RCC->APB2RSTR |= ( 1 << 2 )); (RCC->APB2RSTR &= ~( 1 << 2 ));} while(0)
#define GPIOB_REG_RESET()			do{(RCC->APB2RSTR |= ( 1 << 3 )); (RCC->APB2RSTR &= ~( 1 << 3 ));} while(0)
#define GPIOC_REG_RESET()			do{(RCC->APB2RSTR |= ( 1 << 4 )); (RCC->APB2RSTR &= ~( 1 << 4 ));} while(0)
#define GPIOD_REG_RESET()			do{(RCC->APB2RSTR |= ( 1 << 5 )); (RCC->APB2RSTR &= ~( 1 << 5 ));} while(0)
#define GPIOE_REG_RESET()			do{(RCC->APB2RSTR |= ( 1 << 6 )); (RCC->APB2RSTR &= ~( 1 << 6 ));} while(0)
#define GPIOF_REG_RESET()			do{(RCC->APB2RSTR |= ( 1 << 7 )); (RCC->APB2RSTR &= ~( 1 << 7 ));} while(0)
#define GPIOG_REG_RESET()			do{(RCC->APB2RSTR |= ( 1 << 8 )); (RCC->APB2RSTR &= ~( 1 << 8 ));} while(0)


#define SPI1_REG_RESET()			do{(RCC->APB2RSTR |= ( 1 << 12 )); (RCC->APB2RSTR &= ~( 1 << 12 ));} while(0)
#define SPI2_REG_RESET()			do{(RCC->APB1RSTR |= ( 1 << 14 )); (RCC->APB1RSTR &= ~( 1 << 14 ));} while(0)
#define SPI3_REG_RESET()			do{(RCC->APB1RSTR |= ( 1 << 15 )); (RCC->APB1RSTR &= ~( 1 << 15 ));} while(0)

#define I2C1_REG_RESET()			do{(RCC->APB1RSTR |= ( 1 << 21 )); (RCC->APB1RSTR &= ~( 1 << 21 ));} while(0)
#define I2C2_REG_RESET()			do{(RCC->APB1RSTR |= ( 1 << 22 )); (RCC->APB1RSTR &= ~( 1 << 22 ));} while(0)

#define USART1_REG_RESET()			do{ RCC->APB2RSTR |= ( 1 << 14); RCC->APB2RSTR &= ~( 1 << 14); } while(0)
#define USART2_REG_RESET()			do{ RCC->APB1RSTR |= ( 1 << 17); RCC->APB1RSTR &= ~( 1 << 17); } while(0)
#define USART3_REG_RESET()			do{ RCC->APB1RSTR |= ( 1 << 18); RCC->APB1RSTR &= ~( 1 << 18); } while(0)
#define UART4_REG_RESET()			do{ RCC->APB1RSTR |= ( 1 << 19); RCC->APB1RSTR &= ~( 1 << 19); } while(0)
#define UART5_REG_RESET()			do{ RCC->APB1RSTR |= ( 1 << 20); RCC->APB1RSTR &= ~( 1 << 20); } while(0)



#define GPIO_BASEADDR_TO_CODE(x)	  (	(x == GPIOA) ? 0:\
										(x == GPIOB) ? 1:\
										(x == GPIOC) ? 2:\
										(x == GPIOD) ? 3:\
										(x == GPIOE) ? 4:\
										(x == GPIOF) ? 5:\
										(x == GPIOG) ? 6:0  )



/*
 * IRQ (Interrupt Request) Numbers for STM32F103C8T6
 */
#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI9_5				23
#define IRQ_NO_EXTI15_20			40

#define IRQ_NO_SPI1					35
#define IRQ_NO_SPI2					36
#define IRQ_NO_SPI3					51

#define IRQ_NO_I2C1_EV				31
#define IRQ_NO_I2C1_ER				32
#define IRQ_NO_I2C2_EV				33
#define IRQ_NO_I2C2_ER				34





/*
 * Generic macros
 */
#define ENABLE 			1
#define DISABLE			0
#define SET				ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET



/********************************************************************************
 * Bit position definitions of SPI peripheral
 */

#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7


#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF		5
#define SPI_SR_OVR		6
#define SPI_SR_BSY		7

/********************************************************************************
 * Bit position definitions of I2C peripheral
 */
#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWRST		15

#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_DMAEN		11
#define I2C_CR2_LAST		12

#define I2C_OAR1_ADD0		0
#define I2C_OAR1_ADD_7_1	1
#define I2C_OAR1_ADD_9_8	8
#define I2C_OAR1_ADDMODE	15

#define I2C_OAR2_ENDUAL		0
#define I2C_OAR2_ADD2		1

#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RxNE		6
#define I2C_SR1_TxE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PECERR		12
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_SMBALERT	15

#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_SMBDEFAULT	5
#define I2C_SR2_SMBHOST		6
#define I2C_SR2_DUALF		7
#define I2C_SR2_PEC			8


/********************************************************************************
 * Bit position definitions of USART peripheral
 */

#define USART_SR_PE			0
#define USART_SR_FE			1
#define USART_SR_NE			2
#define USART_SR_ORE		3
#define USART_SR_IDLE		4
#define USART_SR_RXNE		5
#define USART_SR_TC			6
#define USART_SR_TXE		7
#define USART_SR_LBD		8
#define USART_SR_CTS		9


#define USART_CR1_SBK		0
#define USART_CR1_RWU		1
#define USART_CR1_RE		2
#define USART_CR1_TE	 	3
#define USART_CR1_IDLEIE	4
#define USART_CR1_RXNEIE	5
#define USART_CR1_TCIE		6
#define USART_CR1_TXEIE		7
#define USART_CR1_PEIE		8
#define USART_CR1_PS		9
#define USART_CR1_PCE		10
#define USART_CR1_WAKE		11
#define USART_CR1_M			12
#define USART_CR1_UE		13


#define USART_CR2_ADD		0
#define USART_CR2_LBDL		5
#define USART_CR2_LBDIE		6
#define USART_CR2_LBCL		8
#define USART_CR2_CPHA		9
#define USART_CR2_CPOL		10
#define USART_CR2_CLKEN		11
#define USART_CR2_STOP		12
#define USART_CR2_LINEN		14

#define USART_CR3_EIE		0
#define USART_CR3_IREN		1
#define USART_CR3_IRLP		2
#define USART_CR3_HDSEL	 	3
#define USART_CR3_NACK		4
#define USART_CR3_SCEN		5
#define USART_CR3_DMAR		6
#define USART_CR3_DMAT		7
#define USART_CR3_RTSE		8
#define USART_CR3_CTSE		9
#define USART_CR3_CTSIE		10







#endif /* INC_STM32F103XX_H_ */
