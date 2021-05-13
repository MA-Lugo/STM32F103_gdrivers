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

	uint8_t			*pTxBuffer; 	/* !< To store the app. TX buffer address >*/
	uint8_t			*pRxBuffer;		/* !< To store the app. RX buffer address >*/
	uint32_t		TxLen;			/* !< To store the TX len >*/
	uint32_t		RxLen;			/* !< To store the Rx len >*/
	uint8_t			TxRxState;		/* !< To store the communication state >*/
	uint8_t			DevAddress;		/* !< To store the slave/device address >*/
	uint32_t 		RxSize;			/* !< To store the RX Size >*/
	uint8_t			Sr;				/* !< To store the repeat start value >*/
}I2C_Handle_t;

/*
 * SPI Application States
 */
#define I2C_READY					0
#define I2C_BUSY_IN_TX				1
#define I2C_BUSY_IN_RX				2


/*
 * POSSIBLE I2C Applications evens
 */

#define I2C_EVENT_TX_CMPLT			0
#define I2C_EVENT_RX_CMPLT			1
#define I2C_EVENT_STOP				2

#define I2C_ERROR_BERR 				3
#define I2C_ERROR_ARLO 				4
#define I2C_ERROR_AF    			5
#define I2C_ERROR_OVR   			6
#define I2C_ERROR_TIMEOUT 			7

#define I2C_EVENT_DATA_REQ			8
#define I2C_EVENT_DATA_RCV			9

/*
 * I2C REPEAT START CONDITION macros
 */
#define I2C_RS_ENABLE		SET
#define I2C_RS_DISABLE		RESET

/*
 * I2C Related Status Flags Definitions
 */
#define I2C_FLAG_TXE				(1 << I2C_SR1_TxE)
#define I2C_FLAG_RXNE				(1 << I2C_SR1_RxNE)
#define I2C_FLAG_SB					(1 << I2C_SR1_SB)
#define I2C_FLAG_OVR				(1 << I2C_SR1_OVR)
#define I2C_FLAG_AF					(1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO				(1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR				(1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF				(1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10				(1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF				(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR				(1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT			(1 << I2C_SR1_TIMEOUT)



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


void I2C_Init(I2C_Handle_t *pI2CHandle);

void I2C_DeInit(I2C_RegDef_t *pI2Cx);

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis);

/*
 * Send and Receive Data
 */
void I2C_MASTER_SendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t I2C_RS_EnOrDis);

void I2C_MASTER_ReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t I2C_RS_EnOrDis);

void I2C_Close_Transmission(I2C_Handle_t *pI2CHandle);
void I2C_Close_Reception(I2C_Handle_t *pI2CHandle);


void I2C_SLAVE_SendData(I2C_RegDef_t *pI2Cx, uint8_t data );
uint8_t I2C_SLAVE_ReceiveData(I2C_RegDef_t *pI2Cx);


/*
 * IRQ configuration and ISR handling
 */

void I2C_IRQInterrupt_Config(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPiority_Config(uint8_t IRQNumber, uint32_t IRQPriority);

/*
 * Send and Receive Data with Interrupts
 */
uint8_t I2C_MASTER_SendData_IT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t I2C_RS_EnOrDis);
uint8_t I2C_MASTER_ReceiveData_IT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t I2C_RS_EnOrDis);

/*
 * IRQ Handling
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);




/*
 * Others
 */
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis);
void I2C_GenStopCondition(I2C_RegDef_t *pI2Cx);

void I2C_SLAVE_ManageCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis);


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEV);


#endif /* INC_STM32F103_I2C_DRIVER_H_ */
