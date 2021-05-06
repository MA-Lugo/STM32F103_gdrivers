/*
 * stm32f103_i2c_driver.c
 *
 *  Created on: 6 mar. 2021
 *      Author: Mario
 */

#include "stm32f103_i2c_driver.h"

static void I2C_GenStartCondition(I2C_RegDef_t *pI2Cx);


static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);

static void I2C_MasterHandleTXE_IT(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNE_IT(I2C_Handle_t *pI2CHandle);

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};

/**********************************************************
 * @fn				- I2C_CLK_Control
 * @brief			- This function enable or disable the peripheral clock
 * 					  for the given I2Cx
 *
 * @param[in]		- Base addres of the I2C port
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/

void I2C_CLK_Control(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_CLK_ENABLE();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_CLK_ENABLE();
		}

	}

	else
	{
		if (pI2Cx == I2C1)
		{
			I2C1_CLK_DISABLE();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_CLK_DISABLE();
		}

	}

}

/**********************************************************
 * @fn				-
 * @brief			- This function gets the frequency of the
 * 					  APB1 bus.
 *
 * @param[in]		- Clock Source.
 *
 * @return			- Frequency of the APB1.
 *
 * @note			- none
 *********************************************************/
uint32_t RCC_GetPCLK1Value(uint32_t CLK_Source)
{
	uint32_t pclk1;
	uint8_t temp,ahbp,apb1p;

	//For AHB
	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	//For APB1
	temp = ((RCC->CFGR >> 8) & 0x7);
	if(temp < 4)
	{
		apb1p = 1;
	}
	else
	{
		apb1p = APB1_PreScaler[temp-4];
	}

	pclk1 = (CLK_Source/ahbp)/ apb1p;

	return pclk1;
}

/**********************************************************
 * @fn				- I2C_Init
 * @brief			- This function Initialize the given I2C
 * 					  Port
 *
 *
 * @param[in]		- Base addres of the I2Cx
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg;

	//Configure AckControl
	tempreg = pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 = tempreg;
	//Configure the FREQ field of CR2
	tempreg = RCC_GetPCLK1Value(pI2CHandle->SYSTEM_CLK) / 1000000U;
	pI2CHandle->pI2Cx->CR2 |= (tempreg & 0x3F);
	//Configure the device own address
	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviveAddres << 1);
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;
	//Configurate the CCR register
	uint16_t ccr_value;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		ccr_value = (RCC_GetPCLK1Value(pI2CHandle->SYSTEM_CLK) / (2*pI2CHandle->I2C_Config.I2C_SCLSpeed) );
		tempreg = (ccr_value & 0xFFF);
	}
	else
	{
		tempreg = (1 <<15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle <<14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value(pI2CHandle->SYSTEM_CLK) / (3*pI2CHandle->I2C_Config.I2C_SCLSpeed) );

		}
		else
		{
			ccr_value = (RCC_GetPCLK1Value(pI2CHandle->SYSTEM_CLK) / (25*pI2CHandle->I2C_Config.I2C_SCLSpeed) );

		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE CONFIGURATION
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//STANDAR MODE
		tempreg = (RCC_GetPCLK1Value(pI2CHandle->SYSTEM_CLK) / 1000000U ) + 1;
	}
	else
	{
		//FAST MODE
		tempreg = ( (RCC_GetPCLK1Value(pI2CHandle->SYSTEM_CLK)*300) / 1000000000U ) + 1;

	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}


/**********************************************************
 * @fn				- I2C_DeInit
 * @brief			- This function De-Init the given I2C Port
 *
 *
 * @param[in]		- Base addres of the I2Cx
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{

	if (pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if (pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}

}

/**********************************************************
 * @fn				- I2C_PeripheralControl
 * @brief			- This function enable or disable the peripheral
 *
 *
 * @param[in]		- Base addres of the I2Cx
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @note			-
 *********************************************************/
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}


/**********************************************************
 * @fn				- I2C_MASTER_SendData
 * @brief			- This function send data from the
 * 					  given I2C port in master mode
 *
 *
 * @param[in]		- I2C  handle structure
 * @param[in]		- user Tx buffer pointer
 * @param[in]		- length of data (number of bytes)
 * @param[in]		- Slave Address
 * @param[in]		- Repeat Restart Condition EnOrDis
 *
 *
 * @return			- none
 *
 * @note			- This is blocking call
 *********************************************************/

void I2C_MASTER_SendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t I2C_RS_EnOrDis)
{
	//1. Generate the start condition
	I2C_GenStartCondition(pI2CHandle->pI2Cx);
	//2. Confirm that start condition is completed by checking the SB Flag in the SR1
	//	 Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while( ! (pI2CHandle->pI2Cx->SR1 & I2C_FLAG_SB) );
	//3. Set the address of the slave with the r/w bit set to 0 (8bit total)
	uint8_t addr = 0;
	addr = ( (SlaveAddr <<1));
	addr &= ~(1);
	pI2CHandle->pI2Cx->OAR1 = (addr);
	//4. Comfirm that the address phase is completed by the ADDR Flag in the SR1
	while( ! (pI2CHandle->pI2Cx->SR1 & I2C_FLAG_ADDR) );
	//5. Clear the ADDR Flag according to its software sequence
	//	 Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle);
	//6. Send data until Len becomes zero
	while(Len > 0)
	{
		while( ! (pI2CHandle->pI2Cx->SR1 & I2C_FLAG_TXE) );
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;

	}
	//7. When Len becomes zero wait for TXE=1 and BTF = 1 before generating the STOP condition
	//	 Note: TXE=1, BTF=1 means that both SR and DR are empty and next transmission should begin
	// 	 when BTF=1 SCL will be stretched (pulled to LOW)
	while( ! (pI2CHandle->pI2Cx->SR1 & I2C_FLAG_TXE) );
	while( ! (pI2CHandle->pI2Cx->SR1 & I2C_FLAG_BTF) );

	//8. Generate STOP condition and master need not to wait for ge completion of the condition.
	//	 Note:	Generating STOP, automatically clears the BTF

	if(I2C_RS_EnOrDis == I2C_RS_DISABLE)
	{
		I2C_GenStopCondition(pI2CHandle->pI2Cx);
	}

}

/**********************************************************
 * @fn				- I2C_MASTER_ReceiveData
 * @brief			- This function send data from the
 * 					  given I2C port in master mode
 *
 *
 * @param[in]		- I2C  handle structure
 * @param[in]		- User Rx buffer pointer
 * @param[in]		- Length of data (number of bytes)
 * @param[in]		- Slave Address
 * @param[in]		- Repeat Restart Condition EnOrDis
 *
 *
 * @return			- none
 *
 * @note			- This is blocking call
 *********************************************************/
void I2C_MASTER_ReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t I2C_RS_EnOrDis)
{

	//1. Generate the START condition
	I2C_GenStartCondition(pI2CHandle->pI2Cx);
	//2. Confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while( ! (pI2CHandle->pI2Cx->SR1 & I2C_FLAG_SB) );
	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
	uint8_t addr = 0;
	addr = ( (SlaveAddr <<1) );
	addr |= 1;
	pI2CHandle->pI2Cx->DR = (addr);
	//4. wait until address phase is completed by checking the ADDR flag in teh SR1
	while( ! (pI2CHandle->pI2Cx->SR1 & I2C_FLAG_ADDR) );


	//procedure to read only 1 byte from slave
	if(Len == 1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);
		//wait until  RXNE becomes 1
		while( ! (pI2CHandle->pI2Cx->SR1 & I2C_FLAG_RXNE) );
		//generate STOP condition
		if(I2C_RS_EnOrDis == I2C_RS_DISABLE)
		{
			I2C_GenStopCondition(pI2CHandle->pI2Cx);
		}
		//read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

	}


    //procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read the data until Len becomes zero
		for ( uint32_t i = Len ; i > 0 ; i--)
		{
			//wait until RXNE becomes 1
			while( ! (pI2CHandle->pI2Cx->SR1 & I2C_FLAG_RXNE) );

			if(i == 2) //if last 2 bytes are remaining
			{
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//generate STOP condition
				if(I2C_RS_EnOrDis == I2C_RS_DISABLE)
				{
					I2C_GenStopCondition(pI2CHandle->pI2Cx);
				}

			}

			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;
		}

	}
	//re-enable ACKing
	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);

	}

}

/*
 * Others
 */


/**********************************************************
 * @fn				- I2C_ManageAcking
 * @brief			- This function Enable or Disable Acking
 * 					  configuration.
 *
 * @param[in]		- Base addres of the I2C port
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis)
{
	if(EnOrDis == I2C_ACK_ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}

}


/**********************************************************
 * @fn				- I2C_SLAVE_ManageCallbackEvents
 * @brief			- This function enable or disable
 * 					  callback events.
 *
 * @param[in]		- Base addres of the I2C port
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/
void I2C_SLAVE_ManageCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis)
{
	if (EnOrDis == ENABLE)
	{
		pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	else
	{
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
	}
}


/**********************************************************
 * @fn				- I2C_GenStartCondition
 * @brief			- This function generates a start condition
 *
 * @param[in]		- Base addres of the I2C port
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/
static void I2C_GenStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}


/**********************************************************
 * @fn				- I2C_GenStopCondition
 * @brief			- This function generates a stop condition
 *
 * @param[in]		- Base addres of the I2C port
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/
void I2C_GenStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}


/**********************************************************
 * @fn				- I2C_ClearADDRFlag
 * @brief			- This function clear the ADDR Flag
 *
 * @param[in]		- I2C handle structure
 *
 * @return			- none
 *
 * @note			- A private function
 *********************************************************/
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummyread;

	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{//MASTER MODE
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxLen == 1)
			{
				//First disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//Clear the ADDR Flag (read SR1 and SR2)
				dummyread = pI2CHandle->pI2Cx->CR1;
				dummyread = pI2CHandle->pI2Cx->CR2;
				(void)dummyread;

			}
		}
		else
		{
			//Clear the ADDR Flag (read SR1 and SR2)
			dummyread = pI2CHandle->pI2Cx->CR1;
			dummyread = pI2CHandle->pI2Cx->CR2;
			(void)dummyread;
		}

	}
	else
	{///SLAVE MODE
		//Clear the ADDR Flag (read SR1 and SR2)
		dummyread = pI2CHandle->pI2Cx->CR1;
		dummyread = pI2CHandle->pI2Cx->CR2;
		(void)dummyread;
	}



}


/**********************************************************
 * @fn				- I2C_IRQInterrupt_Config
 * @brief			- This function configure the given IRQ Number
 *
 *
 * @param[in]		- IRQ_NO macros
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @note			-
 *********************************************************/
void I2C_IRQInterrupt_Config(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
		{
			if (IRQNumber <= 31)						//ISER0 register
			{
				*NVIC_ISER0 |= (1 << IRQNumber);
			}
			else if (IRQNumber > 31 &&  IRQNumber < 64)//ISER1 register
			{
				*NVIC_ISER1 |= (1 << IRQNumber % 32);
			}
			else if (IRQNumber >= 64 && IRQNumber < 96)//ISER2 register
			{
				*NVIC_ISER2 |= (1 << IRQNumber % 64);
			}

		}

		else
		{
			if (IRQNumber <= 31)						//ICER0 register
			{
				*NVIC_ICER0 |= (1 << IRQNumber);
			}
			else if (IRQNumber > 31 &&  IRQNumber < 64)//ISER1 register
			{
				*NVIC_ICER1 |= (1 << IRQNumber % 32);
			}
			else if (IRQNumber >= 64 && IRQNumber < 96)//ISER2 register
			{
				*NVIC_ICER2 |= (1 << IRQNumber % 64);
			}
		}
}

/**********************************************************
 * @fn				- I2C_IRQPiority_Config
 * @brief			- This function set the priority of the
 * 					- given IRQ number
 *
 * @param[in]		- IRQ number
 * @param[in]		- Priority value
 *
 * @return			- none
 *
 * @note			- none
 **************************************************************/
void I2C_IRQPiority_Config(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENT);

	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

/**********************************************************
 * @fn				- I2C_MASTER_SendData_IT
 * @brief			- This function send data from the
 * 						given I2C Handle with interrupts
 *
 * @param[in]		- I2C Handke structure
 * @param[in]		- Tx Buffer pointer
 * @param[in]		- Len
 * @param[in]		- Slave Address
 * @param[in]		- Repeat Start EnOrDis
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/
uint8_t I2C_MASTER_SendData_IT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t I2C_RS_EnOrDis)
{
	uint8_t state = pI2CHandle->TxRxState;

	if( (state != I2C_BUSY_IN_TX) && (state != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddress = SlaveAddr;
		pI2CHandle->Sr = I2C_RS_EnOrDis;

		//Implement code to Generate START Condition
		I2C_GenStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return state;
}

/**********************************************************
 * @fn				- I2C_MASTER_ReceiveData_IT
 * @brief			- This function receive data from the
 * 						given I2C Handle with interrupts
 *
 * @param[in]		- I2C Handle structure
 * @param[in]		- Rx Buffer pointer
 * @param[in]		- Len
 * @param[in]		- Slave Address
 * @param[in]		- Repeat Start EnOrDis
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/
uint8_t I2C_MASTER_ReceiveData_IT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t I2C_RS_EnOrDis)
{

	uint8_t state = pI2CHandle->TxRxState;

	if( (state != I2C_BUSY_IN_TX) && (state != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddress = SlaveAddr;
		pI2CHandle->Sr = I2C_RS_EnOrDis;

		//Implement code to Generate START Condition
		I2C_GenStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return state;
}

static void I2C_MasterHandleTXE_IT(I2C_Handle_t *pI2CHandle)
{
	//DO THE DATA TRANSMISSION
	if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
	{
		if(pI2CHandle->TxLen > 0)
		{

			//1. Load the data in to DR
			pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

			//2. Decrement the TXLen
			pI2CHandle->TxLen --;

			//3. Increment the buffer address
			pI2CHandle->pTxBuffer++;


		}
	}
}
static void I2C_MasterHandleRXNE_IT(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->RxLen == 1)
	{
		pI2CHandle->pRxBuffer = (uint8_t*)pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen --;

	}
	if(pI2CHandle->RxLen > 1)
	{
		if (pI2CHandle->RxLen == 2)
		{
			//Clear the ack bit
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}

		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;


		if (pI2CHandle->RxLen == 0)
		{
			//Close the I2C data reception and notify the aplication

			//1. Generate the stop condition
			if(pI2CHandle->Sr == I2C_RS_DISABLE)
			{
				I2C_GenStopCondition(pI2CHandle->pI2Cx);
			}

			//2. Close the I2C rx
			I2C_Close_Reception(pI2CHandle);
			//3. Notify the application
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_RX_CMPLT);

		}

	}
}

/**********************************************************
 * @fn				- I2C_SLAVE_SendData
 * @brief			- This function send data from the
 * 					  given I2C port in slave mode
 *
 *
 * @param[in]		- I2C  base addres
 * @param[in]		- Data to send


 * @return			- none
 *
 * @note			- none
 *********************************************************/
void I2C_SLAVE_SendData(I2C_RegDef_t *pI2Cx, uint8_t data )
{
	pI2Cx->DR = data;
}

/**********************************************************
 * @fn				- I2C_SLAVE_ReceiveData
 * @brief			- This function receive data from the
 * 					  given I2C port in slave mode
 *
 *
 * @param[in]		- I2C  base addres


 * @return			- Data received
 *
 * @note			- none
 *********************************************************/
uint8_t I2C_SLAVE_ReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->DR;
}



/**********************************************************
 * @fn				- I2C_EV_IRQHandling
 * @brief			- This function handle IRQ Events
 *
 * @param[in]		- I2C Handle structure

 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	//Interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;
	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if(temp1 && temp3)
	{
		//SB Start bit (Master mode) Flag is set


		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			uint8_t addr = 0;
			addr = (pI2CHandle->DevAddress << 1);
			addr &= ~(1);
			pI2CHandle->pI2Cx->DR = (addr);
		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			uint8_t addr = 0;
			addr = (pI2CHandle->DevAddress << 1);
			addr |= 1;
			pI2CHandle->pI2Cx->DR = (addr);

		}
	}


	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	if(temp1 && temp3)
	{
		//ADDR Flag is set

		I2C_ClearADDRFlag(pI2CHandle);
	}


	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3)
	{
		//BTF Flag is set

		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//make sure that the TXE is also set
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TxE))
			{//BTF, TXE = 1

				if(pI2CHandle->TxLen == 0){

					if(pI2CHandle->Sr == I2C_RS_DISABLE){
						I2C_GenStopCondition(pI2CHandle->pI2Cx);
					}

					//Reset all the elements of the handle structure
					I2C_Close_Transmission(pI2CHandle);
					//Notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_TX_CMPLT);

				}



			}
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{

		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	if(temp1 && temp3)
	{
		//STOPF Flag is set

		//Clear the STOPF		1) read SR1 (done) and wirte CR1
		pI2CHandle->pI2Cx->CR1 |= 0x0000; //write without modify the reg

		//notify the application that the STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_STOP);


	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TxE);
	//5. Handle For interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		//CHECK DEVICE MODE (MASTER)
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			//TXF Flag is set
			I2C_MasterHandleTXE_IT(pI2CHandle);

		}


	}
	else
	{
		//SLAVE MODE

		//make shure that the slave is really in transmitter mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
		{
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_DATA_REQ);
		}

	}


	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RxNE);
	//6. Handle For interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		//RXNE Flag is set
		//CHECK DEVICE MODE (MASTER)
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			//DO THE DATA RECEPTION

			if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNE_IT(pI2CHandle);
			}

		}

		else
		{
			//SLAVE MODE
			//make shure that the slave is really in receiver mode
					if( !(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)) )
					{
						I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_DATA_RCV);
					}
		}
	}

}

/**********************************************************
 * @fn				- I2C_ER_IRQHandling
 * @brief			- This function handle IRQ Errors
 *
 * @param[in]		- I2C Handle structure

 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);


		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);

	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}

void I2C_Close_Reception(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Contro Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);;

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_RS_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}


}
void I2C_Close_Transmission(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;




}

__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEV)
{
	//This is a weak implementation. the application may override this funtcion.
}
