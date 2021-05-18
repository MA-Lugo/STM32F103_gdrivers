/*
 * stm32f103_usart_driver.c
 *
 *  Created on: 7 may. 2021
 *      Author: Mario
 */



#include "stm32f103_usart_driver.h"



/**********************************************************
 * @fn				- USART_CLK_Control
 * @brief			- This function enable or disable the peripheral clock
 * 					  for the given USARTx/UARTx
 *
 * @param[in]		- Base addres of the USART/UART port
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/
void USART_CLK_Control(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
		{
			if (pUSARTx == USART1)
			{
				USART1_CLK_ENABLE();
			}
			else if (pUSARTx == USART2)
			{
				USART2_CLK_ENABLE();
			}
			else if (pUSARTx == USART3)
			{
				USART3_CLK_ENABLE();
			}
			else if (pUSARTx == UART4)
			{
				UART4_CLK_ENABLE();
			}
			else if (pUSARTx == UART5)
			{
				UART5_CLK_ENABLE();
			}


		}

		else
		{
			if (pUSARTx == USART1)
			{
				USART1_CLK_DISABLE();
			}
			else if (pUSARTx == USART2)
			{
				USART2_CLK_DISABLE();
			}
			else if (pUSARTx == USART3)
			{
				USART3_CLK_DISABLE();
			}
			else if (pUSARTx == UART4)
			{
				UART4_CLK_DISABLE();
			}
			else if (pUSARTx == UART5)
			{
				UART5_CLK_DISABLE();
			}


		}
}


/**********************************************************
 * @fn				- USART_Init
 * @brief			- This function Initialize the given
 * 					  USART/UART Port
 *
 *
 * @param[in]		- Base addres of the USARTx/UARTx
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/
void  USART_Init(USART_Handle_t *pUSARTHandle,uint32_t SysCLK)
{
	//Temporary variable
	uint32_t tempreg = 0;

	//*****************Configuration of CR1 Reg*****************//

	//Enable the peripheral Clock
	USART_CLK_Control(pUSARTHandle->pUSARTx, ENABLE);

	//Configure the USART MOde
	if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		tempreg |= (1 << USART_CR1_RE);
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		tempreg |= (1 << USART_CR1_TE);
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		tempreg |= (0x3 << USART_CR1_RE);
	}

	//Implement the code to configure the Word length configuration item
	tempreg |= (pUSARTHandle->USART_Config.USART_WordLenght << USART_CR1_M);

	// Configuration of parity control bit
	if (pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_EN_EVEN)
	{
		//Enable parity control
		tempreg |= (1 << USART_CR1_PCE);
	}
	else if (pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_EN_ODD)
	{
		//Enable parity control
		tempreg |= (1 << USART_CR1_PCE);
		//Enable ODD parity
		tempreg |= (1 << USART_CR1_PS);
	}

	//PROGRAM the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;




	//*****************Configuration of CR2 Reg*****************//

	tempreg = 0;

	//Configure the number of stop bits inserted
	tempreg |= (pUSARTHandle->USART_Config.USART_StopBits << USART_CR2_STOP);

	//PROGRAM the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;




	//*****************Configuration of CR3 Reg*****************//

	tempreg = 0;

	//Configuration of USART Hardware flow control

	if (pUSARTHandle->USART_Config.USART_HWFlow == USART_HW_FLOW_CTRL_CTS)
	{
		//enable CTS flow control
		tempreg |= (1 << USART_CR3_CTSE);
	}
	else if (pUSARTHandle->USART_Config.USART_HWFlow == USART_HW_FLOW_CTRL_RTS)
	{
		//enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE);

	}
	else if (pUSARTHandle->USART_Config.USART_HWFlow == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//enable both CTS and RTS Flow control
		tempreg |= (0x3 << USART_CR3_RTSE);

	}


	// PROGRAM the CR3 register
	pUSARTHandle->pUSARTx->CR3 = tempreg;




	//*****************Configuration of CR3 BRR*****************//
	//*****************  Baudrate register   *****************//
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud, SysCLK);


	////ENABLE USART

	pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_UE);

}




/**********************************************************
 * @fn				- USART_DeInit
 * @brief			- This function De-Init the given USART Port
 *
 *
 * @param[in]		- Base addres of the USARTx/UARTx
 *
 * @return			- none
 *
 * @note			- none
 *********************************************************/
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if (pUSARTx == USART1)
	{
		USART1_REG_RESET();
	}
	else if (pUSARTx == USART2)
	{
		USART2_REG_RESET();
	}
	else if (pUSARTx == USART3)
	{
		USART3_REG_RESET();
	}

	else if (pUSARTx == UART4)
	{
		UART4_REG_RESET();
	}

	else if (pUSARTx == UART5)
	{
		UART5_REG_RESET();
	}
}

/**********************************************************
 * @fn				- USART_PeripheralControl
 * @brief			- This function enable or disable the peripheral
 *
 *
 * @param[in]		- Base addres of the USARTx/UARTx
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @note			-
 *********************************************************/
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}
	else
	{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}



/**********************************************************
 * @fn				- USART_SendData
 * @brief			- This function send data from the given
 * 					  USARTx/UARTx
 *
 *
 * @param[in]		- USART Handle structure
 *
 * @return			- none
 *
 * @note			- This is blocking call
 *********************************************************/
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;

	//Loop over until "Len" number of bytes are transfered
	for (uint32_t i = 0; i < Len; i++)
	{

		//Wait until TXE flag is set in the SR
		while( !(pUSARTHandle->pUSARTx->SR & USART_FLAG_TXE) );


		//check the USART_WordLenght item for 9Bit or 8Bit in a frame
		if (pUSARTHandle->USART_Config.USART_WordLenght == USART_WORDLEN_9BIT)
		{
			//Load the DR with 2bytes masking the bits other than 9 bits
			pdata = (uint16_t*)pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			if(pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_EN_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer. so, 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware.
				pTxBuffer++;
			}

		}

		else
		{
			//for 8Bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);
			//Increment buffer address
			pTxBuffer++;

		}
	}

	//Implement the code to wait until TC (transmission complete) flag is set in the SR
	while(! (pUSARTHandle->pUSARTx->SR & USART_FLAG_TC));
}

/**********************************************************
 * @fn				- USART_ReceiveData
 * @brief			- This function send data from the given
 * 					  USARTx/UARTx
 *
 *
 * @param[in]		- USART Handle structure
 *
 * @return			- none
 *
 * @note			- This is blocking call
 *********************************************************/
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	//Loop over until "Len" number of bytes are transfered
	for(uint32_t i = 0; i < Len; i++)
	{
		//wait until RXNE flag is set in the SR
		while( !(pUSARTHandle->pUSARTx->SR & USART_FLAG_RXNE) );

		//Check the WordLength to decide wheter we are going to receive 9Bit of dataframe or 8Bit
		if (pUSARTHandle->USART_Config.USART_WordLenght == USART_WORDLEN_9BIT)
		{

			//Check the Parity control
			if (pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_EN_DISABLE )
			{
				//Read only first 9 bits. Mask the DR
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);
				pRxBuffer++;
				pRxBuffer++;
			}

			else
			{// Parity Enable
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
				pRxBuffer++;
			}
		}

		else
		{
			//Receive 8Bit

			//Check are Parity control
			if (pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_EN_DISABLE )
			{
				//8 Bit user data
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
			}

			else
			{// Parity Enable 7 bit of user data

				*pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}
}

/**********************************************************
 * @fn				- USART_SetBaudRate
 * @brief			- This function set the baudrate from the given
 * 					  USARTx/UARTx
 *
 *
 * @param[in]		- USART Handle structure
 * @param[in]		- Desired baudrate
 * @param[in]		- System Clock Source Value
 *
 * @return			- none
 *
 * @note			-
 *********************************************************/

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate, uint32_t SysCLK)
{
	//variable to hold the APBx clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//Variables to hold Mantisa and Fraction values
	uint32_t M_part, F_part;

	uint32_t tempreg = 0;

	//Get the value of the APB bus clock
	if (pUSARTx == USART1)
	{
		// USART1 Hanging on APB2 bus
		PCLKx = RCC_GetAPB2_CLKValue(SysCLK);

	}
	else
	{
		// USART2, USART3, UART4 & UART5 Hanging on APB1 bus
		PCLKx = RCC_GetAPB1_CLKValue(SysCLK);
	}


	//Calculate USARTDIV = FCLK/(16*Baud)
	//multiplied by 100 to remove decimal point
	usartdiv = ((25 * PCLKx) / (4 * BaudRate) );

	//Calculate Mantissa part
	M_part = usartdiv / 100;



	//Extract the fraction part (multiplied by 100)
	F_part = (usartdiv - ( M_part * 100));
	//Calculate final fraction
	//multiplied by 16 and rounded, then divided by 100
	F_part = (((F_part * 16) + 50 ) / 100 ) & ((uint8_t)0x0F);


	//place mantissa part in appropriate bit position
	tempreg |= (M_part << 4);
	//place the fractional part in appropriate bit position
	tempreg |= F_part;

	//Load the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;


}


