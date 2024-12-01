/****************************************FILE DESCRIPTION**************************************/
/* FILE 		: UART.c
* PROJECT 		: HAL Library for STM32F4xx
* PROGRAMMER 	: Brijesh Mehta
* DESCRIPTION 	: UART Functions
*/
/*********************************************************************************************/

/************** Includes ***********************/
#include "UART.h"

/********************* Function Description *********************************
 * FUNCTION		:	uartFlagCheck
 *
 * DESCRIPTION	:	This function is used to check whether a flag is set or not
 *
 * PARAMETERS	:	USART_TypeDef *UARTx - UARTx address
 * 					USART_Flag_t flag - UART Flag
 *
 * RETURN		:	void
 *
 * **************************************************************************/
uint8_t uartFlagCheck(USART_TypeDef *UARTx, UART_Flag_t flag)
{
	uint8_t status;
	if(flag == UART_RXNE_FLAG)
	{
		status = REG_READ_BIT(UARTx->SR,USART_SR_RXNE_Pos);
	}
	else if(flag == UART_TXE_FLAG)
	{
		status = REG_READ_BIT(UARTx->SR,USART_SR_TXE_Pos);
	}
	else if(flag == UART_TC_FLAG)
	{
		status = REG_READ_BIT(UARTx->SR,USART_SR_TC_Pos);
	}
	else if(flag == UART_TCIE_FLAG)
	{
		status = REG_READ_BIT(UARTx->CR1,USART_CR1_TCIE_Pos);
	}
	else if(flag == UART_TXEIE_FLAG)
	{
		status = REG_READ_BIT(UARTx->CR1,USART_CR1_TXEIE_Pos);
	}
	else if(flag == UART_RXNEIE_FLAG)
	{
		status = REG_READ_BIT(UARTx->CR1,USART_CR1_RXNEIE_Pos);
	}
	return status;
}

/********************* Function Description *********************************
 * FUNCTION		:	uartClockControl
 *
 * DESCRIPTION	:	This function is used to enable or disable the UART Clock
 *
 * PARAMETERS	:	USART_TypeDef *UARTx - UARTx base address
 * 					uint8_t EnorDi 	- ENABLE or DISABLE
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void uartClockControl(USART_TypeDef *UARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(UARTx == USART1)
		{
			RCC_USART1_CLOCK_ENABLE();
		}
		else if(UARTx == USART2)
		{
			RCC_USART2_CLOCK_ENABLE();
		}
		else if(UARTx == USART3)
		{
			RCC_USART3_CLOCK_ENABLE();
		}
	}
	else if(EnorDi == DISABLE)
	{
		if(UARTx == USART1)
		{
			RCC_USART1_CLOCK_DISABLE();
		}
		else if(UARTx == USART2)
		{
			RCC_USART2_CLOCK_DISABLE();
		}
		else if(UARTx == USART3)
		{
			RCC_USART3_CLOCK_DISABLE();
		}
	}
}

/********************* Function Description *********************************
 * FUNCTION		:	uartWaitOnFlagUntilTimeout
 *
 * DESCRIPTION	:	This function is used to wait on flag until timeout
 *
 * PARAMETERS	:	UART_Handle_t *uartHandleStruct - UART Handle structure
 * 					uint32_t Flag - Flag that needs to be checked
 *
 * RETURN		:	void
 *
 * **************************************************************************/
Status_t uartWaitOnFlagUntilTimeout(UART_Handle_t *uartHandleStruct, uint32_t Flag, uint32_t Timeout)
{
	uint32_t ticksStart;

	ticksStart = getTicks();

	while(!(uartFlagCheck(uartHandleStruct->UARTx, Flag)))
	{
		if((ticksStart - getTicks()) > Timeout)
		{
			return HAL_TIMEOUT;
		}
	}
	return HAL_OK;
}

/********************* Function Description *********************************
 * FUNCTION		:	uartSetBaudRate
 *
 * DESCRIPTION	:	This function is used to set the UART peripheral baudrate
 *
 * PARAMETERS	:	UART_Handle_t *uartHandleStruct - UART Handle structure
 * 					uint32_t baudRate - baud rate value
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void uartSetBaudRate(UART_Handle_t *uartHandleStruct)
{
	/*
	 * USARTDIV is the value that needs to be written in the BRR register to set the Baud Rate
	 * USARTDIV consists of two part the Mantissa(Integer Part) and the Fraction Part
	 * The formula to calculate the USART DIV is
	 * 		USARTDIV = fclk / (8 *(2 - OVER8) * Baudrate)
	 *
	 * The USARTDIV is float number and we need to get the Mantissa and fraction part from it and write it
	 * to the respective registers inside the BRR Register
	 */

	uint8_t over8 = 0;
	uint8_t oversampling = 0;
	uint32_t fclk = 0;
	uint32_t usartdiv = 0;
	uint32_t mantissa = 0;
	uint32_t fraction = 0;
	uint32_t tempreg = 0;

	//Identifying the FCLK
	if(uartHandleStruct->UARTx == USART1)
	{
		fclk = RCC_GetPCLK2Freq();
	}
	else if(uartHandleStruct->UARTx == USART2 || uartHandleStruct->UARTx == USART3)
	{
		fclk = RCC_GetPCLK1Freq();
	}

	//Identify the over sampling rate
	over8 = uartHandleStruct->uartInitStruct.OverSampling;
	if(over8)
	{
		//over sampling by 8
		oversampling = 8;
		usartdiv = ((25 * fclk) / (2 * (uartHandleStruct->uartInitStruct.BaudRate)));
	}
	else
	{
		//over sampling by 16
		oversampling = 16;
		usartdiv = ((25 * fclk) / (4 * (uartHandleStruct->uartInitStruct.BaudRate)));
	}

	mantissa = usartdiv / 100;
	fraction = ((((usartdiv - (mantissa * 100)) * oversampling) + 50) / 100);	//50 is the correction factor to round off a number

	//Storing the value of mantissa in a temporary register
	tempreg |= (mantissa << 4);

	//Storing the fraction value to the temporary register
	tempreg |= fraction;

	//writing the value to the BRR register
	uartHandleStruct->UARTx->BRR = tempreg;
}

/********************* Function Description *********************************
 * FUNCTION		:	uartInit
 *
 * DESCRIPTION	:	This function is used to configure the UART peripheral
 *
 * PARAMETERS	:	UART_Handle_t *uartHandleStruct - UART Handle structure
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void uartInit(UART_Handle_t *uartHandleStruct)
{
	//Enabling the UART Clock
	uartClockControl(uartHandleStruct->UARTx, ENABLE);

	/******************** Configuring the Baud rate **************************/
	uartSetBaudRate(uartHandleStruct);

	/******************* Configuring the Word Length *************************/
	REG_SET_VAL(uartHandleStruct->UARTx->CR1,uartHandleStruct->uartInitStruct.WordLength,0x1,USART_CR1_M_Pos);

	/******************* Configuring the Stop Bits ***********************/
	REG_SET_VAL(uartHandleStruct->UARTx->CR2,uartHandleStruct->uartInitStruct.StopBits,0x3,USART_CR2_STOP_Pos);

	/****************** Configuring the Parity ***************************/
	if(uartHandleStruct->uartInitStruct.Parity != UART_PARITY_NONE)
	{
		//Enabling the Parity
		REG_SET_VAL(uartHandleStruct->UARTx->CR1,0x1,0x1,USART_CR1_PCE_Pos);

		//Configuring the Parity
		REG_SET_VAL(uartHandleStruct->UARTx->CR1,uartHandleStruct->uartInitStruct.Parity,0x1,USART_CR1_PS_Pos);
	}

	/******************* Configuring the Mode ***************************/
	if(uartHandleStruct->uartInitStruct.Mode == UART_MODE_TX || uartHandleStruct->uartInitStruct.Mode == UART_MODE_TX_RX)
	{
		REG_SET_VAL(uartHandleStruct->UARTx->CR1,0x1,0x1,USART_CR1_TE_Pos);
	}
	if(uartHandleStruct->uartInitStruct.Mode == UART_MODE_RX || uartHandleStruct->uartInitStruct.Mode == UART_MODE_TX_RX)
	{
		REG_SET_VAL(uartHandleStruct->UARTx->CR1,0x1,0x1,USART_CR1_RE_Pos);
	}

	/******************* COnfiguring the Hardware Flow Control *************/
	if(uartHandleStruct->uartInitStruct.HwFlowCtl != UART_HWCONTROL_NONE)
	{
		if(uartHandleStruct->uartInitStruct.HwFlowCtl == UART_HWCONTROL_CTS || uartHandleStruct->uartInitStruct.HwFlowCtl == UART_HWCONTROL_RTS_CTS)
		{
			REG_SET_VAL(uartHandleStruct->UARTx->CR3,0x1,0x1, USART_CR3_RTSE_Pos);
		}
		if(uartHandleStruct->uartInitStruct.HwFlowCtl == UART_HWCONTROL_RTS || uartHandleStruct->uartInitStruct.HwFlowCtl == UART_HWCONTROL_RTS_CTS)
		{
			REG_SET_VAL(uartHandleStruct->UARTx->CR3,0x1,0x1,USART_CR3_CTSE_Pos);
		}
	}

	/***************** Configuring the Over sampling ***********************/
	REG_SET_VAL(uartHandleStruct->UARTx->CR1, uartHandleStruct->uartInitStruct.OverSampling,0x1,USART_CR1_OVER8_Pos);

	/***************** Enabling the UART ***********************/
	UART_ENABLE(uartHandleStruct);
}

/********************* Function Description *********************************
 * FUNCTION		:	uartTransmit
 *
 * DESCRIPTION	:	This function is used to transmit data over UART
 *
 * PARAMETERS	:	UART_Handle_t *huart	- UART Handle typedef
 * 					const uint8_t *pTxData	- Pointer to the data to be transmitted
 * 					uint16_t sizeOfData	- Size of the data
 * 					uint32_t Timeout	- Timeout value to prevent from getting stuck in a loop
 *
 * RETURN		:	Status_t - possible values HAL_OK, HAL_ERROR, HAL_TIMEOUT
 *
 * **************************************************************************/
Status_t uartTransmit(UART_Handle_t *uartHandleStruct, uint8_t *pTxData, uint16_t sizeOfData, uint32_t Timeout)
{
	for(uint32_t i=0; i<sizeOfData; i++)
	{
		//Wait until the TXE Flag is set
		if(uartWaitOnFlagUntilTimeout(uartHandleStruct, UART_TXE_FLAG, Timeout) != HAL_OK)
		{
			return HAL_TIMEOUT;
		}

		//Check whether word length is 8Bits or 9Bits
		if(uartHandleStruct->uartInitStruct.WordLength == UART_WORDLENGTH_9B) //If word length is 9 bits
		{
			//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
			uartHandleStruct->UARTx->DR = (uint16_t)(*((uint16_t*) pTxData) & 0x01FFU);

			//check for USART_ParityControl
			if(uartHandleStruct->uartInitStruct.Parity == UART_PARITY_NONE)
			{
				//No parity is used in this transfer , so 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxData++;
				pTxData++;
			}
			else
			{
				//Parity bit is used in this transfer . so 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxData++;
			}
		}
		else if(uartHandleStruct->uartInitStruct.WordLength == UART_WORDLENGTH_8B) //If word Length is 8 Bits
		{
			//8 Bit data transfer
			uartHandleStruct->UARTx->DR = (uint8_t)(*pTxData & 0xFFU);

			//Irrespective of the parity bit the pTxData is incremented by 8 bits
			pTxData++;
		}
	}

	//Wait until TC Flag is set
	if(uartWaitOnFlagUntilTimeout(uartHandleStruct, UART_TC_FLAG, Timeout) != HAL_OK)
	{
		return HAL_TIMEOUT;
	}

	return HAL_OK;
}

/********************* Function Description *********************************
 * FUNCTION		:	uartReceive
 *
 * DESCRIPTION	:	This function is used to receive data over UART
 *
 * PARAMETERS	:	UART_Handle_t *huart	- UART Handle typedef
 * 					const uint8_t *pTxData	- Pointer to the data to be transmitted
 * 					uint16_t sizeOfData	- Size of the data
 * 					uint32_t Timeout	- Timeout value to prevent from getting stuck in a loop
 *
 * RETURN		:	Status_t - possible values HAL_OK, HAL_ERROR, HAL_TIMEOUT
 *
 * **************************************************************************/
Status_t uartReceive(UART_Handle_t *uartHandleStruct, uint8_t *pRxData, uint16_t sizeOfData, uint32_t Timeout)
{
	for(uint32_t i=0; i<sizeOfData; i++)
	{
		//Wait until the RXNE Flag is set
		if(uartWaitOnFlagUntilTimeout(uartHandleStruct, UART_RXNE_FLAG, Timeout) != HAL_OK)
		{
			return HAL_TIMEOUT;
		}

		//Check whether word length is 8Bits or 9Bits
		if(uartHandleStruct->uartInitStruct.WordLength == UART_WORDLENGTH_9B) //If word length is 9 bits
		{
			//Now, check are we using USART_ParityControl control or not
			if(uartHandleStruct->uartInitStruct.Parity == UART_PARITY_NONE) //Parity is not used
			{
				//read only first 9 bits so mask the DR with 0x01FF
				*((uint16_t*)pRxData) = (uartHandleStruct->UARTx->DR & (uint16_t)0x01FF);

				//Increment the pRxData pointer
				pRxData++;
				pRxData++;
			}
			else //Parity is used
			{
				//Parity is used, so 8bits will be of user data and 1 bit is parity
				 *pRxData = (uartHandleStruct->UARTx->DR  & (uint8_t)0xFF);
				 pRxData++;
			}
		}
		else if(uartHandleStruct->uartInitStruct.WordLength == UART_WORDLENGTH_8B)	//If word Length is 8 bits
		{
			//Now, check are we using USART_ParityControl control or not
			if(uartHandleStruct->uartInitStruct.Parity == UART_PARITY_NONE) //Parity is not used
			{
				//No parity is used , so all 8bits will be of user data
				*pRxData = (uartHandleStruct->UARTx->DR & (uint8_t)0xFF);
			}
			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity
				*pRxData = (uartHandleStruct->UARTx->DR & (uint8_t)0x7F);
			}

			//Increment the pRxData pointer
			pRxData++;
		}
	}
	return HAL_OK;
}

/********************* Function Description *********************************
 * FUNCTION		:	uartTransmitIT
 *
 * DESCRIPTION	:	This function is used to transmit data over UART using Interrupts
 *
 * PARAMETERS	:	UART_Handle_t *huart	- UART Handle typedef
 * 					const uint8_t *pTxData	- Pointer to the data to be transmitted
 * 					uint16_t sizeOfData	- Size of the data
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void uartTransmitIT(UART_Handle_t *uartHandleStruct, uint8_t *pTxData, uint16_t sizeOfData)
{
	UART_States_t state = uartHandleStruct->TxState;

	if(state == UART_READY)
	{
		uartHandleStruct->pTxData = pTxData;
		uartHandleStruct->TxLen = sizeOfData;
		uartHandleStruct->TxState = UART_BUSY_IN_TX;

		//Enabling the TXE Interrupt
		REG_SET_VAL(uartHandleStruct->UARTx->CR1,0x1,0x1,USART_CR1_TXEIE_Pos);

		//Enabling the TC Interrupt
		REG_SET_VAL(uartHandleStruct->UARTx->CR1,0x1,0x1,USART_CR1_TCIE_Pos);
	}
}

/********************* Function Description *********************************
 * FUNCTION		:	uartReceiveIT
 *
 * DESCRIPTION	:	This function is used to receive data over UART using Interrupts
 *
 * PARAMETERS	:	UART_Handle_t *huart	- UART Handle typedef
 * 					const uint8_t *pRxData	- Pointer to the data to be received
 * 					uint16_t sizeOfData	- Size of the data
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void uartReceiveIT(UART_Handle_t *uartHandleStruct, uint8_t *pRxData, uint16_t sizeOfData)
{
	UART_States_t state = uartHandleStruct->RxState;

	if(state == UART_READY)
	{
		uartHandleStruct->pRxData = pRxData;
		uartHandleStruct->RxLen = sizeOfData;
		uartHandleStruct->RxState = UART_BUSY_IN_RX;

		//Enabling the RXNE Interrupt
		REG_SET_VAL(uartHandleStruct->UARTx->CR1,0x1,0x1,USART_CR1_RXNEIE_Pos);
	}
}

/********************* Function Description *********************************
 * FUNCTION		:	uartIRQHandler
 *
 * DESCRIPTION	:	This function is used to handle the UART Interrupt
 *
 * PARAMETERS	:	UART_Handle_t *huart	- UART Handle typedef
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void uartIRQHandler(UART_Handle_t *uartHandleStruct)
{
	/****************** Handling the TC Interrupt *****************/
	if((uartFlagCheck(uartHandleStruct->UARTx, UART_TCIE_FLAG)) && (uartFlagCheck(uartHandleStruct->UARTx, UART_TC_FLAG)))
	{
		uartTCInterruptHandle(uartHandleStruct);
	}

	/******************* Handling the TXE Interrupt *************************/
	if((uartFlagCheck(uartHandleStruct->UARTx, UART_TXE_FLAG)) && (uartFlagCheck(uartHandleStruct->UARTx, UART_TXEIE_FLAG)))
	{
		uartTXInterruptHandle(uartHandleStruct);
	}

	/******************* Handling the RXNE Interrupt *************************/
	if((uartFlagCheck(uartHandleStruct->UARTx, UART_RXNE_FLAG)) && (uartFlagCheck(uartHandleStruct->UARTx, UART_RXNEIE_FLAG)))
	{
		uartRXInterruptHandle(uartHandleStruct);
	}
}

/********************* Function Description *********************************
 * FUNCTION		:	uartTXInterruptHandle
 *
 * DESCRIPTION	:	This function is used to handle the UART TxInterrupt
 *
 * PARAMETERS	:	UART_Handle_t *huart	- UART Handle typedef
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void uartTXInterruptHandle(UART_Handle_t *uartHandleStruct)
{
	if(uartHandleStruct->TxState == UART_BUSY_IN_TX)
	{
		if(uartHandleStruct->TxLen > 0)
		{
			//Check whether word length is 8Bits or 9Bits
			if(uartHandleStruct->uartInitStruct.WordLength == UART_WORDLENGTH_9B) //If word length is 9 bits
			{
				//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
				uartHandleStruct->UARTx->DR = (uint16_t)(*((uint16_t*)(uartHandleStruct->pTxData)) & 0x01FFU);

				//check for USART_ParityControl
				if(uartHandleStruct->uartInitStruct.Parity == UART_PARITY_NONE)
				{
					//No parity is used in this transfer , so 9bits of user data will be sent
					//Implement the code to increment pTxBuffer twice
					(uartHandleStruct->pTxData)++;
					(uartHandleStruct->pTxData)++;

					//Reducing the Length
					(uartHandleStruct->TxLen)--;
					(uartHandleStruct->TxLen)--;
				}
				else
				{
					//Parity bit is used in this transfer . so 8bits of user data will be sent
					//The 9th bit will be replaced by parity bit by the hardware
					(uartHandleStruct->pTxData)++;

					//Reducing the Length
					(uartHandleStruct->TxLen)--;
				}
			}
			else if(uartHandleStruct->uartInitStruct.WordLength == UART_WORDLENGTH_8B) //If word Length is 8 Bits
			{
				//8 Bit data transfer
				uartHandleStruct->UARTx->DR = (uint8_t)(*(uartHandleStruct->pTxData) & 0xFFU);

				//Irrespective of the parity bit the pTxData is incremented by 8 bits
				(uartHandleStruct->pTxData)++;

				//Reducing the Length
				(uartHandleStruct->TxLen)--;
			}
		}
	}
}

/********************* Function Description *********************************
 * FUNCTION		:	uartRXInterruptHandle
 *
 * DESCRIPTION	:	This function is used to handle the UART RxInterrupt
 *
 * PARAMETERS	:	UART_Handle_t *huart	- UART Handle typedef
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void uartRXInterruptHandle(UART_Handle_t *uartHandleStruct)
{
	//Check whether word length is 8Bits or 9Bits
	if(uartHandleStruct->uartInitStruct.WordLength == UART_WORDLENGTH_9B) //If word length is 9 bits
	{
		//Now, check are we using USART_ParityControl control or not
		if(uartHandleStruct->uartInitStruct.Parity == UART_PARITY_NONE) //Parity is not used
		{
			//read only first 9 bits so mask the DR with 0x01FF
			*((uint16_t*)(uartHandleStruct->pRxData)) = (uartHandleStruct->UARTx->DR & (uint16_t)0x01FF);

			//Increment the pRxData pointer
			(uartHandleStruct->pRxData)++;
			(uartHandleStruct->pRxData)++;

			//Reducing the Length
			(uartHandleStruct->RxLen)--;
			(uartHandleStruct->RxLen)--;
		}
		else //Parity is used
		{
			//Parity is used, so 8bits will be of user data and 1 bit is parity
			 *(uartHandleStruct->pRxData) = (uartHandleStruct->UARTx->DR  & (uint8_t)0xFF);
			 (uartHandleStruct->pRxData)++;

			//Reducing the Length
			(uartHandleStruct->RxLen)--;
		}
	}
	else if(uartHandleStruct->uartInitStruct.WordLength == UART_WORDLENGTH_8B)	//If word Length is 8 bits
	{
		//Now, check are we using USART_ParityControl control or not
		if(uartHandleStruct->uartInitStruct.Parity == UART_PARITY_NONE) //Parity is not used
		{
			//No parity is used , so all 8bits will be of user data
			*(uartHandleStruct->pRxData) = (uartHandleStruct->UARTx->DR & (uint8_t)0xFF);
		}
		else
		{
			//Parity is used, so , 7 bits will be of user data and 1 bit is parity
			*(uartHandleStruct->pRxData) = (uartHandleStruct->UARTx->DR & (uint8_t)0x7F);
		}

		//Increment the pRxData pointer
		(uartHandleStruct->pRxData)++;

		//Reducing the Length
		(uartHandleStruct->RxLen)--;
	}
}

/********************* Function Description *********************************
 * FUNCTION		:	uartTCInterruptHandle
 *
 * DESCRIPTION	:	This function is used to handle the UART TcInterrupt
 *
 * PARAMETERS	:	UART_Handle_t *huart	- UART Handle typedef
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void uartTCInterruptHandle(UART_Handle_t *uartHandleStruct)
{
	if(uartHandleStruct->TxState == UART_BUSY_IN_TX)
	{
		if(uartHandleStruct->TxLen == 0)
		{
			//Clearing the TC Flag
			REG_CLR_BIT(uartHandleStruct->UARTx->SR,USART_SR_TC_Pos);

			//Reset the application state
			uartHandleStruct->TxState = UART_READY;

			//Reset the TxBuffer address
			uartHandleStruct->pTxData = NULL;

			//Reset the Length of the data
			uartHandleStruct->TxLen = 0;
		}
	}
}
