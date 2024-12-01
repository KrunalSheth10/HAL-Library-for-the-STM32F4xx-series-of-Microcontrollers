/****************************************FILE DESCRIPTION**************************************/
/* FILE 		: SPI.c
* PROJECT 		: HAL Library for STM32F4xx
* PROGRAMMER 	: Brijesh Mehta
* DESCRIPTION 	: SPI config functions
*/
/*********************************************************************************************/

/************* INCLUDES ****************/
#include "spi.h"

/********************* Function Description *********************************
 * FUNCTION		:	spiClockControl
 *
 * DESCRIPTION	:	This function is used to enable or disable the SPI Clock
 *
 * PARAMETERS	:	SPI_TypeDef *SPIx - SPIx base address
 * 					uint8_t EnorDi 	- ENABLE or DISABLE
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void spiClockControl(SPI_TypeDef *SPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(SPIx == SPI1)
		{
			RCC_SPI1_CLOCK_ENABLE();
		}
		else if(SPIx == SPI2)
		{
			 RCC_SPI2_CLOCK_ENABLE();
		}
		else if(SPIx == SPI3)
		{
			RCC_SPI3_CLOCK_ENABLE();
		}
	}
	else if(EnorDi == DISABLE)
	{
		if(SPIx == SPI1)
		{
			RCC_SPI1_CLOCK_DISABLE();
		}
		else if(SPIx == SPI2)
		{
			RCC_SPI2_CLOCK_DISABLE();
		}
		else if(SPIx == SPI3)
		{
			RCC_SPI3_CLOCK_DISABLE();
		}
	}

}

/********************* Function Description *********************************
 * FUNCTION		:	spiFlagCheck
 *
 * DESCRIPTION	:	This function is used to check whether the Flag is set or not
 *
 * PARAMETERS	:	SPI_TypeDef *SPIx - SPIx base address
 * 					SPI_Flag_t flag - Possible values from SPI_Flag_t
 *
 * RETURN		:	uint8_t - 1 if flag is set and 0 if not
 *
 * **************************************************************************/
uint8_t spiFlagCheck(SPI_TypeDef *SPIx, SPI_Flag_t flag)
{
	uint8_t status;
	if(flag == SPI_RXNE_FLAG)
	{
		status = REG_READ_BIT(SPIx->SR,SPI_SR_RXNE_Pos);
	}
	else if(flag == SPI_TXE_FLAG)
	{
		status = REG_READ_BIT(SPIx->SR,SPI_SR_TXE_Pos);
	}
	else if(flag == SPI_BSY_FLAG)
	{
		status = REG_READ_BIT(SPIx->SR,SPI_SR_BSY_Pos);
	}
	else if(flag == SPI_OVR_FLAG)
	{
		status = REG_READ_BIT(SPIx->SR,SPI_SR_OVR_Pos);
	}
	else if(flag == SPI_TXEIE_FLAG)
	{
		status = REG_READ_BIT(SPIx->CR2,SPI_CR2_TXEIE_Pos);
	}
	else if(flag == SPI_RXNEIE_FLAG)
	{
		status = REG_READ_BIT(SPIx->CR2,SPI_CR2_RXNEIE_Pos);
	}
	else if(flag == SPI_ERRIE_FLAG)
	{
		status = REG_READ_BIT(SPIx->CR2,SPI_CR2_ERRIE_Pos);
	}
	return status;
}

/********************* Function Description *********************************
 * FUNCTION		:	spiInit
 *
 * DESCRIPTION	:	This function is used to configure the GPIO
 *
 * PARAMETERS	:	SPI_Handle_t *spiHandleStructure - SPI handle structure
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void spiInit(SPI_Handle_t *spiHandleStructure)
{
	/******************* Enabling the Clock ***************************/
	spiClockControl(spiHandleStructure->SPIx, ENABLE);

	/*******************Configuring the Mode **************************/
	REG_SET_VAL(spiHandleStructure->SPIx->CR1,spiHandleStructure->Init.Mode,0x1,SPI_CR1_MSTR_Pos);

	/*******************Configuring the Direction *******************/
	REG_SET_VAL(spiHandleStructure->SPIx->CR1,spiHandleStructure->Init.Direction,0x1,SPI_CR1_BIDIMODE_Pos);

	/*******************Configuring the Data Size *******************/
	REG_SET_VAL(spiHandleStructure->SPIx->CR1,spiHandleStructure->Init.Direction,0x1,SPI_CR1_DFF_Pos);

	/*******************Configuring the Clock Polarity CPOL *********/
	REG_SET_VAL(spiHandleStructure->SPIx->CR1,spiHandleStructure->Init.CLKPolarity,0x1,SPI_CR1_CPOL_Pos);

	/*******************Configuring the Clock Phase CPHA *********/
	REG_SET_VAL(spiHandleStructure->SPIx->CR1,spiHandleStructure->Init.CLKPhase,0x1,SPI_CR1_CPHA_Pos);

	/*******************Configuring the NSS **********************/
	if(spiHandleStructure->Init.NSS == SPI_NSS_SOFTWARE)
	{
		REG_SET_VAL(spiHandleStructure->SPIx->CR1,spiHandleStructure->Init.NSS,0x1,SPI_CR1_SSM_Pos); //Software Slave Management
		REG_SET_VAL(spiHandleStructure->SPIx->CR1,0x1,0x1,SPI_CR1_SSI_Pos);		//Setting the SSI Bit high
	}
	else if(spiHandleStructure->Init.NSS == SPI_NSS_HARDWARE)
	{
		REG_SET_VAL(spiHandleStructure->SPIx->CR1,spiHandleStructure->Init.NSS,0x1,SPI_CR1_SSM_Pos); //Hardware Slave Management
	}

	/******************Setting the BaudRate Prescalar *************/
	REG_SET_VAL(spiHandleStructure->SPIx->CR1,spiHandleStructure->Init.BaudRatePrescaler,0x7,SPI_CR1_BR_Pos);

	/*****************Configuring MSB/LSB First ******************/
	REG_SET_VAL(spiHandleStructure->SPIx->CR1,spiHandleStructure->Init.FirstBit,0x1,SPI_CR1_LSBFIRST_Pos);
}


/********************* Function Description *********************************
 * FUNCTION		:	spiTransmit
 *
 * DESCRIPTION	:	This function is used to transmit data using SPI
 *
 * PARAMETERS	:	SPI_Handle_t *spiHandleStructure - SPI Handle Structure
 * 					uint8_t *pTxData	- Pointer to data array
 * 					uint16_t sizeOfData	- size of the data
 * 					uint32_t timeout - timeout value to prevent getting stuck in a loop
 *
 * RETURN		:	Status_t
 *
 * **************************************************************************/
Status_t spiTransmit(SPI_Handle_t *spiHandleStructure, uint8_t *pTxData, uint16_t sizeOfData, uint32_t timeout)
{
	uint32_t ticksStart;

	//Configuration SPI to transmit only if direction is 1 line
	if(spiHandleStructure->Init.Direction == SPI_DIRECTION_1LINE)
	{
		//Setting the BIDIOE bit to transmit only
		REG_SET_VAL(spiHandleStructure->SPIx->CR1,0x1,0x1,SPI_CR1_BIDIOE_Pos);
	}

	//Enabling the SPI peripheral
	SPI_ENABLE(spiHandleStructure);

	while(sizeOfData > 0)
	{
		if(spiHandleStructure->Init.DataSize == SPI_DATA_SIZE_16BIT) //if data is 16 bit
		{
			//Writing 16 bits of data to the Data Register
			REG_WRITE(spiHandleStructure->SPIx->DR, *(uint16_t*)pTxData);

			//Wait until TXE Flag is set
			ticksStart = getTicks();

			while(!(spiFlagCheck(spiHandleStructure->SPIx, SPI_TXE_FLAG)))
			{
				if((ticksStart - getTicks()) > timeout)
				{
					return HAL_TIMEOUT;
				}
			}

			//Subtracting size of data by two as 16 bits of data has been transmitted
			sizeOfData = sizeOfData - 2;

			//Incrementing the TxData
			(uint16_t*)pTxData++;

		}
		else if(spiHandleStructure->Init.DataSize == SPI_DATA_SIZE_8BIT)
		{
			//Writing 8 bits of data to the Data Register
			REG_WRITE(spiHandleStructure->SPIx->DR,*pTxData);

			//Wait until TXE Flag is set
			ticksStart = getTicks();

			while(!(spiFlagCheck(spiHandleStructure->SPIx, SPI_TXE_FLAG)))
			{
				if((ticksStart - getTicks()) > timeout)
				{
					return HAL_TIMEOUT;
				}
			}

			//Subtracting size of data by 1 as 8 bits of data has been transmitted
			sizeOfData--;

			//Incrementing the TxData
			pTxData++;
		}
	}

	//Wait until BSY FLAG is 0
	ticksStart = getTicks();

	while(spiFlagCheck(spiHandleStructure->SPIx, SPI_BSY_FLAG))
	{
		if((ticksStart - getTicks()) > timeout)
		{
			return HAL_TIMEOUT;
		}
	}

	//Disabling the SPI peripheral
	SPI_DISABLE(spiHandleStructure);

	return HAL_OK;
}


/********************* Function Description *********************************
 * FUNCTION		:	spiReceive
 *
 * DESCRIPTION	:	This function is used to receive data using SPI
 *
 * PARAMETERS	:	SPI_Handle_t *spiHandleStructure - SPI Handle Structure
 * 					uint8_t *pRxData	- Pointer to data array
 * 					uint16_t sizeOfData	- size of the data
 * 					uint32_t timeout - timeout value to prevent getting stuck in a loop
 *
 * RETURN		:	Status_t
 *
 * **************************************************************************/
Status_t spiReceive(SPI_Handle_t *spiHandleStructure, uint8_t *pRxData, uint16_t sizeOfData, uint32_t timeout)
{
	uint32_t ticksStart;

	//If the SPI Mode is master and Full duplex
	if(spiHandleStructure->Init.Mode == SPI_MODE_MASTER && spiHandleStructure->Init.Direction ==  SPI_DIRECTION_2LINE)
	{
		spiTransmitReceive(spiHandleStructure, pRxData, pRxData, sizeOfData, timeout);
	}

	//Enabling the SPI peripheral
	SPI_ENABLE(spiHandleStructure);

	//Configuration SPI to receive only if direction is 1 line
	if(spiHandleStructure->Init.Direction == SPI_DIRECTION_1LINE || spiHandleStructure->Init.Mode == SPI_MODE_SLAVE)
	{
		if(spiHandleStructure->Init.Direction == SPI_DIRECTION_1LINE)
		{
			//Setting the BIDIOE bit to receive only
			REG_CLR_VAL(spiHandleStructure->SPIx->CR1,0x1,SPI_CR1_BIDIOE_Pos);
		}
		while(sizeOfData > 0)
		{
			//Wait until the RXNE Flag is set
			ticksStart = getTicks();

			while(!(spiFlagCheck(spiHandleStructure->SPIx, SPI_RXNE_FLAG)))
			{
				if((ticksStart - getTicks()) > timeout)
				{
					return HAL_TIMEOUT;
				}
			}

			if(spiHandleStructure->Init.DataSize == SPI_DATA_SIZE_16BIT) //if data is 16 bit
			{
				*((uint16_t*)pRxData) = REG_READ(spiHandleStructure->SPIx->DR);

				//Subtracting size of data by two as 16 bits of data has been transmitted
				sizeOfData = sizeOfData - 2;

				//Incrementing the RxData
				(uint16_t*)pRxData++;

			}
			else if(spiHandleStructure->Init.DataSize == SPI_DATA_SIZE_8BIT)
			{
				*pRxData = REG_READ(spiHandleStructure->SPIx->DR);

				//Subtracting size of data by 1 as 8 bits of data has been transmitted
				sizeOfData--;

				//Incrementing the TxData
				pRxData++;
			}
		}
	}

	//Disabling the SPI peripheral
	SPI_DISABLE(spiHandleStructure);

	return HAL_OK;
}

/********************* Function Description *********************************
 * FUNCTION		:	spiTransmitReceive
 *
 * DESCRIPTION	:	This function is used to transmit and receive data using SPI
 *
 * PARAMETERS	:	SPI_Handle_t *spiHandleStructure - SPI Handle Structure
 * 					uint8_t *pTxData - Pointer to data array to be Transmitted
 * 					uint8_t *pRxData - Pointer to data array to be Received
 * 					uint16_t sizeOfData	- size of the data
 * 					uint32_t timeout - timeout value to prevent getting stuck in a loop
 *
 * RETURN		:	Status_t
 *
 * **************************************************************************/
Status_t spiTransmitReceive(SPI_Handle_t *spiHandleStructure, uint8_t *pTxData, uint8_t *pRxData, uint16_t sizeOfData, uint32_t timeout)
{
	uint32_t ticksStart;

	//Enabling the SPI peripheral
	SPI_ENABLE(spiHandleStructure);

	while(sizeOfData > 0)
	{
		if(spiHandleStructure->Init.DataSize == SPI_DATA_SIZE_16BIT) //if data is 16 bit
		{
			//Writing 16 bits of data to the Data Register
			REG_WRITE(spiHandleStructure->SPIx->DR, *(uint16_t*)pTxData);

			//Wait until TXE Flag is set
			ticksStart = getTicks();

			while(!(spiFlagCheck(spiHandleStructure->SPIx, SPI_TXE_FLAG)))
			{
				if((ticksStart - getTicks()) > timeout)
				{
					return HAL_TIMEOUT;
				}
			}

			//Wait until the RXNE Flag is set
			ticksStart = getTicks();

			while(!(spiFlagCheck(spiHandleStructure->SPIx, SPI_RXNE_FLAG)))
			{
				if((ticksStart - getTicks()) > timeout)
				{
					return HAL_TIMEOUT;
				}
			}

			//Reading 16 bits
			*((uint16_t*)pRxData) = REG_READ(spiHandleStructure->SPIx->DR);

			//Subtracting size of data by two as 16 bits of data has been transmitted
			sizeOfData = sizeOfData - 2;

			//Incrementing the TxData
			(uint16_t*)pTxData++;

			//Incrementing the RxData
			(uint16_t*)pRxData++;
		}
		else if(spiHandleStructure->Init.DataSize == SPI_DATA_SIZE_8BIT)
		{
			//Writing 8 bits of data to the Data Register
			REG_WRITE(spiHandleStructure->SPIx->DR, *pTxData);

			//Wait until TXE Flag is set
			ticksStart = getTicks();

			while(!(spiFlagCheck(spiHandleStructure->SPIx, SPI_TXE_FLAG)))
			{
				if((ticksStart - getTicks()) > timeout)
				{
					return HAL_TIMEOUT;
				}
			}

			//Wait until the RXNE Flag is set
			ticksStart = getTicks();

			while(!(spiFlagCheck(spiHandleStructure->SPIx, SPI_RXNE_FLAG)))
			{
				if((ticksStart - getTicks()) > timeout)
				{
					return HAL_TIMEOUT;
				}
			}

			//Reading 8 bits
			*(pRxData) = REG_READ(spiHandleStructure->SPIx->DR);

			//Subtracting size of data by 1 as 8  bits are transferred
			sizeOfData = sizeOfData - 1;

			//Incrementing the TxData
			pTxData++;

			//Incrementing the RxData
			pRxData++;
		}
	}

	//Disabling the SPI peripheral
	SPI_DISABLE(spiHandleStructure);

	return HAL_OK;
}

/********************* Function Description *********************************
 * FUNCTION		:	spiTransmitIT
 *
 * DESCRIPTION	:	This function is used to transmit data using SPI in Interrupt Mode
 *
 * PARAMETERS	:	SPI_Handle_t *spiHandleStructure - SPI Handle Structure
 * 					uint8_t *pTxData	- Pointer to data array
 * 					uint16_t sizeOfData	- size of the data
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void spiTransmitIT(SPI_Handle_t *spiHandleStructure, uint8_t *pTxData, uint16_t sizeOfData)
{
	SPI_States_t state = spiHandleStructure->TxState;

	if(state == SPI_READY)
	{
		/*
		 * 1. Save the TxData address and sizeOfData in the respective variables created in the handle structure.
		 * 	  This will be later used by the Interrupt Handler to Transmit the Data.
		 */
		spiHandleStructure->pTxData = pTxData;
		spiHandleStructure->TxLen = sizeOfData;

		/*
		 * 2. Mark the SPI state as busy in transmission so that no other code can take over the same SPI
		 * 	  peripheral until transmission is over.
		 */
		spiHandleStructure->TxState = SPI_BUSY_IN_TX;

		/*
		 * 3.Configuring SPI to transmit only if direction is 1 line
		 */
		if(spiHandleStructure->Init.Direction == SPI_DIRECTION_1LINE)
		{
			//Setting the BIDIOE bit to transmit only
			REG_SET_VAL(spiHandleStructure->SPIx->CR1,0x1,0x1,SPI_CR1_BIDIOE_Pos);
		}

		/*
		 * 4.Enabling the SPI Peripheral
		 */
		SPI_ENABLE(spiHandleStructure);

		/*
		 * 5.Enable the TXEIE control bit to get interrupt whenever the TXE Flag is set in SR
		 */
		SPI_ENABLE_IT(spiHandleStructure,SPI_IT_TXE);

		/*
		 * 6. Data transmission will be handled by the ISR
		 */
	}
}

/********************* Function Description *********************************
 * FUNCTION		:	spiTransmitIT
 *
 * DESCRIPTION	:	This function is used to transmit data using SPI in Interrupt Mode
 *
 * PARAMETERS	:	SPI_Handle_t *spiHandleStructure - SPI Handle Structure
 * 					uint8_t *pRxData	- Pointer to data array
 * 					uint16_t sizeOfData	- size of the data
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void spiReceiveIT(SPI_Handle_t *spiHandleStructure, uint8_t *pRxData, uint16_t sizeOfData)
{
	SPI_States_t state = spiHandleStructure->RxState;

	//If the SPI Mode is master and Full duplex
	if(spiHandleStructure->Init.Mode == SPI_MODE_MASTER && spiHandleStructure->Init.Direction ==  SPI_DIRECTION_2LINE)
	{
		spiTransmitReceiveIT(spiHandleStructure, pRxData, pRxData, sizeOfData);
	}

	else if(spiHandleStructure->Init.Direction == SPI_DIRECTION_1LINE || spiHandleStructure->Init.Mode == SPI_MODE_SLAVE)
	{
		if(state == SPI_READY)
		{
			/*
			 * 1. Save the RxData address and sizeOfData in the respective variables created in the handle structure.
			 * 	  This will be later used by the Interrupt Handler to Transmit the Data.
			 */
			spiHandleStructure->pRxData = pRxData;
			spiHandleStructure->RxLen = sizeOfData;

			/*
			 * 2. Mark the SPI state as busy in reception so that no other code can take over the same SPI
			 * 	  peripheral until transmission is over.
			 */
			spiHandleStructure->TxState = SPI_BUSY_IN_RX;

			/*
			 * 3.Configuring SPI to receive only as direction is 1 line
			 */
			REG_CLR_VAL(spiHandleStructure->SPIx->CR1,0x1,SPI_CR1_BIDIOE_Pos);

			/*
			 * 4.Enabling the SPI Peripheral
			 */
			SPI_ENABLE(spiHandleStructure);

			/*
			 * 5.Enable the RXNEIE control bit to get interrupt whenever the TXE Flag is set in SR
			 */
			SPI_ENABLE_IT(spiHandleStructure,SPI_IT_RXNE);

			/*
			 * 6. Data reception will be handled by the ISR
			 */
		}
	}
}

/********************* Function Description *********************************
 * FUNCTION		:	spiTransmitReceiveIT
 *
 * DESCRIPTION	:	This function is used to transmit data using SPI in Interrupt Mode
 *
 * PARAMETERS	:	SPI_Handle_t *spiHandleStructure - SPI Handle Structure
 * 					uint8_t *pTxData - Pointer to data array to be Transmitted
 * 					uint8_t *pRxData - Pointer to data array to be Received
 * 					uint16_t sizeOfData	- size of the data
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void spiTransmitReceiveIT(SPI_Handle_t *spiHandleStructure, uint8_t *pTxData, uint8_t *pRxData, uint16_t sizeOfData)
{
	SPI_States_t txstate = spiHandleStructure->TxState;
	SPI_States_t rxstate = spiHandleStructure->RxState;

	if(txstate == SPI_READY && rxstate == SPI_READY)
	{
		/*
		 * 1. Save the TxData,RxData address and sizeOfData in the respective variables created in the handle structure.
		 * 	  This will be later used by the Interrupt Handler to Transmit the Data.
		 */
		spiHandleStructure->pTxData = pTxData;
		spiHandleStructure->TxLen = sizeOfData;

		spiHandleStructure->pRxData = pRxData;
		spiHandleStructure->RxLen = sizeOfData;

		/*
		 * 2. Mark the SPI state as busy in reception and transmission so that no other code can take over the same SPI
		 * 	  peripheral until transmission is over.
		 */
		spiHandleStructure->TxState = SPI_BUSY_IN_TX;
		spiHandleStructure->RxState = SPI_BUSY_IN_RX;

		/*
		 * 4.Enabling the SPI Peripheral
		 */
		SPI_ENABLE(spiHandleStructure);

		/*
		 * 5.Enable the TXEIE and RXNEIE control bit to get interrupt whenever the TXE Flag is set in SR
		 */
		SPI_ENABLE_IT(spiHandleStructure,(SPI_IT_TXE | SPI_IT_RXNE ));

		/*
		 * 6. Data transmission and reception will be handled by the ISR
		 */
	}
}

/********************* Function Description *********************************
 * FUNCTION		:	spiIRQHandler
 *
 * DESCRIPTION	:	This function is used to handle the SPI interrupt
 *
 * PARAMETERS	:	SPI_Handle_t *spiHandleStructure - SPI Handle Structure
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void spiIRQHandler(SPI_Handle_t *spiHandleStructure)
{
	//Transmit
	if((spiFlagCheck(spiHandleStructure->SPIx,SPI_TXE_FLAG)) && (spiFlagCheck(spiHandleStructure->SPIx,SPI_TXEIE_FLAG)))
	{
		spiTXInterruptHandle(spiHandleStructure);
	}

	//Receive
	if((spiFlagCheck(spiHandleStructure->SPIx,SPI_RXNE_FLAG)) && (spiFlagCheck(spiHandleStructure->SPIx,SPI_RXNEIE_FLAG)))
	{
		spiRXInterruptHandle(spiHandleStructure);
	}
}

/********************* Function Description *********************************
 * FUNCTION		:	spiTXInterruptHandle
 *
 * DESCRIPTION	:	This function is used to handle the SPI TXEIE interrupt
 *
 * PARAMETERS	:	SPI_Handle_t *spiHandleStructure - SPI Handle Structure
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void spiTXInterruptHandle(SPI_Handle_t *spiHandleStructure)
{
	//Configuration SPI to transmit only if direction is 1 line
	if(spiHandleStructure->Init.Direction == SPI_DIRECTION_1LINE)
	{
		//Setting the BIDIOE bit to transmit only
		REG_SET_VAL(spiHandleStructure->SPIx->CR1,0x1,0x1,SPI_CR1_BIDIOE_Pos);
	}


	if(spiHandleStructure->Init.DataSize == SPI_DATA_SIZE_16BIT) //if data is 16 bit
	{
		//Writing 16 bits of data to the Data Register
		REG_WRITE(spiHandleStructure->SPIx->DR, *(uint16_t*)spiHandleStructure->pTxData);\

		//Subtracting size of data by two as 16 bits of data has been transmitted
		spiHandleStructure->TxLen = spiHandleStructure->TxLen - 2;

		//Incrementing the TxData
		(uint16_t*)spiHandleStructure->pTxData++;
	}
	else if(spiHandleStructure->Init.DataSize == SPI_DATA_SIZE_8BIT)	//If data is 8 bit
	{
		//Writing 8 bits of data to the Data Register
		REG_WRITE(spiHandleStructure->SPIx->DR,*spiHandleStructure->pTxData);

		//Subtracting size of data by 1 as 8 bits of data has been transmitted
		spiHandleStructure->TxLen--;

		//Incrementing the TxData
		spiHandleStructure->pTxData++;
	}

	if(spiHandleStructure->TxLen == 0)	//When all the data is transmitted
	{
		//CLear the TXEIE Bit in the CR2 Register to diable the interrupt
		SPI_DISABLE_IT(spiHandleStructure,SPI_IT_TXE);

		//Change the pointer of the Txdata to NULL
		spiHandleStructure->pTxData = NULL;

		//Change the Txlen to 0
		spiHandleStructure->TxLen = 0;

		//Change the SPI Txstate to Ready
		spiHandleStructure->TxState = SPI_READY;

		//Disable the SPI Peripheral
		SPI_DISABLE(spiHandleStructure);
	}
}

/********************* Function Description *********************************
 * FUNCTION		:	spiRXInterruptHandle
 *
 * DESCRIPTION	:	This function is used to handle the SPI RXNEIE interrupt
 *
 * PARAMETERS	:	SPI_Handle_t *spiHandleStructure - SPI Handle Structure
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void spiRXInterruptHandle(SPI_Handle_t *spiHandleStructure)
{
	//Configuration SPI to receive only if direction is 1 line
	if(spiHandleStructure->Init.Direction == SPI_DIRECTION_1LINE || spiHandleStructure->Init.Mode == SPI_MODE_SLAVE)
	{
		if(spiHandleStructure->Init.Direction == SPI_DIRECTION_1LINE)
		{
			//Setting the BIDIOE bit to receive only
			REG_CLR_VAL(spiHandleStructure->SPIx->CR1,0x1,SPI_CR1_BIDIOE_Pos);
		}
	}

	if(spiHandleStructure->Init.DataSize == SPI_DATA_SIZE_16BIT) //if data is 16 bit
	{
		*((uint16_t*)spiHandleStructure->pRxData) = REG_READ(spiHandleStructure->SPIx->DR);

		//Subtracting size of data by two as 16 bits of data has been transmitted
		spiHandleStructure->RxLen = spiHandleStructure->RxLen - 2;

		//Incrementing the RxData
		(uint16_t*)spiHandleStructure->pRxData++;

	}
	else if(spiHandleStructure->Init.DataSize == SPI_DATA_SIZE_8BIT) //if data is 8 bit
	{
		*spiHandleStructure->pRxData = REG_READ(spiHandleStructure->SPIx->DR);

		//Subtracting size of data by 1 as 8 bits of data has been transmitted
		spiHandleStructure->RxLen--;

		//Incrementing the TxData
		spiHandleStructure->pRxData++;
	}

	if(spiHandleStructure->RxLen == 0)	//When all the data is received
	{
		//CLear the RXNEIE Bit in the CR2 Register to diable the interrupt
		SPI_DISABLE_IT(spiHandleStructure,SPI_IT_RXNE);

		//Change the pointer of the Rxdata to NULL
		spiHandleStructure->pRxData = NULL;

		//Change the Rxlen to 0
		spiHandleStructure->RxLen = 0;

		//Change the SPI Txstate to Ready
		spiHandleStructure->RxState = SPI_READY;

		//Disable the SPI Peripheral
		SPI_DISABLE(spiHandleStructure);
	}
}
