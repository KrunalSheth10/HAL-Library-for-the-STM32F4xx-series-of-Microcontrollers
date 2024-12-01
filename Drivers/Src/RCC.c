/****************************************FILE DESCRIPTION**************************************/
/* FILE 		: RCC.c
* PROJECT 		: HAL Library for STM32F4xx
* PROGRAMMER 	: Brijesh Mehta
* DESCRIPTION 	: RCC Config Functions
*/
/*********************************************************************************************/

/************* INCLUDES ****************/
#include "RCC.h"

/********************* Function Description *********************************
 * FUNCTION		:	RCC_OscillatorConfig
 *
 * DESCRIPTION	:	This function is used to configure the various Oscillators
 *
 * PARAMETERS	:	RCC_OscInit_t *RCC_OscInitStruct - Pointer to the RCC_OscInit_t Structure
 *
 * RETURN		:	Status_t - Return HAL_OK, HAL_ERROR to help debug problems
 *
 * **************************************************************************/
Status_t RCC_OscillatorConfig(RCC_OscInit_t *RCC_OscInitStruct)
{
	uint32_t sysClkSrc, pllClkSrc;

	sysClkSrc = RCC_GET_SYSCLK_SRC();		//Identify the System Clock Source
	pllClkSrc = RCC_GET_PLL_OSC_SOURCE();	//Identify the PLL Clock Source

	/********************* HSI Configuration *******************/
	if((RCC_OscInitStruct->oscillatorType & RCC_OSCILLATOR_TYPE_HSI) == RCC_OSCILLATOR_TYPE_HSI)
	{
		//Check if HSI is used as system Clock source or as PLL Clock Source
		if(sysClkSrc == RCC_CFGR_SWS_HSI || ((sysClkSrc == RCC_CFGR_SWS_PLL) && (pllClkSrc == RCC_PLLCFGR_PLLSRC_HSI)))
		{
			//Cannot turn of HSI Oscillator if it is used as system clock or PLL Clock Source
			if(RCC_OscInitStruct->hsiState == RCC_HSI_OFF)
			{
				return HAL_ERROR;
			}
		}
		else
		{
			// If the HSI State is ON
			if(RCC_OscInitStruct->hsiState != RCC_HSI_OFF)
			{

				RCC_HSI_ENABLE();	//Enabling the HSI Clock

				//Waiting for the HSI Ready Flag to be set
				while(READ_BIT(RCC->CR,RCC_CR_HSIRDY) == 0U);
			}
			else	//If the HSI state is OFF
			{
				RCC_HSI_DISABLE();	//Disable the HSI Clock

				//Wait for the HSI Ready Flag to be cleared
				while(READ_BIT(RCC->CR,RCC_CR_HSIRDY) != 0U);
			}
		}
	}

	/********************* HSE Configuration ************************/
	if((RCC_OscInitStruct->oscillatorType & RCC_OSCILLATOR_TYPE_HSE) == RCC_OSCILLATOR_TYPE_HSE)
	{
		//Check if HSE is used as system Clock source or as PLL Clock Source
		if((sysClkSrc == RCC_CFGR_SWS_HSE) || ((sysClkSrc == RCC_CFGR_SWS_HSE) && (pllClkSrc == RCC_PLLCFGR_PLLSRC_HSE)))
		{
			//Cannot turn of HSE Oscillator if it is used as system clock or PLL Clock Source
			if(RCC_OscInitStruct->hsiState == RCC_HSI_OFF)
			{
				return HAL_ERROR;
			}
		}
		else
		{
			if(RCC_OscInitStruct->hseState == RCC_HSE_ON) //If HSE State is ON
			{
				RCC_HSE_ENABLE();	//Enabling HSE

				//Waiting for the HSE RDY Flag to be set
				while((READ_BIT(RCC->CR,RCC_CR_HSERDY)) == 0U);
			}
			else if(RCC_OscInitStruct->hseState == RCC_HSE_BYPASS)	//If HSE State is Bypass
			{
				SET_BIT(RCC->CR,RCC_CR_HSEBYP);
				RCC_HSE_ENABLE();	//Enabling HSE

				//Waiting for the HSE RDY Flag to be set
				while((READ_BIT(RCC->CR,RCC_CR_HSERDY)) == 0U);
			}
			else	//If HSE State is OF
			{
				RCC_HSE_DISABLE();

				//Waiting for the HSE RDY Flag to be Cleared
				while((READ_BIT(RCC->CR,RCC_CR_HSERDY)) != 0U);
			}
		}
	}

	/********************** PLL Configuration ************************/
	if((RCC_OscInitStruct->oscillatorType & RCC_OSCILLATOR_TYPE_PLL) == RCC_OSCILLATOR_TYPE_PLL)
	{
		if(RCC_OscInitStruct->pll.pllState == RCC_PLL_ON)
		{
			//Setting the PLL Clk Source
			if(RCC_OscInitStruct->pll.pllSource == RCC_PLL_SOURCE_HSI)
			{
				CLEAR_BIT(RCC->PLLCFGR,RCC_PLLCFGR_PLLSRC_HSI);
			}
			else if(RCC_OscInitStruct->pll.pllSource == RCC_PLL_SOURCE_HSE)
			{
				SET_BIT(RCC->PLLCFGR,RCC_PLLCFGR_PLLSRC_HSE);
			}

			//Configuring the PLLM Bit
			MODIFY_REG(RCC->PLLCFGR,RCC_PLLCFGR_PLLM_Msk,((RCC_OscInitStruct->pll.pllM) << RCC_PLLCFGR_PLLM_Pos));

			//Configuring the PLLN Bit
			MODIFY_REG(RCC->PLLCFGR,RCC_PLLCFGR_PLLN_Msk,((RCC_OscInitStruct->pll.pllN) << RCC_PLLCFGR_PLLN_Pos));

			//Configuring the PLLP Bit
			MODIFY_REG(RCC->PLLCFGR,RCC_PLLCFGR_PLLP_Msk,((RCC_OscInitStruct->pll.pllP) << RCC_PLLCFGR_PLLP_Pos));

			//Enabling the PLL Clock
			RCC_PLL_ENABLE();

			//Waiting for the PLL RDY Flag to be set
			while((READ_BIT(RCC->CR,RCC_CR_PLLRDY)) == 0U);
		}
		else
		{
			if(sysClkSrc != RCC_CFGR_SWS_PLL)
			{
				//Disabling the PLL
				RCC_PLL_DISABLE();

				//Waiting for the PLL RDY Flag to be cleared
				while((READ_BIT(RCC->CR,RCC_CR_PLLRDY)) != 0U);
			}
			else
			{
				return HAL_ERROR;
			}

		}
	}
	return HAL_OK;
}

/********************* Function Description *********************************
 * FUNCTION		:	RCC_ClockConfig
 *
 * DESCRIPTION	:	This function is used to configure the system and bus Clock Speed
 *
 * PARAMETERS	:	RCC_ClkInit_t *RCC_ClkInitStruct - Pointer to the RCC_ClkInit_t Structure
 * 					flashLatency values from :
 * 						FLASH_ACR_LATENCY_0WS          0x00000000U
						FLASH_ACR_LATENCY_1WS          0x00000001U
						FLASH_ACR_LATENCY_2WS          0x00000002U
 						FLASH_ACR_LATENCY_3WS          0x00000003U
 						FLASH_ACR_LATENCY_4WS          0x00000004U
						FLASH_ACR_LATENCY_5WS          0x00000005U
 						FLASH_ACR_LATENCY_6WS          0x00000006U
 						FLASH_ACR_LATENCY_7WS          0x00000007U
 *
 * RETURN		:	Status_t - Return HAL_OK, HAL_ERROR to help debug problems
 *
 * **************************************************************************/
Status_t RCC_ClockConfig(RCC_ClkInit_t *RCC_ClkInitStruct, uint32_t flashLatency)
{

	/******************* Flash Config ****************************/
	//If you are increasing the CPU Frequency
	if(flashLatency > FLASH_GET_LATENCY())
	{
		//Set the new latency value
		FLASH_SET_LATENCY(flashLatency);

		//Check if the Latency value has been changed or not
		if(FLASH_GET_LATENCY() != flashLatency)
		{
			return HAL_ERROR;
		}
	}

	/******************* System Clock Config ***********************/
	if((RCC_ClkInitStruct->clockType & RCC_CLOCKTYPE_SYSCLK) == RCC_CLOCKTYPE_SYSCLK)
	{
		//If SYSCLOCK is HSI
		if(RCC_ClkInitStruct->sysClkSource == RCC_SYSCLKSOURCE_HSI)
		{
			//Checking if HSI is ready or not
			if(READ_BIT(RCC->CR,RCC_CR_HSIRDY) == 0U)
			{
				return HAL_ERROR; //Return error if HSI is not ready
			}
			else
			{
				MODIFY_REG(RCC->CFGR,RCC_CFGR_SW,RCC_SYSCLKSOURCE_HSI); //Change Clock to HSI
			}

			//Wait until the System Clock Changes to HSI
			while(RCC_GET_SYSCLK_SRC() != RCC_CFGR_SWS_HSI);
		}
		else if(RCC_ClkInitStruct->sysClkSource == RCC_SYSCLKSOURCE_HSE)	//If SYSCLOCK is HSE
		{
			//Checking if HSE is ready or not
			if(READ_BIT(RCC->CR,RCC_CR_HSERDY) == 0U)
			{
				return HAL_ERROR;	//Return error if HSE is not ready
			}
			else
			{
				MODIFY_REG(RCC->CFGR,RCC_CFGR_SW,RCC_SYSCLKSOURCE_HSE); //Change Clock to HSE
			}

			//Wait until the System Clock Changes to HSE
			while(RCC_GET_SYSCLK_SRC() != RCC_CFGR_SWS_HSE);
		}
		else if(RCC_ClkInitStruct->sysClkSource == RCC_SYSCLKSOURCE_PLL)
		{
			//Checking if PLL is ready or not
			if(READ_BIT(RCC->CR,RCC_CR_PLLRDY) == 0U)
			{
				return HAL_ERROR;	//Return error if PLL is not ready
			}
			else
			{
				MODIFY_REG(RCC->CFGR,RCC_CFGR_SW,RCC_SYSCLKSOURCE_PLL);	//Change Clock source to PLL
			}

			//Wait until the System Clock Changes to PLL
			while(RCC_GET_SYSCLK_SRC() != RCC_CFGR_SWS_PLL);
		}
	}

	/******************* Flash Config ****************************/
	//If you are decreasing the CPU Frequency
	if(flashLatency < FLASH_GET_LATENCY())
	{
		//Set the new latency value
		FLASH_SET_LATENCY(flashLatency);

		//Check if the Latency value has been changed or not
		if(FLASH_GET_LATENCY() != flashLatency)
		{
			return HAL_ERROR;
		}
	}

	/******************* AHB(HCLK) Clock Config ***********************/
	if((RCC_ClkInitStruct->clockType & RCC_CLOCKTYPE_HCLK) == RCC_CLOCKTYPE_HCLK)
	{
		MODIFY_REG(RCC->CFGR,RCC_CFGR_HPRE,RCC_ClkInitStruct->ahbClkDivider);
	}

	/****************** APB1(PCLK1) Clock Config **********************/
	if((RCC_ClkInitStruct->clockType & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
	{
		MODIFY_REG(RCC->CFGR,RCC_CFGR_PPRE1,RCC_ClkInitStruct->apb1ClkDivider);
	}

	/****************** APB2(PCLK2) Clock Config **********************/
	if((RCC_ClkInitStruct->clockType & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
	{
		MODIFY_REG(RCC->CFGR,RCC_CFGR_PPRE2,RCC_ClkInitStruct->apb2ClkDivider);
	}

	return HAL_OK;
}

/********************* Function Description *********************************
 * FUNCTION		:	RCC_GetSysClockFreq
 *
 * DESCRIPTION	:	This function is used to know the system Clock Frequency
 *
 * PARAMETERS	:	void
 *
 * RETURN		:	returns the system Clock Frequency
 *
 * **************************************************************************/
uint32_t RCC_GetSysClockFreq(void)
{
	uint32_t sysclkfeq = 0U;
	uint32_t sysClkSrc, pllClkSrc;
	uint32_t pllvco, plln, pllp, pllm;

	sysClkSrc = RCC_GET_SYSCLK_SRC();		//Identify the System Clock Source
	pllClkSrc = RCC_GET_PLL_OSC_SOURCE();	//Identify the PLL Clock Source

	if(sysClkSrc == RCC_CFGR_SWS_HSI)	//If System clock is HSI
	{
		return HSI_FREQ;
	}
	else if(sysClkSrc == RCC_CFGR_SWS_HSE)	//If System Clock is HSE
	{
		return HSE_FREQ;
	}
	else if(sysClkSrc == RCC_CFGR_SWS_PLL)	//If System Clock is PLL
	{
		/* PLL_VCO = (HSE_VALUE or HSI_VALUE) * PLLN / PLLM
		   SYSCLK = PLL_VCO / PLLR
		*/

		if(pllClkSrc == RCC_PLLCFGR_PLLSRC_HSI)	//If HSI is Clock Source
		{
			pllm = ((READ_BIT(RCC->PLLCFGR,RCC_PLLCFGR_PLLM)) >> RCC_PLLCFGR_PLLM_Pos);
			plln = ((READ_BIT(RCC->PLLCFGR,RCC_PLLCFGR_PLLN)) >> RCC_PLLCFGR_PLLN_Pos);

			pllvco = ((HSI_FREQ) * (plln/pllm));

			pllp = ((READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos));

			sysclkfeq = pllvco / pllp;
		}
		else	//If HSE is Clock Source
		{
			pllm = ((READ_BIT(RCC->PLLCFGR,RCC_PLLCFGR_PLLM)) >> RCC_PLLCFGR_PLLM_Pos);
			plln = ((READ_BIT(RCC->PLLCFGR,RCC_PLLCFGR_PLLN)) >> RCC_PLLCFGR_PLLN_Pos);

			pllvco = ((HSE_FREQ) * (plln/pllm));

			pllp = ((READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos));

			switch(pllp)
			{
				case RCC_PLLP_DIV2 : sysclkfeq = pllvco / 2; break;
				case RCC_PLLP_DIV4 : sysclkfeq = pllvco / 4; break;
				case RCC_PLLP_DIV6 : sysclkfeq = pllvco / 6; break;
				case RCC_PLLP_DIV8 : sysclkfeq = pllvco / 8; break;
			}
		}
	}
	return sysclkfeq;
}

/********************* Function Description *********************************
 * FUNCTION		:	RCC_GetHCLKFreq
 *
 * DESCRIPTION	:	This function is used to know the HCLK Clock Frequency
 *
 * PARAMETERS	:	void
 *
 * RETURN		:	returns the HCLK Clock Frequency
 *
 * **************************************************************************/
uint32_t RCC_GetHCLKFreq(void)
{
	uint32_t hclkFreq = 0;
	uint32_t sysClkFreq = RCC_GetSysClockFreq();
	uint16_t clockDivider = (REG_READ(RCC->CFGR) & RCC_CFGR_HPRE);

	switch(clockDivider)
	{
		case RCC_CFGR_HPRE_DIV1 :
			hclkFreq = sysClkFreq / 1; break;
		case RCC_CFGR_HPRE_DIV2 :
			hclkFreq = sysClkFreq / 2; break;
		case RCC_CFGR_HPRE_DIV4 :
			hclkFreq = sysClkFreq / 4; break;
		case RCC_CFGR_HPRE_DIV8 :
			hclkFreq = sysClkFreq / 8; break;
		case RCC_CFGR_HPRE_DIV16 :
			hclkFreq = sysClkFreq / 16; break;
		case RCC_CFGR_HPRE_DIV64 :
			hclkFreq = sysClkFreq / 64; break;
		case RCC_CFGR_HPRE_DIV128 :
			hclkFreq = sysClkFreq / 128; break;
		case RCC_CFGR_HPRE_DIV256 :
			hclkFreq = sysClkFreq / 256; break;
		case RCC_CFGR_HPRE_DIV512 :
			hclkFreq = sysClkFreq / 512; break;
	}

	return hclkFreq;
}

/********************* Function Description *********************************
 * FUNCTION		:	RCC_GetPCLK1Freq
 *
 * DESCRIPTION	:	This function is used to know the PCLK1 Clock Frequency
 *
 * PARAMETERS	:	void
 *
 * RETURN		:	returns the PCLK1 Clock Frequency
 *
 * **************************************************************************/
uint32_t RCC_GetPCLK1Freq(void)
{
	uint32_t pclk1Freq = 0;
	uint32_t hclkFreq = RCC_GetHCLKFreq();
	uint16_t clockDivider = (REG_READ(RCC->CFGR) & RCC_CFGR_PPRE1);

	switch(clockDivider)
	{
		case RCC_CFGR_PPRE1_DIV1 :
			pclk1Freq = hclkFreq / 1; break;
		case RCC_CFGR_PPRE1_DIV2 :
			pclk1Freq = hclkFreq / 2; break;
		case RCC_CFGR_PPRE1_DIV4 :
			pclk1Freq = hclkFreq / 4; break;
		case RCC_CFGR_PPRE1_DIV8 :
			pclk1Freq = hclkFreq / 8; break;
		case RCC_CFGR_PPRE1_DIV16 :
			pclk1Freq = hclkFreq / 16; break;
	}

	return pclk1Freq;
}

/********************* Function Description *********************************
 * FUNCTION		:	RCC_GetPCLK2Freq
 *
 * DESCRIPTION	:	This function is used to know the PCLK1 Clock Frequency
 *
 * PARAMETERS	:	void
 *
 * RETURN		:	returns the PCLK1 Clock Frequency
 *
 * **************************************************************************/
uint32_t RCC_GetPCLK2Freq(void)
{
	uint32_t pclk2Freq = 0;
	uint32_t hclkFreq = RCC_GetHCLKFreq();
	uint16_t clockDivider = (REG_READ(RCC->CFGR) & RCC_CFGR_PPRE2);

	switch(clockDivider)
	{
		case RCC_CFGR_PPRE2_DIV1 :
			pclk2Freq = hclkFreq / 1; break;
		case RCC_CFGR_PPRE2_DIV2 :
			pclk2Freq = hclkFreq / 2; break;
		case RCC_CFGR_PPRE2_DIV4 :
			pclk2Freq = hclkFreq / 4; break;
		case RCC_CFGR_PPRE2_DIV8 :
			pclk2Freq = hclkFreq / 8; break;
		case RCC_CFGR_PPRE2_DIV16 :
			pclk2Freq = hclkFreq / 16; break;
	}

	return pclk2Freq;
}
