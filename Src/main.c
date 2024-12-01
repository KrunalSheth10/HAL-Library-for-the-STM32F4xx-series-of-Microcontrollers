/****************************************FILE DESCRIPTION**************************************/
/* FILE 		: main.c
* PROJECT 		: HAL Library for STM32F4xx
* PROGRAMMER 	: Brijesh Mehta
* DESCRIPTION 	: main c file
*/
/*********************************************************************************************/

#include <stdint.h>
#include "main.h"

void systemClockConfig(void);

int main(void)
{
	systemClockConfig();

	systickInit();

	while(1)
	{

	}
}

void systemClockConfig(void)
{
	/*
	 * System Clock = 168Mhz
	 * AHB Clock Speed = 168Mhz
	 * APB1 Clock Speed = 42Mhz
	 * APB2 Clock Speed = 84Mhz
	 */

	RCC_OscInit_t RCC_OscInitStruct = {0};

	RCC_OscInitStruct.oscillatorType = RCC_OSCILLATOR_TYPE_HSE | RCC_OSCILLATOR_TYPE_PLL;
	RCC_OscInitStruct.hseState = RCC_HSE_ON;
	RCC_OscInitStruct.pll.pllSource = RCC_PLL_SOURCE_HSE;
	RCC_OscInitStruct.pll.pllState = RCC_PLL_ON;
	RCC_OscInitStruct.pll.pllM = 8U;
	RCC_OscInitStruct.pll.pllN = 336U;
	RCC_OscInitStruct.pll.pllP = RCC_PLLP_DIV2;

	if(RCC_OscillatorConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		ErrorHandler();
	}

	RCC_ClkInit_t RCC_ClkInitStruct = {0};

	RCC_ClkInitStruct.clockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.sysClkSource = RCC_SYSCLKSOURCE_PLL;
	RCC_ClkInitStruct.apb1ClkDivider = RCC_CFGR_PPRE1_DIV4;
	RCC_ClkInitStruct.apb2ClkDivider = RCC_CFGR_PPRE2_DIV2;

	if(RCC_ClockConfig(&RCC_ClkInitStruct,FLASH_ACR_LATENCY_5WS) != HAL_OK)
	{
		ErrorHandler();
	}
}

