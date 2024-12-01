/****************************************FILE DESCRIPTION**************************************/
/* FILE 		: systick.c
* PROJECT 		: HAL Library for STM32F4xx
* PROGRAMMER 	: Brijesh Mehta
* DESCRIPTION 	: configure the systick
*/
/*********************************************************************************************/

/************ Includes **********************/
#include "systick.h"

/************ Variables ********************/
volatile uint32_t current_ticks;

/********************* Function Description *********************************
 * FUNCTION		:	delay
 *
 * DESCRIPTION	:	This function is used to generate a delay in ms
 *
 * PARAMETERS	:	uint32_t Delay - Value of delay in ms
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void delay(uint32_t Delay)
{
	uint32_t ticksStart = getTicks();

	while((getTicks() - ticksStart) < Delay);	//Waiting until ms is less than Delay
}

/********************* Function Description *********************************
 * FUNCTION		:	systickDeinit
 *
 * DESCRIPTION	:	This function disable the Systick
 *
 * PARAMETERS	:	void
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void systickDeinit(void)
{
	//Setting the Reload Value of Systick to zero
	MODIFY_REG(SysTick->LOAD, SysTick_LOAD_RELOAD_Msk, 0);

	//Disabling the Systick exceptions
	CLEAR_BIT(SysTick->CTRL,SysTick_CTRL_TICKINT_Msk);

	//Disabling the systick
	CLEAR_BIT(SysTick->CTRL,SysTick_CTRL_ENABLE_Msk);
}

/********************* Function Description *********************************
 * FUNCTION		:	systickInit
 *
 * DESCRIPTION	:	This function initializes the systick to generate an interrupt every 1ms
 *
 * PARAMETERS	:	void
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void systickInit(void)
{
	uint32_t sysClkFreq = 0;
	uint64_t reloadValue = 0;

	sysClkFreq = RCC_GetSysClockFreq();

	//reloadValue = System Clock / 1000 for 1 ms Delay
	reloadValue = (sysClkFreq / 1000) - 1;

	//Setting the Reload Value of Systick
	MODIFY_REG(SysTick->LOAD, SysTick_LOAD_RELOAD_Msk, reloadValue);

	//Selecting the clock source of systick
	SET_BIT(SysTick->CTRL,SysTick_CTRL_CLKSOURCE_Msk);

	//Enabling the Systick exceptions
	SET_BIT(SysTick->CTRL,SysTick_CTRL_TICKINT_Msk);

	//Setting the Interrupt Priority
	NVIC_SetPriority(SysTick_IRQn,0);

	//Enabling the Interrupt
	NVIC_EnableIRQ(SysTick_IRQn);

	//Enabling the systick
	SET_BIT(SysTick->CTRL,SysTick_CTRL_ENABLE_Msk);
}

/********************* Function Description *********************************
 * FUNCTION		:	getTicks
 *
 * DESCRIPTION	:	This returns current tick value
 *
 * PARAMETERS	:	void
 *
 * RETURN		:	uint32_t current tick values
 *
 * **************************************************************************/
uint32_t getTicks(void)
{
	return current_ticks;
}

/********************* Function Description *********************************
 * FUNCTION		:	SysTick_Handler
 *
 * DESCRIPTION	:	This is the Systick exception handler
 *
 * PARAMETERS	:	void
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void SysTick_Handler(void)
{
	current_ticks++;	//Incrementing ms variable every 1 ms
}
