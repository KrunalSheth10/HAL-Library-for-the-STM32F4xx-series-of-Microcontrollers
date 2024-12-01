/****************************************FILE DESCRIPTION**************************************/
/* FILE 		: btnDebounce.c
* PROJECT 		: HAL Library for STM32F4xx
* PROGRAMMER 	: Brijesh Mehta
* DESCRIPTION 	: Button Debounce Functions
*/
/*********************************************************************************************/

/*****************Includes*************************/
#include "btnDebounce.h"

/********************* Function Description *********************************
 * FUNCTION		:	debounceReadPin
 *
 * DESCRIPTION	:	This function is used to wait for a certain time period and check whether
 * 					the btn is pressed
 *
 * PARAMETERS	:	uint8_t GPIO_Pin - gpio pin to be checked
 * 					GPIO_TypeDef *GPIOx - GPIO Port
 * 					uint8_t stableInterval - Time period usually between 10-20ms
 *
 * RETURN		:	uint8_t - return the state of the pin set or reset
 *
 * **************************************************************************/
uint8_t debounceReadPin(uint8_t GPIO_Pin, GPIO_TypeDef *GPIOx, uint8_t stableInterval)
{
	GPIO_PinState pinState = GPIO_PIN_RESET;
	uint8_t pinStateWeAreLookingFor = 0;

	uint32_t msTimeStamp = getTicks();

	pinState = gpioReadPin(GPIOx, GPIO_Pin);

	if(pinState == GPIO_PIN_RESET)	//Changing the pinStateWeAreLookingFor according to the pinState
	{
		pinStateWeAreLookingFor = 0;
	}
	else
	{
		pinStateWeAreLookingFor = 1;
	}

	while(getTicks() < (msTimeStamp + stableInterval))	//At every 1ms checking the pinState
	{
		pinState = gpioReadPin(GPIOx, GPIO_Pin);

		if(pinState != pinStateWeAreLookingFor)
		{
			pinStateWeAreLookingFor = !pinStateWeAreLookingFor;	//Changing the pinStateWeAreLookingFor
			//Reseting the time stamp if the pinState is not the same as pinStateWeAreLookingFor
			msTimeStamp = getTicks();
		}
	}
	return pinStateWeAreLookingFor;
}
