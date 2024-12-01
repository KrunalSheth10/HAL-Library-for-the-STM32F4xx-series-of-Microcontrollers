/****************************************FILE DESCRIPTION**************************************/
/* FILE 		: btnDebounce.h
* PROJECT 		: HAL Library for STM32F4xx
* PROGRAMMER 	: Brijesh Mehta
* DESCRIPTION 	: Header file for button Debounce
*/
/*********************************************************************************************/

#ifndef BTNDEBOUNCE_H_
#define BTNDEBOUNCE_H_

/******************* Includes *************************/
#include <stdint.h>
#include "stm32f407xx.h"
#include "stm32f4xx.h"
#include "util.h"
#include "systick.h"
#include "gpio.h"


/******************* Function Prototype ******************/
uint8_t debounceReadPin(uint8_t GPIO_Pin, GPIO_TypeDef *GPIOx, uint8_t stableInterval);



#endif /* BTNDEBOUNCE_H_ */
