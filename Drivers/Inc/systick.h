/****************************************FILE DESCRIPTION**************************************/
/* FILE 		: systick.h
* PROJECT 		: HAL Library for STM32F4xx
* PROGRAMMER 	: Brijesh Mehta
* DESCRIPTION 	: systick header file
*/
/*********************************************************************************************/
#ifndef SYSTICK_H_
#define SYSTICK_H_

/************ Includes **********************/
#include "stm32f407xx.h"
#include "stm32f4xx.h"
#include "util.h"
#include "core_cm4.h"
#include "RCC.h"

/************* Function Prototype *********************/
void delay(uint32_t Delay);

void systickDeinit(void);

void systickInit(void);

uint32_t getTicks(void);

#endif /* SYSTICK_H_ */
