/****************************************FILE DESCRIPTION**************************************/
/* FILE 		: util.h
* PROJECT 		: HAL Library for STM32F4xx
* PROGRAMMER 	: Brijesh Mehta
* DESCRIPTION 	: Basic utility
*/
/*********************************************************************************************/
#ifndef UTIL_H_
#define UTIL_H_

/*****************Includes **********************/
#include "stm32f4xx.h"
#include <stdio.h>

/*
 * Bit Manipulation Macros
 */
#define REG_WRITE(reg, val)   					((reg) = (val))
#define REG_READ(reg)         					((reg))
#define REG_SET_BIT(reg,pos)   					((reg) |=  (1U << (pos)))
#define REG_CLR_BIT(reg,pos)    				((reg) &= ~(1U << (pos)))
#define REG_READ_BIT(reg,pos)    				((reg) &   (1U << (pos)))
#define REG_CLR_VAL(reg,clrmask,pos)   			((reg) &= ~((clrmask) << (pos)))
#define REG_SET_VAL(reg,val,setmask,pos) 		do {\
													REG_CLR_VAL(reg,setmask,pos);\
													((reg) |= ((val) << (pos))); \
												}while(0)


#define __weak   __attribute__((weak))

typedef enum
{
	HAL_OK 	 		=	0x00,
	HAL_ERROR 		=	0x01,
	HAL_TIMEOUT 	=	0x02
}Status_t;

/**************Function Prototype *********************/
void ErrorHandler(void);

#endif /* UTIL_H_ */
