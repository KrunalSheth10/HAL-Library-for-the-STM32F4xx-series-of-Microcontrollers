/****************************************FILE DESCRIPTION**************************************/
/* FILE 		: flash.h
* PROJECT 		: HAL Library for STM32F4xx
* PROGRAMMER 	: Brijesh Mehta
* DESCRIPTION 	: Flash config
*/
/*********************************************************************************************/
#ifndef FLASH_H_
#define FLASH_H_

/****************Includes***********************/
#include <stdint.h>
#include "stm32f407xx.h"
#include "stm32f4xx.h"
#include "util.h"

/*
 * Macro Function to set flash latency
 * The possible values of Latency can be:
 * 		FLASH_ACR_LATENCY_0WS
 * 		FLASH_ACR_LATENCY_1WS
 * 		FLASH_ACR_LATENCY_2WS
 * 		FLASH_ACR_LATENCY_3WS
 * 		FLASH_ACR_LATENCY_4WS
 * 		FLASH_ACR_LATENCY_5WS
 * 		FLASH_ACR_LATENCY_6WS
 * 		FLASH_ACR_LATENCY_7WS
 */
#define FLASH_SET_LATENCY(flashLatency)			MODIFY_REG(FLASH->ACR,FLASH_ACR_LATENCY,flashLatency)

/*
 * Macro Function to get Flash Latency
 * The possible return values of Latency can be:
 * 		FLASH_ACR_LATENCY_0WS
 * 		FLASH_ACR_LATENCY_1WS
 * 		FLASH_ACR_LATENCY_2WS
 * 		FLASH_ACR_LATENCY_3WS
 * 		FLASH_ACR_LATENCY_4WS
 * 		FLASH_ACR_LATENCY_5WS
 * 		FLASH_ACR_LATENCY_6WS
 * 		FLASH_ACR_LATENCY_7WS
 */
#define FLASH_GET_LATENCY()			READ_BIT(FLASH->ACR,FLASH_ACR_LATENCY)

#endif /* FLASH_H_ */
