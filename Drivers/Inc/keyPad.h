/****************************************FILE DESCRIPTION**************************************/
/* FILE 		: keyPad.h
* PROJECT 		: HAL Library for STM32F4xx
* PROGRAMMER 	: Brijesh Mehta
* DESCRIPTION 	: Configure keypad
*/
/*********************************************************************************************/

#ifndef KEYPAD_H_
#define KEYPAD_H_

/******************* Includes *****************************/
#include  <stdint.h>
#include "stm32f407xx.h"
#include "stm32f4xx.h"
#include "util.h"
#include "gpio.h"
#include "systick.h"
#include "btnDebounce.h"

/**************** Keyboard Type *******************************/
#define MATRIX_4_4		((uint8_t)0x01)

/******************* For 4x4 Keyboard Matrix ********************/
#ifdef MATRIX_4_4

#define ROWS			4
#define COLUMNS			4
#define KEYS			16

#define ROW_PORT		GPIOA
#define COL_PORT		GPIOB

extern const uint8_t R[4];

extern const uint8_t C[4];

extern const char keys[4][4];

#endif

/******************* Function Prototype *********************************/
void keyPadInitPolling(uint8_t keyBoardType);

void keyPadInitInterrupt(uint8_t keyBoardType);

char keyPadScanPolling(uint8_t keyBoardType);

char keyPadScanInterrupt(uint8_t keyBoardType, uint8_t GPIO_PIN);
#endif /* KEYPAD_H_ */
