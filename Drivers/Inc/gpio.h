/****************************************FILE DESCRIPTION**************************************/
/* FILE 		: gpio.h
* PROJECT 		: HAL Library for STM32F4xx
* PROGRAMMER 	: Brijesh Mehta
* DESCRIPTION 	: GPIO config
*/
/*********************************************************************************************/
#ifndef GPIO_H_
#define GPIO_H_

/************************ Includes *************************/
#include <stdint.h>
#include "stm32f407xx.h"
#include "stm32f4xx.h"
#include "util.h"
#include "flash.h"
#include "systick.h"

/*
 * Macro function to clear the EXTI line Interrupt for particular GPIO Pin
 */
#define GPIO_EXTI_CLEAR_IT(GPIO_Pin)			(REG_SET_BIT(EXTI->PR,GPIO_Pin))

/*
 * Macro function to identify which gpio pin has raised the interrupt
 */
#define GPIO_EXTI_GET_IT(GPIO_Pin)				(REG_READ_BIT(EXTI->PR,GPIO_Pin))

/*
 * Macro function to disable Exti interrupt mask
 */
#define GPIO_EXTI_DISABLE_IT_MASK(GPIO_Pin)		(REG_CLR_BIT(EXTI->IMR,GPIO_Pin))

/*
 * Macro function to enable Exti interrupt mask
 */
#define GPIO_EXTI_ENABLE_IT_MASK(GPIO_Pin)		(REG_SET_BIT(EXTI->IMR,GPIO_Pin))

//GPIO PIN State
typedef enum
{
	GPIO_PIN_RESET	=	0x00,
	GPIO_PIN_SET	=	0x01
}GPIO_PinState;

#define ENABLE		1
#define DISABLE		0

//GPIO_PINS
#define GPIO_PIN_0                 ((uint8_t)0x00)  /* Pin 0 selected    */
#define GPIO_PIN_1                 ((uint8_t)0x01)  /* Pin 1 selected    */
#define GPIO_PIN_2                 ((uint8_t)0x02)  /* Pin 2 selected    */
#define GPIO_PIN_3                 ((uint8_t)0x03)  /* Pin 3 selected    */
#define GPIO_PIN_4                 ((uint8_t)0x04)  /* Pin 4 selected    */
#define GPIO_PIN_5                 ((uint8_t)0x05)  /* Pin 5 selected    */
#define GPIO_PIN_6                 ((uint8_t)0x06)  /* Pin 6 selected    */
#define GPIO_PIN_7                 ((uint8_t)0x07)  /* Pin 7 selected    */
#define GPIO_PIN_8                 ((uint8_t)0x08)  /* Pin 8 selected    */
#define GPIO_PIN_9                 ((uint8_t)0x09)  /* Pin 9 selected    */
#define GPIO_PIN_10                ((uint8_t)0x0A)  /* Pin 10 selected   */
#define GPIO_PIN_11                ((uint8_t)0x0B)  /* Pin 11 selected   */
#define GPIO_PIN_12                ((uint8_t)0x0C)  /* Pin 12 selected   */
#define GPIO_PIN_13                ((uint8_t)0x0D)  /* Pin 13 selected   */
#define GPIO_PIN_14                ((uint8_t)0x0E)  /* Pin 14 selected   */
#define GPIO_PIN_15                ((uint8_t)0x0F)  /* Pin 15 selected   */

//GPIO_PULL
#define GPIO_NOPULL                 ((uint8_t)0x00)   /*!< No Pull-up or Pull-down activation  */
#define GPIO_PULLUP                 ((uint8_t)0x01)   /*!< Pull-up activation                  */
#define GPIO_PULLDOWN               ((uint8_t)0x02)   /*!< Pull-down activation                */

//GPIO_SPEED
#define GPIO_SPEED_LOW             	((uint8_t)0x00)  /*!< Low speed       */
#define GPIO_SPEED_MEDIUM          	((uint8_t)0x01)  /*!< Medium speed    */
#define GPIO_SPEED_HIGH            	((uint8_t)0x02)  /*!< High speed      */
#define GPIO_SPEED_VERY_HIGH       	((uint8_t)0x03)  /*!< Very high speed */

//GPIO Alternate Function Modes @GPIO_ALF
#define GPIO_AF0					((uint8_t)0x00)
#define GPIO_AF1					((uint8_t)0x01)
#define GPIO_AF2					((uint8_t)0x02)
#define GPIO_AF3					((uint8_t)0x03)
#define GPIO_AF4					((uint8_t)0x04)
#define GPIO_AF5					((uint8_t)0x05)
#define GPIO_AF6					((uint8_t)0x06)
#define GPIO_AF7					((uint8_t)0x07)
#define GPIO_AF8					((uint8_t)0x08)
#define GPIO_AF9					((uint8_t)0x09)
#define GPIO_AF10					((uint8_t)0x0A)
#define GPIO_AF11					((uint8_t)0x0B)
#define GPIO_AF12					((uint8_t)0x0C)
#define GPIO_AF13					((uint8_t)0x0D)
#define GPIO_AF14					((uint8_t)0x0E)
#define GPIO_AF15					((uint8_t)0x0F)

//GPIO_MODE
#define GPIO_MODE_INPUT					0
#define GPIO_MODE_ANALOG				1
#define GPIO_MODE_OUTPUT_PUSHPULL		2
#define GPIO_MODE_OUTPUT_OD				3
#define GPIO_MODE_ALTFUNC_PUSHPULL		4
#define GPIO_MODE_ALTFUNC_OD			5
#define GPIO_MODE_IT_RISING				6
#define GPIO_MODE_IT_FALLING			7
#define GPIO_MODE_IT_RISING_FALLING		8

/************** GPIO Configuration Structure *********************/
typedef struct
{
	uint32_t Pin;        /*!< Specifies the GPIO pins to be configured.
	                           This parameter can be any value of @ref GPIO_pins */

	uint32_t Mode;       /*!< Specifies the operating mode for the selected pins.
	                           This parameter can be a value of @ref GPIO_mode */

	uint32_t Pull;       /*!< Specifies the Pull-up or Pull-Down activation for the selected pins.
	                           This parameter can be a value of @ref GPIO_pull */

	uint32_t Speed;      /*!< Specifies the speed for the selected pins.
	                           This parameter can be a value of @GPIO_SPEED */

	uint32_t Alternate;  /*!< Peripheral to be connected to the selected pins
	                            This parameter can be a value of @GPIO_ALF */
}GPIO_Init_t;

/****************** Function Prototype *******************/
void GPIOCLockControl(GPIO_TypeDef *GPIOx, uint8_t EnorDi);

void gpioInit(GPIO_TypeDef *GPIOx, GPIO_Init_t *GPIO_InitStruct);

GPIO_PinState gpioReadPin(GPIO_TypeDef *GPIOx, uint8_t GPIO_Pin);

void gpioWritePin(GPIO_TypeDef *GPIOx, uint8_t GPIO_Pin, GPIO_PinState pinState);

void gpioTogglePin(GPIO_TypeDef *GPIOx, uint8_t GPIO_Pin);

void GPIO_EXTI_IRQHandler(uint8_t GPIO_Pin);

__weak void GPIO_EXTI_Callback(uint8_t GPIO_Pin);
#endif /* GPIO_H_ */
