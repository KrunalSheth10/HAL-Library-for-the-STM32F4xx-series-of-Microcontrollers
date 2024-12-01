/****************************************FILE DESCRIPTION**************************************/
/* FILE 		: gpio.c
* PROJECT 		: HAL Library for STM32F4xx
* PROGRAMMER 	: Brijesh Mehta
* DESCRIPTION 	: GPIO config functions
*/
/*********************************************************************************************/

/*****************Includes*************************/
#include "gpio.h"

/********************* Function Description *********************************
 * FUNCTION		:	gpioInit
 *
 * DESCRIPTION	:	This function is used to configure the GPIO
 *
 * PARAMETERS	:	GPIO_TypeDef *GPIOx - GPIO Port
 * 					GPIO_Init_t *GPIO_InitStruct - GPIO Init Structure
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void gpioInit(GPIO_TypeDef *GPIOx, GPIO_Init_t *GPIO_InitStruct)
{
	/*******************Configuring the Mode of GPIO***********************/
	if(((GPIO_InitStruct->Mode) == GPIO_MODE_OUTPUT_PUSHPULL) || ((GPIO_InitStruct->Mode) == GPIO_MODE_OUTPUT_OD))	//Checking if mode is output
	{
		//Setting the GPIO to Output Mode
		REG_SET_VAL(GPIOx->MODER,0x1,0x3,(2*(GPIO_InitStruct->Pin)));

		if(((GPIO_InitStruct->Mode) == GPIO_MODE_OUTPUT_PUSHPULL))	//Checking if output mode is push pull
		{
			//Setting GPIO to PushPull
			REG_SET_VAL(GPIOx->OTYPER,0x0,0x1,GPIO_InitStruct->Pin);
		}
		else if(((GPIO_InitStruct->Mode) == GPIO_MODE_OUTPUT_OD))	//Checking if output mode is Open Drain
		{
			//Setting the GPIO to Open Drain
			REG_SET_VAL(GPIOx->OTYPER,0x1,0x1,GPIO_InitStruct->Pin);
		}
	}
	else if(((GPIO_InitStruct->Mode) == GPIO_MODE_ALTFUNC_PUSHPULL) || ((GPIO_InitStruct->Mode) == GPIO_MODE_ALTFUNC_OD))	//Checking if mode is Alternate Function
	{
		//Setting the GPIO to Alternate Function Mode
		REG_SET_VAL(GPIOx->MODER,0x2,0x3,(2*(GPIO_InitStruct->Pin)));

		if(((GPIO_InitStruct->Mode) == GPIO_MODE_ALTFUNC_PUSHPULL))	//Checking if output mode is push pull
		{
			//Setting GPIO to PushPull
			REG_SET_VAL(GPIOx->OTYPER,0x0,0x1,GPIO_InitStruct->Pin);
		}
		else if(((GPIO_InitStruct->Mode) == GPIO_MODE_ALTFUNC_OD))	//Checking if output mode is Open Drain
		{
			//Setting the GPIO to Open Drain
			REG_SET_VAL(GPIOx->OTYPER,0x1,0x1,GPIO_InitStruct->Pin);
		}

		/******************** Configuring the Alternate Functionality *****************/
		if(GPIO_InitStruct->Pin <= 7)	//For GPIO PIN 0-7
		{
			//Configure the AFRL Register
			REG_SET_VAL(GPIOx->AFR[0],GPIO_InitStruct->Alternate,0xF,(4*(GPIO_InitStruct->Pin)));
		}
		else if(GPIO_InitStruct->Pin >= 8 && GPIO_InitStruct->Pin <= 15)	//For GPIO PIN 8-15
		{
			//Configure the AFRL Register
			REG_SET_VAL(GPIOx->AFR[1],GPIO_InitStruct->Alternate,0xF,((4*(GPIO_InitStruct->Pin))%32));
		}
	}
	else if(((GPIO_InitStruct->Mode) == GPIO_MODE_INPUT) || ((GPIO_InitStruct->Mode) == GPIO_MODE_IT_RISING) || ((GPIO_InitStruct->Mode) == GPIO_MODE_IT_FALLING) || ((GPIO_InitStruct->Mode) == GPIO_MODE_IT_RISING_FALLING))	//If the mode is Input
	{
		REG_SET_VAL(GPIOx->MODER,0x0,0x3,(2*(GPIO_InitStruct->Pin)));	//Setting the Mode to Input

		/***************** Configuring Interrupts **************************/
		if(((GPIO_InitStruct->Mode) == GPIO_MODE_IT_RISING) || ((GPIO_InitStruct->Mode) == GPIO_MODE_IT_FALLING) || ((GPIO_InitStruct->Mode) == GPIO_MODE_IT_RISING_FALLING))
		{
			/*
			 * Configuring the Trigger Detection
			 */
			if(((GPIO_InitStruct->Mode) == GPIO_MODE_IT_RISING))	//If interrupt is for rising edge
			{
				REG_SET_VAL(EXTI->RTSR,0x1,0x1,GPIO_InitStruct->Pin);	//Setting the bit inside the EXTI_RTSR Reg
			}
			else if(((GPIO_InitStruct->Mode) == GPIO_MODE_IT_FALLING))	//If interrupt is for falling edge
			{
				REG_SET_VAL(EXTI->FTSR,0x1,0x1,GPIO_InitStruct->Pin);	//Setting the bit inside the EXTI_FTSR Reg
			}
			else if(((GPIO_InitStruct->Mode) == GPIO_MODE_IT_RISING_FALLING))	//If interrupt is for both rising and falling
			{
				REG_SET_VAL(EXTI->RTSR,0x1,0x1,GPIO_InitStruct->Pin);	//Setting the bit inside the EXTI_RTSR Reg
				REG_SET_VAL(EXTI->FTSR,0x1,0x1,GPIO_InitStruct->Pin);	//Setting the bit inside the EXTI_FTSR Reg
			}

			/*
			 * Configuring the SYSCFG EXTI Registers
			 * This will tell the EXTI to listen from which port & pin and forward their interrupt
			 * to the NVIC.
			 *
			 */

			RCC_SYSCFG_CLOCK_ENABLE();	//Enabling the SYSCFG CLOCK
			uint8_t gpioPort;
			//Finding the Appropriate Port Value
			if(GPIOx == GPIOA)
			{
				gpioPort = 0x00;
			}
			else if(GPIOx == GPIOB)
			{
				gpioPort = 0x01;
			}
			else if(GPIOx == GPIOC)
			{
				gpioPort = 0x02;
			}
			else if(GPIOx == GPIOD)
			{
				gpioPort = 0x03;
			}
			else if(GPIOx == GPIOE)
			{
				gpioPort = 0x04;
			}

			//Making changes in the SYSCFG EXTICR Register depending on the GPIO Pin Number
			if(GPIO_InitStruct->Pin >=0 || GPIO_InitStruct->Pin <= 3)
			{
				REG_SET_VAL(SYSCFG->EXTICR[0],gpioPort,0xF,(((GPIO_InitStruct->Pin) % 4) * 4));
			}
			else if(GPIO_InitStruct->Pin >=4 || GPIO_InitStruct->Pin <= 7)
			{
				REG_SET_VAL(SYSCFG->EXTICR[1],gpioPort,0xF,(((GPIO_InitStruct->Pin) % 4) * 4));
			}
			else if(GPIO_InitStruct->Pin >=8 || GPIO_InitStruct->Pin <= 11	)
			{
				REG_SET_VAL(SYSCFG->EXTICR[2],gpioPort,0xF,(((GPIO_InitStruct->Pin) % 4) * 4));
			}
			else if(GPIO_InitStruct->Pin >=12 || GPIO_InitStruct->Pin <= 15)
			{
				REG_SET_VAL(SYSCFG->EXTICR[3],gpioPort,0xF,(((GPIO_InitStruct->Pin) % 4) * 4));
			}

			/*
			 * //Enabling the EXTI interrupt delivery using IMR
			 */

			REG_SET_VAL(EXTI->IMR,0x1,0x1,GPIO_InitStruct->Pin);
		}
	}
	else	//The mode will Analog
	{
		REG_SET_VAL(GPIOx->MODER,GPIO_InitStruct->Mode,0x3,(2*(GPIO_InitStruct->Pin)));
	}

	/****************** Configuring Pull-up or Pull-down ************************/
	REG_SET_VAL(GPIOx->PUPDR,GPIO_InitStruct->Pull,0x3,(2*(GPIO_InitStruct->Pin)));

	/***************** Configuring the Speed *************************/
	REG_SET_VAL(GPIOx->OSPEEDR,GPIO_InitStruct->Speed,0x3,(2*(GPIO_InitStruct->Pin))); //Should be configured only if GPIO is in output mode
}

/********************* Function Description *********************************
 * FUNCTION		:	gpioReadPin
 *
 * DESCRIPTION	:	This function is used to read the GPIO Pin
 *
 * PARAMETERS	:	GPIO_TypeDef *GPIOx - GPIO Port
 * 					uint8_t GPIO_Pin - GPIO Pin No
 *
 * RETURN		:	GPIO_PinState - Return GPIO_PIN_RESET, GPIO_PIN_SET
 *
 * **************************************************************************/
GPIO_PinState gpioReadPin(GPIO_TypeDef *GPIOx, uint8_t GPIO_Pin)
{
	GPIO_PinState bitStatus;

	if((READ_BIT(GPIOx->IDR,(1<<GPIO_Pin)) >> GPIO_Pin) == GPIO_PIN_SET)
	{
		bitStatus = GPIO_PIN_SET;
	}
	else
	{
		bitStatus = GPIO_PIN_RESET;
	}
	return bitStatus;
}

/********************* Function Description *********************************
 * FUNCTION		:	gpioWritePin
 *
 * DESCRIPTION	:	This function is used to write to the GPIO Pin
 *
 * PARAMETERS	:	GPIO_TypeDef *GPIOx - GPIO Port
 * 					uint8_t GPIO_Pin - GPIO Pin No
 * 					GPIO_PinState pinState - Can be GPIO_PIN_RESET, GPIO_PIN_SET
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void gpioWritePin(GPIO_TypeDef *GPIOx, uint8_t GPIO_Pin, GPIO_PinState pinState)
{
	if(pinState == GPIO_PIN_SET)
	{
		GPIOx->BSRR = (uint32_t)(1 << GPIO_Pin);
	}
	else
	{
		GPIOx->BSRR = (uint32_t)(1 << (GPIO_Pin + 16));
	}
}

/********************* Function Description *********************************
 * FUNCTION		:	gpioTogglePin
 *
 * DESCRIPTION	:	This function is used to toggle the GPIO Pin
 *
 * PARAMETERS	:	GPIO_TypeDef *GPIOx - GPIO Port
 * 					uint8_t GPIO_Pin - GPIO Pin No
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void gpioTogglePin(GPIO_TypeDef *GPIOx, uint8_t GPIO_Pin)
{
	GPIO_PinState bitStatus;
	bitStatus = (READ_BIT(GPIOx->ODR,(1 << GPIO_Pin)) >> GPIO_Pin);


	if(bitStatus == GPIO_PIN_SET)
	{
		gpioWritePin(GPIOx,GPIO_Pin,GPIO_PIN_RESET);
	}
	else
	{
		gpioWritePin(GPIOx,GPIO_Pin,GPIO_PIN_SET);
	}
}

/********************* Function Description *********************************
 * FUNCTION		:	GPIO_EXTI_IRQHandler
 *
 * DESCRIPTION	:	This function is used to handle the GPIO Interrupt
 *
 * PARAMETERS	:	uint8_t GPIO_Pin
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void GPIO_EXTI_IRQHandler(uint8_t GPIO_Pin)
{

	if(GPIO_EXTI_GET_IT(GPIO_Pin) != 0x00u)
	{
		GPIO_EXTI_CLEAR_IT(GPIO_Pin);
		GPIO_EXTI_Callback(GPIO_Pin);	//Callback Function to perform certain tasks
	}
}

/********************* Function Description *********************************
 * FUNCTION		:	GPIOCLockControl
 *
 * DESCRIPTION	:	This function is used to enable or disable the GPIO Clock
 *
 * PARAMETERS	:	GPIO_TypeDef *GPIOx - GPIO Port
 * 					uint8_t EnorDi - Enable or Disable
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void GPIOCLockControl(GPIO_TypeDef *GPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (GPIOx == GPIOA)
		{
			RCC_GPIOA_CLOCK_ENABLE();
		}
		else if (GPIOx == GPIOB)
		{
			RCC_GPIOB_CLOCK_ENABLE();
		}
		else if (GPIOx == GPIOC)
		{
			RCC_GPIOC_CLOCK_ENABLE();
		}
		else if (GPIOx == GPIOD)
		{
			RCC_GPIOD_CLOCK_ENABLE();
		}
	}
	else if (EnorDi == DISABLE)
	{
		if (GPIOx == GPIOA)
		{
			RCC_GPIOA_CLOCK_DISABLE();
		}
		else if (GPIOx == GPIOB)
		{
			RCC_GPIOB_CLOCK_DISABLE();
		}
		else if (GPIOx == GPIOC)
		{
			RCC_GPIOC_CLOCK_DISABLE();
		}
		else if (GPIOx == GPIOD)
		{
			RCC_GPIOD_CLOCK_DISABLE();
		}
	}
}
