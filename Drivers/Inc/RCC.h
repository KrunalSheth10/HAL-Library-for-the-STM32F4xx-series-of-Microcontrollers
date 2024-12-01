/****************************************FILE DESCRIPTION**************************************/
/* FILE 		: RCC.h
* PROJECT 		: HAL Library for STM32F4xx
* PROGRAMMER 	: Brijesh Mehta
* DESCRIPTION 	: RCC Config Structure definitions
*/
/*********************************************************************************************/

#ifndef RCC_H_
#define RCC_H_

/****************************************Maximum Clock Speeds Possible*****************************/
//System Clock = 168Mhz
//AHB Bus Clock = 168Mhz
//APB1 Bus Clock = 42Mhz
//APB2 Bus Clock = 84Mhz

/****************Includes***********************/
#include <stdint.h>
#include "stm32f407xx.h"
#include "stm32f4xx.h"
#include "util.h"
#include "flash.h"
#include "systick.h"

/**************** Macros *********************/
#define HSI_FREQ		((uint32_t)16000000U)
#define HSE_FREQ		((uint32_t)8000000U)

/*
 * Macro Function to identify the System Clock Source
 * The returned value can be :
 * 		RCC_CFGR_SWS_HSI	HSI oscillator used as system clock
 * 		RCC_CFGR_SWS_HSE	HSE oscillator used as system clock
 * 		RCC_CFGR_SWS_PLL	PLL used as system clock
 */
#define RCC_GET_SYSCLK_SRC()			(READ_BIT(RCC->CFGR,RCC_CFGR_SWS))

/*
 * Macro Function to idenitfy the PLL CLock Source
 * The returned value can be:
 * 		RCC_PLLCFGR_PLLSRC_HSE	HSE selected as PLL Clock Source
 * 		RCC_PLLCFGR_PLLSRC_HSI	HSI selected as PLL Clock Source
 */
#define RCC_GET_PLL_OSC_SOURCE()		(READ_BIT(RCC->PLLCFGR,RCC_PLLCFGR_PLLSRC))

/*
 * Macro Function to Enable the HSI CLock
 */
#define RCC_HSI_ENABLE()				(SET_BIT(RCC->CR,RCC_CR_HSION))

/*
 * Macro Function to Disable the HSI CLock
 */
#define RCC_HSI_DISABLE()				(CLEAR_BIT(RCC->CR,RCC_CR_HSION))

/*
 * Macro Function to Enable the HSE CLock
 */
#define RCC_HSE_ENABLE()				(SET_BIT(RCC->CR,RCC_CR_HSEON))

/*
 * Macro Function to Disable the HSE CLock
 */
#define RCC_HSE_DISABLE()				(CLEAR_BIT(RCC->CR,RCC_CR_HSEON))

/*
 * Macro Function to Enable the PLL CLock
 */
#define RCC_PLL_ENABLE()				(SET_BIT(RCC->CR,RCC_CR_PLLON))

/*
 * Macro Function to Disable the PLL CLock
 */
#define RCC_PLL_DISABLE()				(CLEAR_BIT(RCC->CR,RCC_CR_PLLON))

/*
 * Macro to enable GPIOA peripheral Clock
 */
#define RCC_GPIOA_CLOCK_ENABLE()		(SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOAEN))

/*
 * Macro to disable GPIOA peripheral Clock
 */
#define RCC_GPIOA_CLOCK_DISABLE()		(CLEAR_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOAEN))

/*
 * Macro to enable GPIOB peripheral Clock
 */
#define RCC_GPIOB_CLOCK_ENABLE()		(SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOBEN))

/*
 * Macro to disable GPIOB peripheral Clock
 */
#define RCC_GPIOB_CLOCK_DISABLE()		(CLEAR_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOBEN))

/*
 * Macro to enable GPIOC peripheral Clock
 */
#define RCC_GPIOC_CLOCK_ENABLE()		(SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOCEN))

/*
 * Macro to disable GPIOA peripheral Clock
 */
#define RCC_GPIOC_CLOCK_DISABLE()		(CLEAR_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOCEN))

/*
 * Macro to enable GPIOD peripheral Clock
 */
#define RCC_GPIOD_CLOCK_ENABLE()		(SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIODEN))

/*
 * Macro to disable GPIOA peripheral Clock
 */
#define RCC_GPIOD_CLOCK_DISABLE()		(CLEAR_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIODEN))

/*
 * Macro to enable SYSCFG peripheral Clock
 */
#define RCC_SYSCFG_CLOCK_ENABLE()		(SET_BIT(RCC->APB2ENR,RCC_APB2ENR_SYSCFGEN))

/*
 * Macro to enable SPI1 peripheral Clock
 */
#define RCC_SPI1_CLOCK_ENABLE()			(SET_BIT(RCC->APB2ENR,RCC_APB2ENR_SPI1EN))

/*
 * Macro to enable SPI2 peripheral Clock
 */
#define RCC_SPI2_CLOCK_ENABLE()			(SET_BIT(RCC->APB1ENR,RCC_APB1ENR_SPI2EN))

/*
 * Macro to enable SPI3 peripheral Clock
 */
#define RCC_SPI3_CLOCK_ENABLE()			(SET_BIT(RCC->APB1ENR,RCC_APB1ENR_SPI3EN))

/*
 * Macro to disable SPI1 peripheral Clock
 */
#define RCC_SPI1_CLOCK_DISABLE()			(CLEAR_BIT(RCC->APB2ENR,RCC_APB2ENR_SPI1EN))

/*
 * Macro to disable SPI2 peripheral Clock
 */
#define RCC_SPI2_CLOCK_DISABLE()			(CLEAR_BIT(RCC->APB1ENR,RCC_APB1ENR_SPI2EN))

/*
 * Macro to disable SPI3 peripheral Clock
 */
#define RCC_SPI3_CLOCK_DISABLE()			(CLEAR_BIT(RCC->APB1ENR,RCC_APB1ENR_SPI3EN))

/*
 * Macro to enable USART1 peripheral Clock
 */
#define RCC_USART1_CLOCK_ENABLE()			(SET_BIT(RCC->APB2ENR,RCC_APB2ENR_USART1EN))

/*
 * Macro to enable USART2 peripheral Clock
 */
#define RCC_USART2_CLOCK_ENABLE()			(SET_BIT(RCC->APB1ENR,RCC_APB1ENR_USART2EN))

/*
 * Macro to enable USART2 peripheral Clock
 */
#define RCC_USART3_CLOCK_ENABLE()			(SET_BIT(RCC->APB1ENR,RCC_APB1ENR_USART3EN))

/*
 * Macro to disable USART1 peripheral Clock
 */
#define RCC_USART1_CLOCK_DISABLE()			(CLEAR_BIT(RCC->APB2ENR,RCC_APB2ENR_USART1EN))

/*
 * Macro to disable USART2 peripheral Clock
 */
#define RCC_USART2_CLOCK_DISABLE()			(CLEAR_BIT(RCC->APB1ENR,RCC_APB1ENR_USART2EN))

/*
 * Macro to disable USART2 peripheral Clock
 */
#define RCC_USART3_CLOCK_DISABLE()			(CLEAR_BIT(RCC->APB1ENR,RCC_APB1ENR_USART3EN))

//RCC Oscillator Types	@OSCILLATOR_TYPES
#define RCC_OSCILLATOR_TYPE_HSI			0x00000001U
#define RCC_OSCILLATOR_TYPE_HSE			0x00000002U
#define RCC_OSCILLATOR_TYPE_PLL			0x00000008U

//RCC HSI State	@HSI_STATE
#define RCC_HSI_OFF						0x00000000U
#define RCC_HSI_ON						RCC_CR_HSION

//RCC HSE State	@HSE_STATE
#define RCC_HSE_OFF						0x00000000U
#define RCC_HSE_ON						RCC_CR_HSEON
#define RCC_HSE_BYPASS					RCC_CR_HSEBYP

//RCC PLL State	@PLL_STATE
#define RCC_PLL_OFF						0x00000000U
#define RCC_PLL_ON						RCC_CR_PLLON

//RCC PLL Clk Source	@PLL_CLK_SOURCE
#define RCC_PLL_SOURCE_HSI				RCC_PLLCFGR_PLLSRC_HSI
#define RCC_PLL_SOURCE_HSE				RCC_PLLCFGR_PLLSRC_HSE

//PLLM_Value can be from 0-63  @PLLM_VALUE

//PLLN_Value can be from 0-511	@PLLN_VALUE

//RCC PLLP Values	@PLLP_VALUE
#define RCC_PLLP_DIV2                  0   /*!< PLLP division factor = 2  */
#define RCC_PLLP_DIV4                  1   /*!< PLLP division factor = 4  */
#define RCC_PLLP_DIV6                  2   /*!< PLLP division factor = 6  */
#define RCC_PLLP_DIV8                  3   /*!< PLLP division factor = 8  */

//RCC Clock System Clock Type	@SYSTEM_CLOCK_TYPE
#define RCC_CLOCKTYPE_SYSCLK           0x00000001U   /*!< SYSCLK to configure */
#define RCC_CLOCKTYPE_HCLK             0x00000002U   /*!< HCLK to configure */
#define RCC_CLOCKTYPE_PCLK1            0x00000004U   /*!< PCLK1 to configure */
#define RCC_CLOCKTYPE_PCLK2            0x00000008U   /*!< PCLK2 to configure */

//RCC System Clock Source		@SYSTEM_CLOCK_SOURCE
#define RCC_SYSCLKSOURCE_HSI			RCC_CFGR_SW_HSI
#define RCC_SYSCLKSOURCE_HSE			RCC_CFGR_SW_HSE
#define RCC_SYSCLKSOURCE_PLL			RCC_CFGR_SW_PLL

//RCC AHB Clock Divider			@AHB_CLOCK_DIV
#define RCC_CFGR_HPRE_DIV1                 0x00000000U                         /*!< SYSCLK not divided    */
#define RCC_CFGR_HPRE_DIV2                 0x00000080U                         /*!< SYSCLK divided by 2   */
#define RCC_CFGR_HPRE_DIV4                 0x00000090U                         /*!< SYSCLK divided by 4   */
#define RCC_CFGR_HPRE_DIV8                 0x000000A0U                         /*!< SYSCLK divided by 8   */
#define RCC_CFGR_HPRE_DIV16                0x000000B0U                         /*!< SYSCLK divided by 16  */
#define RCC_CFGR_HPRE_DIV64                0x000000C0U                         /*!< SYSCLK divided by 64  */
#define RCC_CFGR_HPRE_DIV128               0x000000D0U                         /*!< SYSCLK divided by 128 */
#define RCC_CFGR_HPRE_DIV256               0x000000E0U                         /*!< SYSCLK divided by 256 */
#define RCC_CFGR_HPRE_DIV512               0x000000F0U                         /*!< SYSCLK divided by 512 */

//RCC APB1 Clock Divider		@APB1_CLOCK_DIV
#define RCC_CFGR_PPRE1_DIV1                0x00000000U                         /*!< HCLK not divided   */
#define RCC_CFGR_PPRE1_DIV2                0x00001000U                         /*!< HCLK divided by 2  */
#define RCC_CFGR_PPRE1_DIV4                0x00001400U                         /*!< HCLK divided by 4  */
#define RCC_CFGR_PPRE1_DIV8                0x00001800U                         /*!< HCLK divided by 8  */
#define RCC_CFGR_PPRE1_DIV16               0x00001C00U                         /*!< HCLK divided by 16 */

//RCC APB2 Clock Divider		@APB2_CLOCK_DIV
#define RCC_CFGR_PPRE2_DIV1                0x00000000U                         /*!< HCLK not divided   */
#define RCC_CFGR_PPRE2_DIV2                0x00008000U                         /*!< HCLK divided by 2  */
#define RCC_CFGR_PPRE2_DIV4                0x0000A000U                         /*!< HCLK divided by 4  */
#define RCC_CFGR_PPRE2_DIV8                0x0000C000U                         /*!< HCLK divided by 8  */
#define RCC_CFGR_PPRE2_DIV16               0x0000E000U                         /*!< HCLK divided by 16 */

//Timeout Values
#define HSI_TIMEOUT_VALUE          2U    /* 2 ms (minimum Tick + 1) */
#define HSE_TIMEOUT_VALUE          2U    /* 2 ms (minimum Tick + 1) */
#define PLL_TIMEOUT_VALUE          2U    /* 2 ms (minimum Tick + 1) */

/************* RCC PLL Oscillator Configuration Structure *******************/
typedef struct
{
	uint32_t pllState;  	//Values from @PLL_STATE
	uint32_t pllSource;		/*!< RCC_PLLSource: PLL entry clock source.
                            This parameter must be a value of @PLL_CLK_SOURCE */

	uint32_t pllM;			/*!< PLLM: Division factor for PLL VCO input clock.
								Values from @PLLM_VALUE */

	uint32_t pllN;			/*!< PLLN: Multiplication factor for PLL VCO output clock
								 Values from @PLLN_VALUE */

	uint32_t pllP;			/*!< PLLP: Division factor for SAI clock.
								 Values from @PLLP_VALUE */
}RCC_PLLInit_t;

/************** RCC Oscillator Configuration Structure *********************/
typedef struct
{
	uint32_t oscillatorType;	/*!< The oscillators to be configured.
                                      This parameter can be a value of @OSCILLATOR_TYPES*/

	uint32_t hsiState;			//Values from @HSI_STATE
	uint32_t hseState;			//Values from @HSE_STATE

	RCC_PLLInit_t pll;			//RCC_PLL_Init Structure

}RCC_OscInit_t;

/************* RCC System, AHB and APB Clock Configuration Structure *************/
typedef struct
{
	uint32_t clockType;			//Values from @SYSTEM_CLOCK_TYPE 	The clock that needs to be configured

	uint32_t sysClkSource;		/*!< The clock source used as system clock (SYSCLK).
                                     This parameter can be a value of @SYSTEM_CLOCK_SOURCE    */

	uint32_t ahbClkDivider;		/*!< The AHB clock (HCLK) divider. This clock is derived from the system clock (SYSCLK).
                                     This parameter can be a value of @AHB_CLOCK_DIV       */

	uint32_t apb1ClkDivider;	/*!< The APB1 clock (PCLK1) divider. This clock is derived from the AHB clock (HCLK).
                                     This parameter can be a value of @APB1_CLOCK_DIV */

	uint32_t apb2ClkDivider;	/*!< The APB2 clock (PCLK2) divider. This clock is derived from the AHB clock (HCLK).
                                     This parameter can be a value of @APB2_CLOCK_DIV */

}RCC_ClkInit_t;

/**************Function Prototype *************************/
Status_t RCC_OscillatorConfig(RCC_OscInit_t *RCC_OscInitStruct);

Status_t RCC_ClockConfig(RCC_ClkInit_t *RCC_ClkInitStruct, uint32_t flashLatency);

uint32_t RCC_GetSysClockFreq(void);

uint32_t RCC_GetHCLKFreq(void);

uint32_t RCC_GetPCLK1Freq(void);

uint32_t RCC_GetPCLK2Freq(void);

#endif /* RCC_H_ */
