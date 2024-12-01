/****************************************FILE DESCRIPTION**************************************/
/* FILE 		: UART.h
* PROJECT 		: HAL Library for STM32F4xx
* PROGRAMMER 	: Brijesh Mehta
* DESCRIPTION 	: UART config
*/
/*********************************************************************************************/

#ifndef UART_H_
#define UART_H_

/************** Includes ***********************/
#include <stdint.h>
#include "stm32f407xx.h"
#include "stm32f4xx.h"
#include "util.h"
#include "gpio.h"
#include "systick.h"

/*
 * Macro to  enable UART
 */
#define UART_ENABLE(__HANDLE__)                   ((__HANDLE__)->UARTx->CR1 |= USART_CR1_UE)

/** @brief  Disable UART.
  * @param  __HANDLE__ specifies the UART Handle.
  * @retval None
  */
#define UART_DISABLE(__HANDLE__)                  ((__HANDLE__)->UARTx->CR1 &= ~USART_CR1_UE)

//UART_Word_Length
#define UART_WORDLENGTH_8B				0
#define UART_WORDLENGTH_9B				1

//UART_Stop_Bits
#define UART_STOPBITS_1					0
#define UART_STOPBITS_2					2

//UART_Parity
#define UART_PARITY_NONE				2
#define UART_PARITY_EVEN				0
#define UART_PARITY_ODD					1

//UART_Mode
#define UART_MODE_RX					0
#define UART_MODE_TX					1
#define UART_MODE_TX_RX					2

//UART_Hardware_Flow_Control
#define UART_HWCONTROL_NONE				0
#define UART_HWCONTROL_RTS				1
#define UART_HWCONTROL_CTS				2
#define UART_HWCONTROL_RTS_CTS			3

//UART_BAUDRATE
#define UART_STD_BAUD_1200				1200
#define UART_STD_BAUD_2400				2400
#define UART_STD_BAUD_9600				9600
#define UART_STD_BAUD_19200 			19200
#define UART_STD_BAUD_38400 			38400
#define UART_STD_BAUD_57600 			57600
#define UART_STD_BAUD_115200 			115200
#define UART_STD_BAUD_230400 			230400
#define UART_STD_BAUD_460800 			460800
#define UART_STD_BAUD_921600 			921600

//UART_Over_Sampling
#define UART_OVERSAMPLING_16            0
#define UART_OVERSAMPLING_8				1

//UART FLAGS Enum
typedef enum
{
	UART_RXNE_FLAG		=	0x00,
	UART_TXE_FLAG		=	0x01,
	UART_TC_FLAG		=	0x02,
	UART_TCIE_FLAG		=	0x03,
	UART_TXEIE_FLAG 	= 	0x04,
	UART_RXNEIE_FLAG	=   0x05
}UART_Flag_t;

/************ UART States **************************/
typedef enum
{
	UART_READY			= 	0x00,
	UART_BUSY_IN_RX		=	0x01,
	UART_BUSY_IN_TX		=	0x02
}UART_States_t;

/************************* UART Init Structure **********************/
typedef struct
{
	  uint32_t BaudRate;                  /*!< This member configures the UART communication baud rate.
	                                           This parameter can be a value of @ref UART_BAUDRATE  */

	  uint32_t WordLength;                /*!< Specifies the number of data bits transmitted or received in a frame.
	                                           This parameter can be a value of @ref UART_Word_Length */

	  uint32_t StopBits;                  /*!< Specifies the number of stop bits transmitted.
	                                           This parameter can be a value of @ref UART_Stop_Bits */

	  uint32_t Parity;                    /*!< Specifies the parity mode.
	                                           This parameter can be a value of @ref UART_Parity
	                                           @note When parity is enabled, the computed parity is inserted
	                                                 at the MSB position of the transmitted data (9th bit when
	                                                 the word length is set to 9 data bits; 8th bit when the
	                                                 word length is set to 8 data bits). */

	  uint32_t Mode;                      /*!< Specifies whether the Receive or Transmit mode is enabled or disabled.
	                                           This parameter can be a value of @ref UART_Mode */

	  uint32_t HwFlowCtl;                 /*!< Specifies whether the hardware flow control mode is enabled or disabled.
	                                           This parameter can be a value of @ref UART_Hardware_Flow_Control */

	  uint32_t OverSampling;              /*!< Specifies whether the Over sampling 8 is enabled or disabled, to achieve higher speed (up to fPCLK/8).
	                                             This parameter can be a value of @ref UART_Over_Sampling */
}UART_Init_t;

/***************** UART Handle Structure ***********************/
typedef struct
{
	USART_TypeDef *UARTx;
	UART_Init_t uartInitStruct;

	/******************** Used for Interrupt Mode *******************/
	uint8_t 		*pTxData;	//Store the address of the buffer which stores the data to be transmitted

	uint8_t 		*pRxData;	//Store the address of the buffer which stores the received data

	uint32_t		TxLen;		//Store the Length of the data to transmitted

	uint32_t		RxLen;		//Store the Length of the data to be received

	uint8_t			TxState;	//Store Tx state. Possible values from UART States

	uint8_t			RxState;	//Store Rx State. Possible values from UART States
}UART_Handle_t;

/**************** Function Prototype *************************/
void uartClockControl(USART_TypeDef *UARTx, uint8_t EnorDi);
void uartInit(UART_Handle_t *uartHandleStruct);
void uartSetBaudRate(UART_Handle_t *uartHandleStruct);

uint8_t uartFlagCheck(USART_TypeDef *SPIx, UART_Flag_t flag);

Status_t uartWaitOnFlagUntilTimeout(UART_Handle_t *uartHandleStruct, uint32_t Flag, uint32_t Timeout);

Status_t uartTransmit(UART_Handle_t *uartHandleStruct, uint8_t *pTxData, uint16_t sizeOfData, uint32_t Timeout);
Status_t uartReceive(UART_Handle_t *uartHandleStruct, uint8_t *pRxData, uint16_t sizeOfData, uint32_t Timeout);

void uartTransmitIT(UART_Handle_t *uartHandleStruct, uint8_t *pTxData, uint16_t sizeOfData);
void uartReceiveIT(UART_Handle_t *uartHandleStruct, uint8_t *pRxData, uint16_t sizeOfData);

void uartIRQHandler(UART_Handle_t *uartHandleStruct);
void uartTXInterruptHandle(UART_Handle_t *uartHandleStruct);
void uartRXInterruptHandle(UART_Handle_t *uartHandleStruct);
void uartTCInterruptHandle(UART_Handle_t *uartHandleStruct);

#endif /* UART_H_ */
