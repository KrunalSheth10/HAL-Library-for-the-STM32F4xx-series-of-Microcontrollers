/****************************************FILE DESCRIPTION**************************************/
/* FILE 		: SPI.h
* PROJECT 		: HAL Library for STM32F4xx
* PROGRAMMER 	: Brijesh Mehta
* DESCRIPTION 	: SPI config
*/
/*********************************************************************************************/

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>
#include "stm32f407xx.h"
#include "stm32f4xx.h"
#include "util.h"
#include "flash.h"
#include "systick.h"

/*
 * Macro to enable SPI
 * __HANDLE__ specifies the SPI Handle.
 */
#define SPI_ENABLE(__HANDLE__)  SET_BIT((__HANDLE__)->SPIx->CR1, SPI_CR1_SPE)

/*
 * Macro to enable SPI
 * __HANDLE__ specifies the SPI Handle.
 */
#define SPI_DISABLE(__HANDLE__) CLEAR_BIT((__HANDLE__)->SPIx->CR1, SPI_CR1_SPE)

/*
 * Macro Function To enable Specific Interrupt
 * __HANDLE__ specifies the SPI Handle.
 * __INTERRUPT__ specifies the interrupt source to enable.
 *         This parameter can be one of the following values:
 *            @arg SPI_IT_TXE: Tx buffer empty interrupt enable
 *            @arg SPI_IT_RXNE: RX buffer not empty interrupt enable
 *            @arg SPI_IT_ERR: Error interrupt enable
 */
#define SPI_ENABLE_IT(__HANDLE__, __INTERRUPT__)   SET_BIT((__HANDLE__)->SPIx->CR2, (__INTERRUPT__))

/*
 * Macro Function To disable Specific Interrupt
 * __HANDLE__ specifies the SPI Handle.
 * __INTERRUPT__ specifies the interrupt source to enable.
 *         This parameter can be one of the following values:
 *            @arg SPI_IT_TXE: Tx buffer empty interrupt enable
 *            @arg SPI_IT_RXNE: RX buffer not empty interrupt enable
 *            @arg SPI_IT_ERR: Error interrupt enable
 */
#define SPI_DISABLE_IT(__HANDLE__, __INTERRUPT__)   CLEAR_BIT((__HANDLE__)->SPIx->CR2, (__INTERRUPT__))

//SPI_Mode
#define SPI_MODE_SLAVE				((uint8_t)0x00)
#define SPI_MODE_MASTER				((uint8_t)0x01)

//SPI_Direction
#define SPI_DIRECTION_2LINE					((uint8_t)0x00)		//Full Duplex
#define SPI_DIRECTION_1LINE					((uint8_t)0x01)		//Half Duplex

//SPI_Data_Size
#define SPI_DATA_SIZE_8BIT			((uint8_t)0x00)
#define SPI_DATA_SIZE_16BIT			((uint8_t)0x01)

//SPI_Clock_Polarity
#define SPI_POLARITY_LOW			((uint8_t)0x00)
#define SPI_POLARITY_HIGH			((uint8_t)0x01)

//SPI_Clock_Phase
#define SPI_PHASE_1EDGE				((uint8_t)0x00)		//Data capture will happen at the first edge of the clock
#define SPI_PHASE_2EDGE				((uint8_t)0x01)		//Data capture will happen at the second edge of the clock

//SPI_Slave_Select_management
#define SPI_NSS_SOFTWARE			((uint8_t)0x01)
#define SPI_NSS_HARDWARE			((uint8_t)0x00)

//SPI_BaudRate_Prescaler
#define SPI_BAUDRATE_PRESCALAR_2	((uint8_t)0x00)
#define SPI_BAUDRATE_PRESCALAR_4	((uint8_t)0x01)
#define SPI_BAUDRATE_PRESCALAR_8	((uint8_t)0x02)
#define SPI_BAUDRATE_PRESCALAR_16	((uint8_t)0x03)
#define SPI_BAUDRATE_PRESCALAR_32	((uint8_t)0x04)
#define SPI_BAUDRATE_PRESCALAR_64	((uint8_t)0x05)
#define SPI_BAUDRATE_PRESCALAR_128	((uint8_t)0x06)
#define SPI_BAUDRATE_PRESCALAR_256	((uint8_t)0x07)

//SPI_MSB_LSB_transmission
#define SPI_MSB_FIRST				((uint8_t)0x00)
#define SPI_LSB_FIRST				((uint8_t)0x01)

//SPI Interrupts
#define SPI_IT_TXE                  SPI_CR2_TXEIE
#define SPI_IT_RXNE                 SPI_CR2_RXNEIE
#define SPI_IT_ERR                  SPI_CR2_ERRIE

/************ SPI Flags Enum Definition ******************/
typedef enum
{
	SPI_RXNE_FLAG	=	0x00,
	SPI_TXE_FLAG	=	0x01,
	SPI_BSY_FLAG	=	0x02,
	SPI_OVR_FLAG	=	0x03,
	SPI_TXEIE_FLAG	=	0x04,
	SPI_RXNEIE_FLAG = 	0x05,
	SPI_ERRIE_FLAG	=	0x06

} SPI_Flag_t;

/************ SPI States **************************/
typedef enum
{
	SPI_READY			= 	0x00,
	SPI_BUSY_IN_RX		=	0x01,
	SPI_BUSY_IN_TX		=	0x02
}SPI_States_t;

/***************** SPI Configuration Structure Definition ******************/
typedef struct
{
	uint32_t Mode;                /*!< Specifies the SPI operating mode.
	                                     This parameter can be a value of @ref SPI_Mode */

	uint32_t Direction;           /*!< Specifies the SPI bidirectional mode state.
	                                     This parameter can be a value of @ref SPI_Direction */

	uint32_t DataSize;            /*!< Specifies the SPI data size.
	                                     This parameter can be a value of @ref SPI_Data_Size */

	uint32_t CLKPolarity;         /*!< Specifies the serial clock steady state.
	                                     This parameter can be a value of @ref SPI_Clock_Polarity */

	uint32_t CLKPhase;            /*!< Specifies the clock active edge for the bit capture.
	                                     This parameter can be a value of @ref SPI_Clock_Phase */

	uint32_t NSS;                 /*!< Specifies whether the NSS signal is managed by
	                                     hardware (NSS pin) or by software using the SSI bit.
	                                     This parameter can be a value of @ref SPI_Slave_Select_management */

	uint32_t BaudRatePrescaler;   /*!< Specifies the Baud Rate prescaler value which will be
	                                     used to configure the transmit and receive SCK clock.
	                                     This parameter can be a value of @ref SPI_BaudRate_Prescaler
	                                     @note The communication clock is derived from the master
	                                     clock. The slave clock does not need to be set. */

	uint32_t FirstBit;            /*!< Specifies whether data transfers start from MSB or LSB bit.
	                                     This parameter can be a value of @ref SPI_MSB_LSB_transmission */
}SPI_Init_t;

/******************* SPI Handle Structure Definition ********************/
typedef struct
{
	SPI_TypeDef 	*SPIx;	//Base address of the SPI Peripheral

	SPI_Init_t 		Init;	//SPI Init Structure

	/******************** Used for Interrupt Mode *******************/
	uint8_t 		*pTxData;	//Store the address of the buffer which stores the data to be transmitted

	uint8_t 		*pRxData;	//Store the address of the buffer which stores the received data

	uint32_t		TxLen;		//Store the Length of the data to transmitted

	uint32_t		RxLen;		//Store the Length of the data to be received

	uint8_t			TxState;	//Store Tx state. Possible values from SPI States

	uint8_t			RxState;	//Store Rx State. Possible values from SPI States

}SPI_Handle_t;

/*************** Function Prototype *******************************/
void spiInit(SPI_Handle_t *spiHandleStructure);

void spiClockControl(SPI_TypeDef *SPIx, uint8_t EnorDi);

uint8_t spiFlagCheck(SPI_TypeDef *SPIx, SPI_Flag_t flag);

Status_t spiTransmit(SPI_Handle_t *spiHandleStructure, uint8_t *pTxData, uint16_t sizeOfData, uint32_t timeout);
Status_t spiReceive(SPI_Handle_t *spiHandleStructure, uint8_t *pRxData, uint16_t sizeOfData, uint32_t timeout);
Status_t spiTransmitReceive(SPI_Handle_t *spiHandleStructure, uint8_t *pTxData, uint8_t *pRxData, uint16_t sizeOfData, uint32_t timeout);

void spiTransmitIT(SPI_Handle_t *spiHandleStructure, uint8_t *pTxData, uint16_t sizeOfData);
void spiReceiveIT(SPI_Handle_t *spiHandleStructure, uint8_t *pRxData, uint16_t sizeOfData);
void spiTransmitReceiveIT(SPI_Handle_t *spiHandleStructure, uint8_t *pTxData, uint8_t *pRxData, uint16_t sizeOfData);

void spiIRQHandler(SPI_Handle_t *spiHandleStructure);
void spiTXInterruptHandle(SPI_Handle_t *spiHandleStructure);
void spiRXInterruptHandle(SPI_Handle_t *spiHandleStructure);

#endif /* SPI_H_ */
