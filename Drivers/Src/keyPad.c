/****************************************FILE DESCRIPTION**************************************/
/* FILE 		: keyPad.c
* PROJECT 		: HAL Library for STM32F4xx
* PROGRAMMER 	: Brijesh Mehta
* DESCRIPTION 	: Configure keypad functions
*/
/*********************************************************************************************/

/*****************Includes*************************/
#include "keyPad.h"

/****************** 4x4 Keyboard Matrix Global Variables *************************/
const uint8_t R[4] = {GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_2,GPIO_PIN_3};

const uint8_t C[4] = {GPIO_PIN_7,GPIO_PIN_6,GPIO_PIN_5,GPIO_PIN_4};

const char keys[4][4] = {{'1','2','3','A'},
						{'4','5','6','B'},
						{'7','8','9','C'},
						{'*','0','#','D'}};

/********************* Function Description *********************************
 * FUNCTION		:	keyPadInitPolling
 *
 * DESCRIPTION	:	This function is used to configure the GPIO pins for the keyboard in polling mode
 * 					The columns are configured as Outputs
 * 					The Rows are configured as Inputs
 *
 * PARAMETERS	:	uint8_t keyBoardType
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void keyPadInitPolling(uint8_t keyBoardType)
{
	//Enable the Clock for the particular port
	GPIOCLockControl(ROW_PORT, ENABLE);
	GPIOCLockControl(COL_PORT, ENABLE);

	GPIO_Init_t gpioInitStructure = {0};

	if(keyBoardType == MATRIX_4_4)
	{
		//Configuring the Rows as Input
		for(uint8_t i = 0; i< ROWS; i++)
		{
			gpioInitStructure.Pin = R[i];
			gpioInitStructure.Mode = GPIO_MODE_INPUT;
			gpioInitStructure.Pull = GPIO_PULLDOWN;

			gpioInit(ROW_PORT, &gpioInitStructure);
		}

		//Configuring the Columns as Output
		for(uint8_t j = 0; j< COLUMNS; j++)
		{
			gpioInitStructure.Pin = C[j];
			gpioInitStructure.Mode = GPIO_MODE_OUTPUT_PUSHPULL;
			gpioInitStructure.Pull = GPIO_NOPULL;
			gpioInitStructure.Speed = GPIO_SPEED_LOW;

			gpioInit(COL_PORT, &gpioInitStructure);
		}

		//Setting the Columns to High
		for(uint8_t k = 0; k< COLUMNS; k++)
		{
			gpioWritePin(COL_PORT, C[k], GPIO_PIN_SET);
		}

	}
}

/********************* Function Description *********************************
 * FUNCTION		:	keyPadInitInterrupt
 *
 * DESCRIPTION	:	This function is used to configure the GPIO pins for the keyboard in interrupt mode
 * 					The columns are configured as Outputs
 * 					The Rows are configured as Inputs (With rising edge interrupts)
 *
 * PARAMETERS	:	uint8_t keyBoardType
 *
 * RETURN		:	void
 *
 * **************************************************************************/
void keyPadInitInterrupt(uint8_t keyBoardType)
{
	//Enable the Clock for the particular port
	GPIOCLockControl(ROW_PORT, ENABLE);
	GPIOCLockControl(COL_PORT, ENABLE);

	GPIO_Init_t gpioInitStructure = {0};

	if(keyBoardType == MATRIX_4_4)
	{
		//Configuring the Rows as Input
		for(uint8_t i = 0; i< ROWS; i++)
		{
			gpioInitStructure.Pin = R[i];
			gpioInitStructure.Mode = GPIO_MODE_IT_RISING;
			gpioInitStructure.Pull = GPIO_PULLDOWN;

			gpioInit(ROW_PORT, &gpioInitStructure);
		}

		//Configuring the Columns as Output
		for(uint8_t j = 0; j< COLUMNS; j++)
		{
			gpioInitStructure.Pin = C[j];
			gpioInitStructure.Mode = GPIO_MODE_OUTPUT_PUSHPULL;
			gpioInitStructure.Pull = GPIO_NOPULL;
			gpioInitStructure.Speed = GPIO_SPEED_LOW;

			gpioInit(COL_PORT, &gpioInitStructure);
		}

		//Setting the Columns to High
		for(uint8_t k = 0; k< COLUMNS; k++)
		{
			gpioWritePin(COL_PORT, C[k], GPIO_PIN_SET);
		}
	}
}


/********************* Function Description *********************************
 * FUNCTION		:	keyPadScanPolling
 *
 * DESCRIPTION	:	This function is used to scan and identify the key press and letter
 *
 * PARAMETERS	:	uint8_t keyBoardType
 *
 * RETURN		:	char - return the letter that is pressed
 *
 * **************************************************************************/
char keyPadScanPolling(uint8_t keyBoardType)
{
	uint8_t pinState = 0;
	uint8_t row = 0;
	uint8_t col = 0;

	//Scanning the rows to identify the row in which the button is pressed
	for(row = 0; row< ROWS; row++)
	{
		pinState = debounceReadPin(R[row], ROW_PORT, 20);	//Reading the state of the Row
		if(pinState == 1)
		{
			break;
		}
	}

	//Making each Column Low and checking if the Row is High or not
	for(col = 0; col<COLUMNS; col++)
	{
		gpioWritePin(COL_PORT, C[col], GPIO_PIN_RESET);
		if((gpioReadPin(ROW_PORT, R[row])) == 0)
		{
			gpioWritePin(COL_PORT, C[col], GPIO_PIN_SET);	//Again making the Column High
			break;
		}
		gpioWritePin(COL_PORT, C[col], GPIO_PIN_SET);	//Again making the Column High
	}
	return keys[row][col];
}

/********************* Function Description *********************************
 * FUNCTION		:	keyPadScanInterrupt
 *
 * DESCRIPTION	:	This function is used to scan and identify the key press and letter
 *
 * PARAMETERS	:	uint8_t keyBoardType
 *
 * RETURN		:	char - return the letter that is pressed
 *
 * **************************************************************************/
char keyPadScanInterrupt(uint8_t keyBoardType, uint8_t GPIO_PIN)
{
	uint8_t row = 0;
	uint8_t col = 0;

	//Setting the Columns Low
	for(uint8_t k = 0; k< COLUMNS; k++)
	{
		gpioWritePin(COL_PORT, C[k], GPIO_PIN_RESET);
	}

	//Scanning the rows to identify the row in which the button is pressed
	for(row = 0; row< ROWS; row++)
	{
		if(GPIO_PIN == R[row])
		{
			break;
		}
	}

	//Making each Column High and checking if the Row is High or not
	for(col = 0; col<COLUMNS; col++)
	{
		gpioWritePin(COL_PORT, C[col], GPIO_PIN_SET);
		if((gpioReadPin(ROW_PORT, R[row])) == 1)
		{
			break;
		}
	}

	//Setting the Columns to High
	for(uint8_t k = 0; k< COLUMNS; k++)
	{
		gpioWritePin(COL_PORT, C[k], GPIO_PIN_SET);
	}
	return keys[row][col];
}
