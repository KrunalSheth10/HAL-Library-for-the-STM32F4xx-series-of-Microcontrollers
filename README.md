# STM32F4xxHardwareAbstractionLayer

 This project is a Hardware Abastraction Layer for the STM32F4xx Microcontroller.
 It makes use of the CMSIS Library for all the register definitions 

 It Contains Polling and Interrupt Drivers for the following peripherals:
 * GPIO
 * SPI
 * UART

It also contains Libraries to:
* Configure the system clock
* To debounce a button using Polling and Interrupts
* To interface with Keypad using Polling and Interrupts
* To Configure the Systick :
   * To implement a Timeout Feature
   * To implement delay function

I have created this project as a learning process to understand the various peripherals and communication protocols at the register level.
