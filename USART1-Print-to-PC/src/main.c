#include "stm32l1xx.h"
#include "stm32l1xx_nucleo.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_usart.h"


static __IO uint32_t TimingDelay;		// Variable to store desired delay
GPIO_InitTypeDef PortAStruct;			// For setting up TX and RX of USART2 -> PA.2 and PA.3
USART_InitTypeDef usart2Struct;			// For setting up USART 2 communication

void init()
{
	DBGMCU_Config(DBGMCU_SLEEP | DBGMCU_STOP | DBGMCU_STANDBY, ENABLE);  // Allow debugging during low power mode

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); 				// Enable clock for GPIO Port A module
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);				// Enable clock for USART 2 module

// ********************** Setup LED*****************************
	GPIO_DeInit(GPIOA);
	PortAStruct.GPIO_Pin = LED2_PIN;
	PortAStruct.GPIO_Mode = GPIO_Mode_OUT;
	PortAStruct.GPIO_OType = GPIO_OType_PP;
	PortAStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(LED2_GPIO_PORT, &PortAStruct);

// ************************  Setup the Tx of USART2 ********************************
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_DeInit(GPIOA);
	GPIO_StructInit(&PortAStruct);
	PortAStruct.GPIO_Pin = GPIO_Pin_2;				// PA.2 Connected to ST-Link on Nucleo 64 board
	PortAStruct.GPIO_Mode = GPIO_Mode_AF;			// Mode -> alternative function
//	PortAStruct.GPIO_OType = GPIO_OType_PP;			// Output type -> Push pull
//	PortAStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;			// Pull Up
	PortAStruct.GPIO_Speed = GPIO_Speed_40MHz;		// Maximum speed
	GPIO_Init(GPIOA, &PortAStruct);					// Initialize PA.2 -> Tx with above settings

// ***********************  Setup the Rx of USART2 *********************************
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	GPIO_DeInit(GPIOA);
	GPIO_StructInit(&PortAStruct);
	PortAStruct.GPIO_Pin = GPIO_Pin_3;
	PortAStruct.GPIO_Mode = GPIO_Mode_AF;
	PortAStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &PortAStruct);

// ***********************  Setup the USART2 *********************************

	USART_StructInit(&usart2Struct);    			// Set up with default settings - Baud rate: 9600, word lenght: 8bits, parity: none, stop bits: 1
	USART_Init(USART2, &usart2Struct);				// Initilaize with the above (default settings)

	if (SysTick_Config(SystemCoreClock / 1000))
		{
			while(1);
		}

}

// ********************* Delay function ******************************************************
void Delay(uint32_t nTime)
{
	TimingDelay = nTime;			// Set the TimingDelay to nTime (in ms)
	while(TimingDelay != 0);		// Wait until TimingDelay gets reduced to 0 by SysTick_Handler()
}

void SysTick_Handler(void)    // Handle the SysTick interrupt that goes every 1ms
{
	if(TimingDelay != 0x00)
	{
		TimingDelay--;			// Reduce the TimingDelay variable with 1 every 1ms
	}
}
// ********************* Main function *******************************************************
int PutCharUSART(USART_TypeDef* USARTx, uint16_t Data)
{
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET); // Wait untill Tx Buffer is empty
	USART_SendData(USARTx, Data);								// Load data into Data register
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET); // Wait until Transfer Complete flag is SET

	return 0;
}
int main(void)
{
	uint16_t Data = 2;
	init();
	USART_Cmd(USART2, ENABLE);						// Start the USART2 communication
	while(1)
	{
		PutCharUSART(USART2, Data);
		Delay(250);
	}
}
