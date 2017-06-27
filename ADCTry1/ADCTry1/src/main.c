/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/
#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_nucleo.h"
#include "stm32l1xx_adc.h"

#define A0 GPIO_Pin_0
uint32_t SysClockSource;

static __IO uint32_t TimingDelay;		// Variable to store desired delay
GPIO_InitTypeDef GPIOPortA;  			//Structure variable to save configuration settings for Port A
ADC_InitTypeDef ADCInit;				// Struct for ADC Config
ADC_CommonInitTypeDef ADCSetup;			// Struct for ADC prescaler clock value
RCC_ClocksTypeDef RCC_Clocks;			// Struct for getting clock values
uint8_t SysClock;

uint16_t Ain;							// Data from ADC convertion
			
void Init(void)
{
	DBGMCU_Config(DBGMCU_SLEEP | DBGMCU_STOP | DBGMCU_STANDBY, ENABLE);  // Allow debugging during low power mode


	SysClock = RCC_GetSYSCLKSource();

	RCC_GetClocksFreq(&RCC_Clocks);						/* Get the clock values
																		Default values:
																SYSCLK_Frequency	uint32_t	32000000
																HCLK_Frequency	uint32_t	32000000
																PCLK1_Frequency	uint32_t	32000000
																PCLK2_Frequency	uint32_t	32000000	*/

//************************ Enable peripherals *********************************

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); // Enable clock for port A
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // Enable clock for ADC 1

//************************ Configure GPIO A.0 ******************************************

	GPIO_StructInit(&GPIOPortA);	// Fill the variable with default settings
	GPIOPortA.GPIO_Pin = A0;
	GPIOPortA.GPIO_Mode = GPIO_Mode_AN;
	GPIOPortA.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIOPortA);

//************************ Configure ADC1  ******************************************
//	RCC_ADCCLKConfig(RCC_PCLK2_Div6);  // Config the ADC clock


	ADC_DeInit(ADC1);    // Reset all ADC settings to default

	ADCInit.ADC_Resolution = ADC_Resolution_10b; // Select resolution
	ADCInit.ADC_ScanConvMode = DISABLE;			// Disable scan mode -> Measure only one input
	ADCInit.ADC_ContinuousConvMode = DISABLE;   // Disable continious mode -> measure only once
	ADCInit.ADC_DataAlign = ADC_DataAlign_Right; // Align the 10bit data to the right
	ADCInit.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; // Don't wait for external trigger to convert
	ADC_Init(ADC1, &ADCInit);					// Initialize the ADC Init struct for ADC1

	ADCSetup.ADC_Prescaler = ADC_Prescaler_Div2; 	//Divide the HCLK by 4 to be th ADC clock speed -> Fadc = 32/4 = 8 Mhz
	ADC_CommonInit(&ADCSetup);


	ADC_Cmd(ADC1, ENABLE);						// Enable the ADC1

	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET);  // Wait untill ADC1 is ON -> ADC Flag ADC on

	if (SysTick_Config(SystemCoreClock / 1000))
	{
			while(1);
	}
}

void Delay(uint32_t nTime)
{
	TimingDelay = nTime;			// Set the TimingDelay to nTime (in ms)
	while(TimingDelay != 0);		// Wait until TimingDelay gets reduced to 0 by SysTick_Handler()
}

uint16_t readADC1(uint8_t channel)
{
	ADC_RegularChannelConfig(ADC1, channel , 1, ADC_SampleTime_24Cycles);  //Configure the channel (PA.0, to be read)
	ADC_SoftwareStartConv(ADC1);										// Start convertion
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == SET);						// Wait until convertion is done
	return ADC_GetConversionValue(ADC1);
}

int main(void)
{
	Init();
	while(1)
	{
		Ain = readADC1(ADC_Channel_0);
	}
}

void SysTick_Handler(void)    // Handle the SysTick interrupt that goes every 1ms
{
	if(TimingDelay != 0x00)
	{
		TimingDelay--;			// Reduce the TimingDelay variable with 1 every 1ms
	}
}
