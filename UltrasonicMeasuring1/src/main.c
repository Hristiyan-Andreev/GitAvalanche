#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_nucleo.h"
#include "stm32l1xx_usart.h"
#include "stm32l1xx_adc.h"

#define A0 GPIO_Pin_0

GPIO_InitTypeDef GPIOPortA;  			//Structure variable to save configuration settings for Port A
ADC_InitTypeDef ADCInit;				// Struct for ADC Config
ADC_CommonInitTypeDef ADCSetup;			// Struct for ADC prescaler clock value
uint16_t Ain;							// Data from ADC convertion

USART_InitTypeDef USART_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;  	// Struct for USART Port A
			
void Init(void)
{
	DBGMCU_Config(DBGMCU_SLEEP | DBGMCU_STOP | DBGMCU_STANDBY, ENABLE);  // Allow debugging during low power mode

	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // Enable clock for ADC 1

// ************************* Cofnfigure ADC *************************************
	GPIO_StructInit(&GPIOPortA);	// Fill the variable with default settings
	GPIOPortA.GPIO_Pin = A0;
	GPIOPortA.GPIO_Mode = GPIO_Mode_AN;
	GPIOPortA.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIOPortA);

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

// *********************** Configure USART2 *************************************

	/* Configure USART2 Tx (PA.02) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Map USART2 to A.02
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	USART_InitStructure.USART_BaudRate = 10000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	/* Configure USART */
	USART_Init(USART2, &USART_InitStructure);
	/* Enable the USART */
	USART_Cmd(USART2, ENABLE);
// ********************** Configure SysTick Timer *********************
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		while(1);
	}
}

static __IO uint32_t TimingDelay;

void Delay(uint32_t nTime)
{
	TimingDelay = nTime;
	while(TimingDelay != 0);
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

	for(;;);
}
