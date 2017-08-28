#include "stm32l1xx.h"
#include "stm32l1xx_nucleo.h"
#include "stm32l1xx_adc.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_dma.h"
#include "stm32l1xx_tim.h"
			
#define Ain1 GPIO_Pin_1
#define Ain1Channel ADC_Channel_1
#define ADCPort GPIOA
#define ADCBufferSize 1024

#define TriggerMeasurePin GPIO_Pin_10
#define TriggerMeasurePort GPIOA

GPIO_InitTypeDef GPIOPortA;  			// Struct for ADC1 GPIOA Config
ADC_InitTypeDef ADCInit;				// Struct for ADC Config
ADC_CommonInitTypeDef ADCSetup;			// Struct for ADC prescaler clock value

DMA_InitTypeDef DMASetup;				// Struct for DMA config

TIM_TimeBaseInitTypeDef TimeBaseSetup; 	// Struct for TimeBase setup
TIM_OCInitTypeDef TimerSetup;			// Struct for Timer setup


static uint32_t ADCDataBuffer[ADCBufferSize];		// Array to store data from ADC


void initDMA(void)
{
  /*------------------------ DMA1 configuration ------------------------------*/

  /* DMA1 channel1 configuration */
  DMA_DeInit(DMA1_Channel1);								// Reset all registers to default values

  DMASetup.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;				// Read from which register
  DMASetup.DMA_MemoryBaseAddr = (uint32_t)&ADCDataBuffer;  // Write to which address
  DMASetup.DMA_DIR = DMA_DIR_PeripheralSRC;					// Read from peripheral write to memory
  DMASetup.DMA_BufferSize = ADCBufferSize;					// How many transfers from register to memory
  DMASetup.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	// Don`t increment the peripheral address
  /*Memory increment doesn't work - buffer[i] oveflow */
  DMASetup.DMA_MemoryInc = DMA_MemoryInc_Enable;			// Increment the memory address Buffer[i] -> Buffer[i+1]

  DMASetup.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	// 16 bits
  DMASetup.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;			// 32 bits
  DMASetup.DMA_Mode = DMA_Mode_Normal;						// When all conversions are done, start again
  DMASetup.DMA_Priority = DMA_Priority_High;				// High priority of the channel
  DMASetup.DMA_M2M = DMA_M2M_Disable;						// Disable memory to memory transfer

  DMA_Init(DMA1_Channel1, &DMASetup);

  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
}

void initADC1(void)
{
	GPIO_StructInit(&GPIOPortA);	// Fill the variable with default settings
	GPIOPortA.GPIO_Pin = Ain1;
	GPIOPortA.GPIO_Mode = GPIO_Mode_AN;
	GPIOPortA.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(ADCPort, &GPIOPortA);

	ADC_DeInit(ADC1);    // Reset all ADC settings to default

	ADCInit.ADC_Resolution = ADC_Resolution_10b; // Select resolution
	ADCInit.ADC_ScanConvMode = DISABLE;			// Disable scan mode -> Measure only one input
	ADCInit.ADC_ContinuousConvMode = ENABLE;   // Enable continious mode -> measure many times same channel
	ADCInit.ADC_DataAlign = ADC_DataAlign_Right; // Align the 10bit data to the right
	ADCInit.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising; // Don't wait for edge to convert
	ADCInit.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO; // Trigger from Timer2
	ADC_Init(ADC1, &ADCInit);					// Initialize the ADC Init struct for ADC1

	ADCSetup.ADC_Prescaler = ADC_Prescaler_Div1; //Divide the HSI by 1 -> 16 Mhz (1Mhz conversion freq)
	ADC_CommonInit(&ADCSetup);

	/* Enable the request after last transfer for DMA Circular mode */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	  /* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);						// Enable the ADC1


	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET);  // Wait untill ADC1 is ON -> ADC Flag ADC on
}

void initTriggerMeasure(void)
{
		GPIO_StructInit(&GPIOPortA);		 	  // Fill the variable with default settings
		GPIOPortA.GPIO_Pin = TriggerMeasurePin;   // Specify LED2, PA.5
		GPIOPortA.GPIO_Mode = GPIO_Mode_OUT;      //Config output mode
		GPIOPortA.GPIO_OType = GPIO_OType_PP;	  //Config Push-Pull mode
		GPIOPortA.GPIO_PuPd = GPIO_PuPd_DOWN;	  // Pull down resistor
		GPIOPortA.GPIO_Speed = GPIO_Speed_2MHz;   // Low speed
		GPIO_Init(TriggerMeasurePort, &GPIOPortA);			// Initialize Port A with the settings saved in the structure variable

}

void Init(void)
{
	DBGMCU_Config(DBGMCU_SLEEP | DBGMCU_STOP | DBGMCU_STANDBY, ENABLE);  // Allow debugging during low power mode

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);    // Enable clock for DMA1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // Enable clock for ADC 1
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); // Enable clock for GPIOA

// *********************** Configure DMA ***************************************
	initDMA();
// *********************** Configure USART2 ************************************
	initADC1();
// ********************** Configure Trigger for start of measure******************************
	initTriggerMeasure();
// ********************** Configure SysTick Timer ******************************
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		while(1);
	}
}

static __IO uint32_t TimingDelay;   // Decrementing value for delay

void Delay(uint32_t nTime)
{
	TimingDelay = nTime;
	while(TimingDelay != 0);
}

void ClearADCDataBuffer (void)
{
	int index = 0;
	for(index = 0; index<= ADCBufferSize; index++)
	{
		ADCDataBuffer[index] = 0;
	}
}

void ResetADC1(void)
{
	ADC_Cmd(ADC1, DISABLE);						// Disable the ADC1
	ADC1->CR2 &= (uint32_t)(~ADC_CR2_SWSTART);
	ADC_ContinuousModeCmd(ADC1, DISABLE);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == SET);  // Wait untill ADC1 is shutdown

	ADC_Cmd(ADC1, ENABLE);						// Enable the ADC1
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET);  // Wait untill ADC1 is ON -> ADC Flag ADC on
	ADC_ContinuousModeCmd(ADC1, ENABLE);
}

void ResetDMA(void)
{
	DMA_Cmd(DMA1_Channel1, DISABLE);
	Delay(1);
	DMA_SetCurrDataCounter(DMA1_Channel1, ADCBufferSize);
	DMA_Cmd(DMA1_Channel1, ENABLE);
}
void TriggerMeasure()
{
	GPIO_SetBits(TriggerMeasurePort, TriggerMeasurePin);
	ADC_SoftwareStartConv(ADC1);
	Delay(1);
	GPIO_ResetBits(TriggerMeasurePort, TriggerMeasurePin);


}
int main(void)
{
	Init();
	ADC_RegularChannelConfig(ADC1, Ain1Channel , 1, ADC_SampleTime_24Cycles); //Configure the channel (PA.0, to be read)
	while(1)
	{
		TriggerMeasure();
		while(DMA_GetFlagStatus(DMA1_FLAG_TC1) != SET);
		ResetADC1();
		ResetDMA();
		ClearADCDataBuffer();
	//  ADC_Cmd(ADC1, ENABLE);
		Delay(1000);
	}


}

void SysTick_Handler(void)
{
	if(TimingDelay != 0x00)
	{
		TimingDelay--;
	}
}
