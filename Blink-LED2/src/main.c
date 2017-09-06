#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_nucleo.h"

#define LED2 GPIO_Pin_5
uint8_t SysClockSource;

GPIO_InitTypeDef GPIOPortAInit;  //Structure variable to save configuration settings for Port A
RCC_ClocksTypeDef RCC_Clocks;



void Init(void)
{
	RCC_GetClocksFreq(&RCC_Clocks);
	SysClockSource = RCC_GetSYSCLKSource();

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); // Enable clock for port A

	GPIO_StructInit(&GPIOPortAInit);	// Fill the variable with default settings
	GPIOPortAInit.GPIO_Pin = LED2;   // Specify LED2, PA.5
	GPIOPortAInit.GPIO_Mode = GPIO_Mode_OUT;    //Config output mode
	GPIOPortAInit.GPIO_OType = GPIO_OType_PP;	//Config Push-Pull mode
	GPIOPortAInit.GPIO_Speed = GPIO_Speed_10MHz;  //Maximum speed
	GPIO_Init(GPIOA, &GPIOPortAInit);			// Initialize Port A with the settings saved in the structure variable

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

int main(void)
{
	Init();

	while(1)
	{
		GPIO_ToggleBits(GPIOA, LED2);
		Delay(500);
	}

}

void SysTick_Handler(void)
{
	if(TimingDelay != 0x00)
	{
		TimingDelay--;
	}
}
