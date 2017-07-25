#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_nucleo.h"
#include "stm32l1xx_usart.h"
#include <stdio.h>

GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;

char ch;

void init()
{
	if (SysTick_Config(SystemCoreClock / 1000))
		{
			while(1);
		}
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* Configure USART1 Tx (PA.02) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// Map USART1 to A.02
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	// Initialize USART
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx;
	/* Configure USART */
	USART_Init(USART1, &USART_InitStructure);
	/* Enable the USART */
	USART_Cmd(USART1, ENABLE);

}

static __IO uint32_t TimingDelay;

void Delay(uint32_t nTime)
{
	TimingDelay = nTime;
	while(TimingDelay != 0);
}
			

int main(void)
{
	init();
	while(1)
	{
		ch = 1;

		USART_SendData(USART1, (uint16_t)ch);
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	}
}

void SysTick_Handler(void)
{
	if(TimingDelay != 0x00)
	{
		TimingDelay--;
	}
}
