#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_nucleo.h"
#include "stm32l1xx_usart.h"



GPIO_InitTypeDef GPIOSetup;
USART_InitTypeDef USARTSetup;
NVIC_InitTypeDef NVICSetup;				// Struct for NVIC config

#define ADCBufferSize 10700
static uint32_t ADCDataBuffer[ADCBufferSize];

typedef enum STATE {ReadyToMeasure, MeasureStarted, MeasureEnded, SendingStarted, SendingEnded} STATE;
STATE State;

typedef enum USARTSTATE{Idle, ComReceived, ParamReceived, SendBufferStart, SendByte1, SendByte2,
	SendByteComplete, SendBufferComplete} USARTSTATE;
USARTSTATE USARTState;

#define SendStartChar "S"
#define SendEndChar "E"

uint16_t ReceivedCommand;
uint16_t CommandParam1;

void initUSART(void)
{
// ************** Configure USART2 Tx (PA.02) and USART Rx (PA.3) as alternate function push-pull ****
	GPIOSetup.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIOSetup.GPIO_Speed = GPIO_Speed_2MHz;
	GPIOSetup.GPIO_Mode = GPIO_Mode_AF;
	GPIOSetup.GPIO_OType = GPIO_OType_OD;
	GPIOSetup.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIOSetup);

// ***************************** Map USART2 to PA.2 and PA.3 ******************************
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

// ***************************** Initialize USART2 *****************************************
	USARTSetup.USART_BaudRate = 10000;
	USARTSetup.USART_WordLength = USART_WordLength_8b;
	USARTSetup.USART_StopBits = USART_StopBits_1;
	USARTSetup.USART_Parity = USART_Parity_No;
	USARTSetup.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USARTSetup.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART2, &USARTSetup); // Configure USART
	USART_Cmd(USART2, ENABLE); // Enable the USART

// **************************** Configure interrupts ***************************************
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
//	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}
void initNVIC(void)
{
// ****************** Setup the Timer count complete interrupt *****************************
	NVICSetup.NVIC_IRQChannel = USART2_IRQn; 		// Enable USART2 Interrupts
	NVICSetup.NVIC_IRQChannelPreemptionPriority = 2;
	NVICSetup.NVIC_IRQChannelSubPriority = 1;
	NVICSetup.NVIC_IRQChannelCmd = ENABLE;			// Enable global interrupts
	NVIC_Init(&NVICSetup);
}

void Init(void)
{
// ********************** Configure SysTick Timer ******************************
	if (SysTick_Config(SystemCoreClock / 1000))	{ while(1);	}

// ********************** Enable periph clocks *********************************
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	initUSART();
}
static __IO uint32_t TimingDelay;

void Delay_ms(uint32_t nTime)
{
	TimingDelay = nTime;
	while(TimingDelay != 0);
}

void SendChar(char Data)
{

}

void SendBufferUSART(uint16_t Buffer)
{
	uint16_t CurrentDigit = 0;
	if((State == SendingStarted) && (USARTState == SendBufferStart))
	{
		SendChar(SendStartChar);
		while (USART_GetITStatus(USART2, USART_IT_TXE) == RESET);	// Wait until TXE is SET -> Transmit register not empty
//		USART_ClearITPendingBit(USART2, USART_IT_TXE);
		USARTState = SendByte1;
	}
	while (CurrentDigit <= ADCBufferSize)
	{
		if((State == SendingStarted) && (USARTState == SendByte1))
		{
			SendChar(ADCDataBuffer[CurrentDigit] & 0x00FF);				// Send lower byte
			while (USART_GetITStatus(USART2, USART_IT_TXE) == RESET);	// Wait until TXE is SET -> Transmit register not empty
			USARTState = SendByte2;
		}
		if((State == SendingStarted) && (USARTState == SendByte2))
		{
			SendChar(ADCDataBuffer[CurrentDigit] & 0xFF00);				// Send upper byte
			while (USART_GetITStatus(USART2, USART_IT_TXE) == RESET);	// Wait until TXE is SET -> Transmit register not empty
			CurrentDigit++;
			USARTState = SendByte1;
		}

	}
}
int main(void)
{
	USARTState = Idle;
	while(1)
	{

	}
}

USART2_IRQn_Handler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE))
	{
		switch(USARTState)
		{
		case Idle:
			ReceivedCommand = USART_ReceiveData(USART2);
			USARTState = ComReceived;
			USART_ClearITPendingBit(USART2, USART_IT_RXNE);
			break;
		case ComReceived:
			CommandParam1 = USART_ReceiveData(USART2);
			USARTState = ParamReceived;
			USART_ClearITPendingBit(USART2, USART_IT_RXNE);
			break;
		}
	}

	if(USART_GetITStatus(USART2, USART_IT_TXE))
	{



	}
}

