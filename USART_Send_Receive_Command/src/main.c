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
	SendBufferComplete} USARTSTATE;
USARTSTATE USARTState;

char SendStartChar = 'S';
char SendEndChar = 'E';

char ReceivedCommand;
char CommandParam1;

uint32_t GlobalInterruptsDisabled;		// 0 if they are enabled, 1 if they are disabled

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
	initNVIC();
}
static __IO uint32_t TimingDelay;

void Delay_ms(uint32_t nTime)
{
	TimingDelay = nTime;
	while(TimingDelay != 0);
}

void SendChar(char Data)
{
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); // Wait until Tx Buffer is empty
	USART_SendData(USART2, Data);								// Load data into Data register
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET); // Wait until Transfer Complete flag is SET
}

void SendBufferUSART(uint16_t Buffer)
{
	uint16_t CurrentDigit = 0;

	if((State == SendingStarted) && (USARTState == SendBufferStart))
	{
		SendChar(SendStartChar);
		USARTState = SendByte1;
	}

	while (CurrentDigit <= ADCBufferSize)
	{
		switch(USARTState)
		{
		case SendByte1:
			SendChar(ADCDataBuffer[CurrentDigit] & 0x00FF);				// Send lower byte
			USARTState = SendByte2;
			/* no break */
		case SendByte2:
			SendChar(ADCDataBuffer[CurrentDigit] & 0xFF00);				// Send upper byte
			CurrentDigit++;
			USARTState = SendByte1;
			break;
		}
		USARTState = SendBufferComplete;
	}

	if((State == SendingStarted) && (USARTState == SendBufferComplete))
	{
		SendChar(SendEndChar);
		USARTState = Idle;
		State = SendingEnded;
	}
}
void CheckAndDisableInterrupts(void) { GlobalInterruptsDisabled = __get_PRIMASK();__disable_irq(); }
void CheckAndEnableInterrupts(void) { if(!GlobalInterruptsDisabled) __enable_irq(); }
void StuffBuffer (void)
{
	uint16_t index = 0;
	for(index = 0; index<= ADCBufferSize; index++)
	{
		ADCDataBuffer[index] = index;
	}
}
void ClearADCDataBuffer (void)
{
	int index = 0;
	for(index = 0; index<= ADCBufferSize; index++)
	{
		ADCDataBuffer[index] = 0;
	}
}
int main(void)
{
	CheckAndDisableInterrupts();
	USARTState = Idle;
	Init();
	State = ReadyToMeasure;
	CheckAndEnableInterrupts();

	while(1)
	{
		switch(State)
		{
		case ReadyToMeasure:
			if(USARTState == ParamReceived)
			{
				if(ReceivedCommand == 'S')
				{
					State = MeasureStarted;
					StuffBuffer();
					State = MeasureEnded;
				}
			}
			break;
		case MeasureEnded:
			State = SendingStarted;
			USARTState = SendBufferStart;
			SendBufferUSART(&ADCDataBuffer);
			State = SendingEnded;
			break;
		case SendingEnded:
			State = ReadyToMeasure;
			break;
		}
	}
}

void USART2_IRQn_Handler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE))
	{
		switch(USARTState)
		{
		case Idle:
			ReceivedCommand = USART_ReceiveData(USART2);
			if(ReceivedCommand != 'S')  { SendChar('E'); }
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

}

