#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stdio.h"
#include "usart.h"

// USART3接收信号量, ISR的give, vTaskControl的take
SemaphoreHandle_t usart3_rx_semaphore;

volatile uint8_t  usart3_rx_buffer[256];
volatile uint16_t usart3_rx_index = 0;
volatile uint8_t  usart3_rx_finished = 0;
volatile uint16_t usart3_rx_count = 0;

/* ================================================================ */
void Usart3Init(void)
{
	// 使能GPIOC, USART, AFIO
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	// 引脚重映射
	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);

	// PC10推挽输出 PC11上拉输入
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// USART3 9600 - 8 - 0 - 0
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART3, &USART_InitStructure);

	// 使能中断
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	// NVIC配置
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStructure);

	// 使能USART3
	USART_Cmd(USART3, ENABLE);
}

// 创建USART3接收信号量, 在vFreeRtosStart中调用
void Usart3SemaphoreCreate(void)
{
	usart3_rx_semaphore = xSemaphoreCreateBinary();
	configASSERT(usart3_rx_semaphore != NULL);
}

// 使用USART3 发送一个字节
void SerialSendByte(uint8_t Byte)
{
	USART_SendData(USART3, Byte);

	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
	{
		;
	}
}

// 使用USART3 发送一个字符串
void SerialSendString(char *String)
{
	uint8_t i;

	for (i = 0; String[i] != '\0'; i++)
	{
		SerialSendByte(String[i]);
	}
}

// 不使用微库
#pragma import(__use_no_semihosting)
struct __FILE { int handle; };
FILE __stdout;

void _sys_exit(int x)
{
	x = x;
}

int fputc(int ch, FILE *f)
{
	USART_SendData(USART3, (uint8_t)ch);

	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
	{
		;
	}
	return ch;
}

void USART3_UartWrite(uint8_t *buf, uint8_t len)
{
	uint8_t i = 0;

	for (i = 0; i < len; i++)
	{
		USART_SendData(USART3, buf[i]);

		while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
		{
			;
		}
	}
}

// USART3中断接收处理函数
void USART3_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		uint8_t data = USART_ReceiveData(USART3);

		if (usart3_rx_index < 256)
		{
			usart3_rx_buffer[usart3_rx_index++] = data;
		}
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}

	if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
	{
		volatile uint16_t temp = USART3->SR;
		temp = USART3->DR;
		(void)temp;

		usart3_rx_count = usart3_rx_index;
		usart3_rx_finished = 1;
		usart3_rx_index = 0;
		USART_ClearITPendingBit(USART3, USART_IT_IDLE);

		xSemaphoreGiveFromISR(usart3_rx_semaphore, &xHigherPriorityTaskWoken);
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
