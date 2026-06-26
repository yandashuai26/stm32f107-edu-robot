#ifndef __USART_H
#define __USART_H
#include <stm32f10x.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "semphr.h"

void usart3_Init(void);
void usart3_semaphore_create(void);
void Serial_SendString(char *String);

// USART3接收完成信号量（ISR中give，vTaskControl中take）
extern SemaphoreHandle_t USART3_RxSemaphore;
#endif
