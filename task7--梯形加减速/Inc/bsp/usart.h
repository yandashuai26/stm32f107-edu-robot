#ifndef __USART_H
#define __USART_H

#include <stdio.h>
#include <stm32f10x.h>
#include "FreeRTOS.h"
#include "semphr.h"

void Usart3Init(void);
void Usart3SemaphoreCreate(void);
void SerialSendString(char *String);

// USART3 receive semaphore, give in ISR, take in vTaskControl
extern SemaphoreHandle_t usart3_rx_semaphore;

#endif
