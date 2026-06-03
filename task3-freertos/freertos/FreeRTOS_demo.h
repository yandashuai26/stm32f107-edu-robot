#ifndef __FREERTOS_DEMO_H
#define __FREERTOS_DEMO_H

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

// ÉùÃ÷¶ÓÁÐ¾ä±ú
extern QueueHandle_t TempQueue;
extern QueueHandle_t DisQueue;

void FreeRTOS_Start(void);

#endif
