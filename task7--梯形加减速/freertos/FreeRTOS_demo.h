#ifndef __FREERTOS_DEMO_H
#define __FREERTOS_DEMO_H

#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>

// Message queue handles
extern QueueHandle_t temp_queue;
extern QueueHandle_t dis_queue;
extern QueueHandle_t abnormal_situation_dis_queue;
extern QueueHandle_t led_mode_queue;

void vFreeRtosStart(void);

#endif
