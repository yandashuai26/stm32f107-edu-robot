#ifndef __GPIO_H
#define __GPIO_H
#include "FreeRTOS.h"
#include "semphr.h"

void LED_Init(void);
void limit_Init(void);
void limit_semaphore_create(void);
uint8_t motor_is_limit_reached(void);
void set_limit_reached_flag(void);
uint8_t get_limit_trigger_pin(void);
void limit_enter_cooldown(void);
void limit_exit_cooldown(void);
void disable_limit_interrupt(void);
void enable_limit_interrupt(void);

// 限位开关信号量句柄（ISR中give，任务中take）
extern SemaphoreHandle_t LimitSemaphore;

#endif
