#ifndef __GPIO_H
#define __GPIO_H
#include "FreeRTOS.h"
#include "semphr.h"

void LedInit(void);
void LimitInit(void);
void LimitSemaphoreCreate(void);
uint8_t MotorIsLimitReached(void);
void SetLimitReachedFlag(void);
uint8_t GetLimitTriggerPin(void);
void LimitEnterCooldown(void);
void LimitExitCooldown(void);
void DisableLimitInterrupt(void);
void EnableLimitInterrupt(void);

// 限位开关信号量, ISR的give, 任务的take
extern SemaphoreHandle_t limit_semaphore;

#endif
