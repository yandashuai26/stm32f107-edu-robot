// main.c
#include "stm32f10x.h"
#include "bsp.h"
#include "FreeRTOS_demo.h"

int main(void)
{
	// 全局初始化
	BspInit();
	DelayMs(100);
	
	MotorSetAccel(100);     
	MotorSetDecel(100);      
	MotorSetSpeed(100);
	MotorSetTargetUnit(500);
	MotorStart();

	//vFreeRtosStart();

	while (1)
	{

	}
}

// 1ms定时器
volatile uint32_t systick_ms = 0;

void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		systick_ms++;
	}
}
