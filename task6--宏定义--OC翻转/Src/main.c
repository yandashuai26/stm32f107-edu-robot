// main.c
#include "stm32f10x.h"                  // Device header
#include "bsp.h"
#include "FreeRTOS_demo.h"

int main(void)
{
	//全部初始化
	bsp_init();
	delay_ms(100);
	
	//设定速度,不要太快
	motor_set_speed(MOTOR_DEFAULT_SPEED_HZ);
	motor_start();
	
	FreeRTOS_Start();
	while(1)
	{

	}
}

// 1ms定时器
volatile uint32_t sysTick_ms = 0;
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        sysTick_ms++;  // 1ms递增1
		// 5s归零
		if(sysTick_ms>5000)
		{
			sysTick_ms = 0;
		}
    }
}
