#include "stm32f10x.h"                  // Device header
#include "timer.h"
#include "stm32f10x_it.h"


/************************************************************

	Timer 相关函数  

**************************************************************/
// timer初始化
void TIM1_Init()
{
	  //开启TIM1的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);      
	
	// 选择TIM1为内部时钟
	TIM_InternalClockConfig(TIM1);
	
    //时基单元初始化 
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;                
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;        //不分频
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;    //向上计数
    TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;                //ARR
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;                //PSC
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;            //重复计数器，高级定时器才会用到
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);               
    // 7200000 / (ARR+1) / (PSC+1)  == 1000  每秒触发重复计数器1000次,每1ms触发一次           
	
	//使能TIM1的更新中断
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);                    //开启TIM1的更新中断                                                               
                                                              
    //NVIC配置
    NVIC_InitTypeDef NVIC_InitStructure;                        
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;              
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;               
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;    
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;            
    NVIC_Init(&NVIC_InitStructure);   // NVIC为分组2(main.c),抢占优先级为3,响应优先级为2                                                                                      

	//TIM1使能
    TIM_Cmd(TIM1, ENABLE);           
}




/************************************************************

	delay 相关函数

**************************************************************/

//// 毫秒级延时
//void delay_ms(uint32_t ms)
//{
//    uint32_t start = sysTick_ms;
//    while ((sysTick_ms - start) < ms);
//}




