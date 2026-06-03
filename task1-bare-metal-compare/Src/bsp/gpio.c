#include "stm32f10x.h"                  // Device header
#include "gpio.h"
#include "motor.h"
#include "timer.h"

static volatile uint8_t limit_reached_flag = 0;// 标志位 0表示未达到限位开关 1表示已达到限位开关 
/************************************************************

限位开关  PC13    PC14

****************************************************************/
void limit_Init()
{
    // 开启GPIOC AFIOʱ时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    
    // PC13 PC14上拉输入
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);         
    
    // 开启AFIO
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13); 
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource14);  
    
    // 配置EXTI13下降沿输入
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line13;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_Init(&EXTI_InitStructure);
    
    //配置EXTI14
    EXTI_InitStructure.EXTI_Line = EXTI_Line14;
    EXTI_Init(&EXTI_InitStructure);
    
    //配置NVIC
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
	//初始化NVIC优先级分组为2(main.c)  抢占优先级0 响应优先级1
}

// 检测是否碰到限位开关（调用后自动清除标志）（外部调用,不要动我的静态变量）
uint8_t motor_is_limit_reached(void)
{
    uint8_t temp = limit_reached_flag;
    if (temp)
        limit_reached_flag = 0;  // 自动清除
    return temp;
}

// 暂停限位开关中断（关闭EXTI13和EXTI14）
void disable_limit_interrupt(void)
{
    EXTI->IMR &= ~(EXTI_Line13 | EXTI_Line14);  // 清除中断屏蔽位
}

// 恢复限位开关中断
void enable_limit_interrupt(void)
{
    EXTI->IMR |= (EXTI_Line13 | EXTI_Line14);   // 设置中断屏蔽位
}

void EXTI15_10_IRQHandler()
{
    if(EXTI_GetITStatus(EXTI_Line13)!= RESET)
	{
		if(0==GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13))
		{
			//消抖
			delay_ms(50);
			// 设置限位标志位
			limit_reached_flag = 1;
			//脉冲计数器清零
			motor_reset_pulse_count();
		}
		EXTI_ClearITPendingBit(EXTI_Line13);
    }
    
    if(EXTI_GetITStatus(EXTI_Line14)!= RESET)
	{
		if(0==GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14))
		{
			//消抖
			delay_ms(50);
			// 设置限位标志位
			limit_reached_flag = 1;
			//脉冲计数器清零
			motor_reset_pulse_count();
		}
		EXTI_ClearITPendingBit(EXTI_Line14);
    }
}
/************************************************************

LED    PE3 --> LED3 PE4 --> LED2 

****************************************************************/
void LED_Init()
{
	
    //开启GPIOC时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE); 
	
    //   PE3和PE4 推挽输出
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3  |  GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE,&GPIO_InitStructure);    

	//默认开启LED
    GPIO_ResetBits(GPIOE,GPIO_Pin_3 | GPIO_Pin_4);
}
