#include "stm32f10x.h"                  // Device header
#include "bsp.h"  

//一键初始化
void bsp_init()
{
	limit_Init();
	LED_Init();
	TIM1_Init();
	usart3_Init();
	motor_init();
	OLED_Init();
	OLED_Clear();
	RS485_Init();
	// 初始化NVIC优先级分组为2
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

// 毫秒级延时
void delay_ms(uint32_t ms)
{
	//1ms 所需的循环次数 = 72,000,000 / 1000 / 4 = 18,000
    const uint32_t loops_per_ms = 18000;
	//一个循环大约需要4个周期
    for (uint32_t i = 0; i < ms; i++) {
        volatile uint32_t j;
        for (j = 0; j < loops_per_ms; j++);
    }
}
