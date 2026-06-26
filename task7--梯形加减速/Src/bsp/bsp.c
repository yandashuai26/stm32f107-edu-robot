#include "stm32f10x.h"
#include "bsp.h"

// 一体化初始化
void BspInit(void)
{
	LimitInit();
	LedInit();
	Tim1Init();
	Usart3Init();
	MotorInit();
	OledInit();
	OledClear();
	Rs485Init();
	// 配置NVIC优先级分组为2
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

// 阻塞级延时
void DelayMs(uint32_t ms)
{
	// 1ms 大概循环次数 = 72,000,000 / 1000 / 4 = 18,000
	const uint32_t loops_per_ms = 18000;

	for (uint32_t i = 0; i < ms; i++)
	{
		volatile uint32_t j;

		for (j = 0; j < loops_per_ms; j++)
		{
			;
		}
	}
}
