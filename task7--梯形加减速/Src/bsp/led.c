#include "stm32f10x.h"
#include "led.h"

/* ---------- 关闭LED ---------- */
void LedOff(void)
{
	GPIO_SetBits(GPIOE, GPIO_Pin_3 | GPIO_Pin_4);
}

// 打开LED
void LedOn(void)
{
	GPIO_ResetBits(GPIOE, GPIO_Pin_3 | GPIO_Pin_4);
}
