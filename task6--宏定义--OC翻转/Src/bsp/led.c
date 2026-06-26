#include "stm32f10x.h"                  // Device header
#include "led.h"
/* ---------- 开启LED2和LED3 ---------- */
void LED_OFF()
{
	GPIO_SetBits(GPIOE,GPIO_Pin_3 | GPIO_Pin_4);
}

// 关闭LED2和LED3
void LED_ON()
{
	GPIO_ResetBits(GPIOE,GPIO_Pin_3 | GPIO_Pin_4);
}

// LED2和LED3慢闪
//void LED_SLOW()
//{
//	GPIO_ResetBits(GPIOC,GPIO_Pin_3 | GPIO_Pin_4);
//	//delay_ms(500);
//	GPIO_SetBits(GPIOC,GPIO_Pin_3 | GPIO_Pin_4);
//	//delay_ms(500);
//}
// LED2和LED3快闪
//void LED_FAST()
//{
//	GPIO_ResetBits(GPIOC,GPIO_Pin_3 | GPIO_Pin_4);
//	//delay_ms(100);
//	GPIO_SetBits(GPIOC,GPIO_Pin_3 | GPIO_Pin_4);
//	//delay_ms(100);
//}
