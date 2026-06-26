#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "gpio.h"
#include "motor.h"
#include "timer.h"

static volatile uint8_t limit_reached_flag = 0;		// 标志位 0=未达到限位 1=已达到限位
static volatile uint8_t limit_trigger_pin = 0;		// 记录触发引脚号 0=无触发 13=PC13触发 14=PC14触发
static volatile uint8_t limit_cooldown = 0;			// 冷却标志 0=正常 1=冷却中(禁止释放信号量)

// 限位开关信号量, ISR的give, 任务的take
SemaphoreHandle_t limit_semaphore;

/* ---------- 创建限位开关信号量 ---------- */
void LimitSemaphoreCreate(void)
{
	limit_semaphore = xSemaphoreCreateBinary();
	configASSERT(limit_semaphore != NULL);
}

void LimitInit(void)
{
	// 使能GPIOC AFIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	// PC13 PC14上拉输入
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// 配置AFIO
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource14);

	// 配置EXTI13下降沿触发
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line13;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitStructure);

	// 配置EXTI14
	EXTI_InitStructure.EXTI_Line = EXTI_Line14;
	EXTI_Init(&EXTI_InitStructure);

	// 配置NVIC
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
}

// 查询是否触发限位开关, 读取后自动清除标志, 供外部调用
uint8_t MotorIsLimitReached(void)
{
	uint8_t temp = limit_reached_flag;

	if (temp)
	{
		limit_reached_flag = 0;
	}
	return temp;
}

// 设置限位到达标志, 供vTaskLimitSwitch内部使用
void SetLimitReachedFlag(void)
{
	limit_reached_flag = 1;
}

// 获取限位触发引脚号, 供vTaskLimitSwitch内部使用
uint8_t GetLimitTriggerPin(void)
{
	uint8_t pin = limit_trigger_pin;
	limit_trigger_pin = 0;
	return pin;
}

// 暂停限位开关中断, 关闭EXTI13和EXTI14
void DisableLimitInterrupt(void)
{
	EXTI->IMR &= ~(EXTI_Line13 | EXTI_Line14);
}

// 恢复限位开关中断
void EnableLimitInterrupt(void)
{
	EXTI->IMR |= (EXTI_Line13 | EXTI_Line14);
}

// 限位开关中断服务函数
void EXTI15_10_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (EXTI_GetITStatus(EXTI_Line13) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line13);

		if (!limit_cooldown)
		{
			limit_trigger_pin = 13;
			xSemaphoreGiveFromISR(limit_semaphore, &xHigherPriorityTaskWoken);
		}
	}

	if (EXTI_GetITStatus(EXTI_Line14) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line14);

		if (!limit_cooldown)
		{
			limit_trigger_pin = 14;
			xSemaphoreGiveFromISR(limit_semaphore, &xHigherPriorityTaskWoken);
		}
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// 进入冷却期, 外部调用, 禁止释放信号量
void LimitEnterCooldown(void)
{
	limit_cooldown = 1;
}

// 退出冷却期, 冷却时间到后调用
void LimitExitCooldown(void)
{
	limit_cooldown = 0;
}

/* ================================================================ */
void LedInit(void)
{
	// 使能GPIOE时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

	// PE3和PE4 推挽输出
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// 默认关闭LED
	GPIO_ResetBits(GPIOE, GPIO_Pin_3 | GPIO_Pin_4);
}
