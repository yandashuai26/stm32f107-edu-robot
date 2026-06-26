#include "stm32f10x.h"                  // Device header
#include "gpio.h"
#include "motor.h"
#include "timer.h"
#include "FreeRTOS.h"
#include "semphr.h"

static volatile uint8_t limit_reached_flag = 0;// 标志位 0表示未达到限位开关 1表示已达到限位开关
static volatile uint8_t limit_trigger_pin = 0; // 记录触发引脚：0=无触发 13=PC13触发 14=PC14触发
static volatile uint8_t limit_cooldown = 0;     // 冷却标志 0=正常 1=冷却期(忽略释放抖动)

// 限位开关二进制信号量（ISR中give，任务中take）
SemaphoreHandle_t LimitSemaphore;

/************************************************************

限位开关  PC13    PC14
ISR中记录触发引脚号，中断只传递信号量 , 和标志位

****************************************************************/
// 创建限位开关信号量
void limit_semaphore_create(void)
{
    LimitSemaphore = xSemaphoreCreateBinary();
    configASSERT(LimitSemaphore != NULL);
}

void limit_Init()
{
    // 开启GPIOC AFIO时钟
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
    //初始化NVIC优先级分组为2(main.c)  抢占优先级0 响应优先级0
}

// 检测是否碰到限位开关（调用后自动清除标志）（外部调用,不要动我的静态变量）
uint8_t motor_is_limit_reached(void)
{
    uint8_t temp = limit_reached_flag;
    if (temp)
        limit_reached_flag = 0;  // 自动清除
    return temp;
}

// 设置限位到达标志（由vTaskLimitSwitch任务调用）
void set_limit_reached_flag(void)
{
    limit_reached_flag = 1;
}

// 获取并清除触发引脚号（由vTaskLimitSwitch任务调用）
// 返回 0=无触发, 13=PC13触发, 14=PC14触发
uint8_t get_limit_trigger_pin(void)
{
    uint8_t pin = limit_trigger_pin;
    limit_trigger_pin = 0;
    return pin;
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

// 限位开关中断服务函数
// 只记录触发引脚+释放信号量，消抖和电机控制全部由任务完成
void EXTI15_10_IRQHandler()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(EXTI_GetITStatus(EXTI_Line13) != RESET)
    {
        // 清理中断标志位
        EXTI_ClearITPendingBit(EXTI_Line13);
        // 冷却期内忽略释放抖动 , 在冷却期不会释放信号量
        if (!limit_cooldown)
        {
            // 记录触发引脚
            limit_trigger_pin = 13;
            // 释放信号量唤醒限位开关任务
            xSemaphoreGiveFromISR(LimitSemaphore, &xHigherPriorityTaskWoken);
        }
    }

    if(EXTI_GetITStatus(EXTI_Line14) != RESET)
    {
        // 清理中断标志位
        EXTI_ClearITPendingBit(EXTI_Line14);
        // 冷却期内忽略释放抖动 , 在冷却期不会释放信号量
        if (!limit_cooldown)
        {
            // 记录触发引脚
            limit_trigger_pin = 14;
            // 释放信号量唤醒限位开关任务
            xSemaphoreGiveFromISR(LimitSemaphore, &xHigherPriorityTaskWoken);
        }
    }

    // 如果唤醒了更高优先级的任务，进行任务切换
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
// 进入冷却期（换向后调用，屏蔽开关释放抖动）
void limit_enter_cooldown(void)
{
    limit_cooldown = 1;
}

// 退出冷却期（冷却时间到后调用）
void limit_exit_cooldown(void)
{
    limit_cooldown = 0;
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
