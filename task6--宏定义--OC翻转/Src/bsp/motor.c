#include "stm32f10x.h"                  // Device header
#include "motor.h"
#include "FreeRTOS_demo.h"
#include "bsp.h"

static motor_t g_motor;                  // 电机全局状态

// 电机目标到达信号量(ISR中give, vTaskMotorReached中take)
SemaphoreHandle_t MotorTargetReachedSemaphore;

// 当前CCR1间隔(ISR中计算缓存)
static uint16_t g_ccr_interval;

// CC1沿翻转: 0/1交替, 每2次CC1=1个完整脉冲
static uint8_t g_edge_toggle;

// 钳制频率大小, 低于 MOTOR_MIN_HZ 或高于 MOTOR_MAX_HZ 会被限制
static uint32_t clamp_hz(uint32_t hz)
{
    if (hz < MOTOR_MIN_HZ) return MOTOR_MIN_HZ;
    if (hz > MOTOR_MAX_HZ) return MOTOR_MAX_HZ;
    return hz;
}

// 更新CCR间隔值, 不直接写寄存器(ISR负责动态更新CCR)
static void update_interval(uint32_t hz)
{
    g_ccr_interval = MOTOR_CALC_CCR_INTERVAL(hz);
}

// 设置首次CCR1匹配点: 读取当前CNT, 计算第一次翻转时机
// 仅在motor_start()中调用, 用于启动脉冲输出
static void set_first_ccr(uint32_t hz)
{
    uint16_t interval = MOTOR_CALC_CCR_INTERVAL(hz);
    uint16_t cnt = TIM_GetCounter(MOTOR_TIM);
    g_ccr_interval = interval;
    TIM_SetCompare1(MOTOR_TIM, cnt + interval);
}


/* ---------- 电机初始化  PA0-->PUL   PE12-->DIR   PB11-->ENA(低电平有效)  ---------- */
void motor_init()
{
    // 初始化电机结构体, 设定默认值
    memset(&g_motor, 0, sizeof(g_motor));
    g_motor.target_pulses  = MOTOR_DEFAULT_TARGET_PULSES;
    g_motor.current_hz     = MOTOR_DEFAULT_SPEED_HZ;
    g_motor.target_hz      = MOTOR_DEFAULT_SPEED_HZ;
    g_motor.user_speed_hz  = MOTOR_DEFAULT_SPEED_HZ;
    g_ccr_interval         = MOTOR_CALC_CCR_INTERVAL(MOTOR_DEFAULT_SPEED_HZ);

    // 使能 GPIOA, GPIOB, GPIOE, TIM5 时钟
    RCC_APB2PeriphClockCmd(MOTOR_PUL_CLK, ENABLE);   // PA0  -> TIM5_CH1 复用输出
    RCC_APB2PeriphClockCmd(MOTOR_ENA_CLK, ENABLE);   // PB11 -> ENA 使能
    RCC_APB2PeriphClockCmd(MOTOR_DIR_CLK, ENABLE);   // PE12 -> DIR 方向
    RCC_APB1PeriphClockCmd(MOTOR_TIM_RCC,  ENABLE);

    // 配置控制引脚 PB11(ENA) PE12(DIR)
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = MOTOR_GPIO_MODE;
    GPIO_InitStructure.GPIO_Pin = MOTOR_ENA_PIN;
    GPIO_InitStructure.GPIO_Speed = MOTOR_GPIO_SPEED;
    GPIO_Init(MOTOR_ENA_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = MOTOR_GPIO_MODE;
    GPIO_InitStructure.GPIO_Pin = MOTOR_DIR_PIN;
    GPIO_InitStructure.GPIO_Speed = MOTOR_GPIO_SPEED;
    GPIO_Init(MOTOR_DIR_PORT, &GPIO_InitStructure);

    GPIO_ResetBits(MOTOR_ENA_PORT, MOTOR_ENA_PIN);    // ENA低电平 -> 使能电机驱动
    GPIO_SetBits(MOTOR_DIR_PORT,  MOTOR_DIR_PIN);     // DIR高电平 -> 默认正方向

    // TIM5 时基: 72MHz/PSC=8=9MHz, ARR=0xFFFF (固定最大值, CNT自由运行)
    TIM_InternalClockConfig(MOTOR_TIM);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = MOTOR_TIM_CKD;
    TIM_TimeBaseInitStructure.TIM_CounterMode = MOTOR_TIM_COUNTER_MODE;
    TIM_TimeBaseInitStructure.TIM_Period = MOTOR_TIM_ARR;          // ARR=0xFFFF
    TIM_TimeBaseInitStructure.TIM_Prescaler = MOTOR_TIM_PRESCALER - 1;  // PSC=8
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(MOTOR_TIM, &TIM_TimeBaseInitStructure);

    // 配置 TIM5_CH1 输出比较翻转模式
    // ARR=0xFFFF固定, CCR在ISR中动态更新以控制脉冲间隔
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = MOTOR_TIM_OC_MODE;
    TIM_OCInitStructure.TIM_OCPolarity = MOTOR_TIM_OC_POLARITY;
    TIM_OCInitStructure.TIM_OutputState = MOTOR_TIM_OC_STATE;
    TIM_OCInitStructure.TIM_Pulse = 0;                            // 初始CCR=0, motor_start()设置
    TIM_OC1Init(MOTOR_TIM, &TIM_OCInitStructure);

    // 配置 PA0 为复用推挽输出 -> TIM5_CH1(PUL)
    GPIO_InitStructure.GPIO_Mode = MOTOR_PUL_GPIO_MODE;
    GPIO_InitStructure.GPIO_Pin = MOTOR_PUL_PIN;
    GPIO_InitStructure.GPIO_Speed = MOTOR_GPIO_SPEED;
    GPIO_Init(MOTOR_PUL_PORT, &GPIO_InitStructure);

    // 配置 TIM5 CC1 中断优先级 (NVIC group 2, 在 FreeRTOS_demo.c 中配置)
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = MOTOR_TIM_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = MOTOR_TIM_PREEMPT_PRIO;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = MOTOR_TIM_SUB_PRIO;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // TIM5使能, CNT开始自由运行; CC1中断暂不使能(motor_start()中开启)
    TIM_Cmd(MOTOR_TIM, ENABLE);
    // TIM5 CNT在0~65535之间循环, OC输出因CCR=0会立即产生首个脉冲,
    // 但CC1中断未使能, ISR不会触发, 无实际影响
}

// 创建电机目标到达信号量(在FreeRTOS_Start中调用)
void motor_target_semaphore_create(void)
{
    MotorTargetReachedSemaphore = xSemaphoreCreateBinary();
    configASSERT(MotorTargetReachedSemaphore != NULL);
}


/* ---------- 方向控制 ---------- */

// 切换电机方向, 带加减速保护
// 流程: 先停机 -> 等待完全停止 -> 切换方向引脚 -> 重新启动
void motor_set_dir(uint8_t dir)
{
    uint8_t current_dir = GPIO_ReadOutputDataBit(MOTOR_DIR_PORT, MOTOR_DIR_PIN);
    if (current_dir == dir) return;

    motor_stop();

    while (g_motor.running) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    GPIO_WriteBit(MOTOR_DIR_PORT, MOTOR_DIR_PIN, (BitAction)dir);
    motor_start();
}

// 直接反转方向, 不停机不加减速
void motor_change_dir(void)
{
    uint8_t current_dir = GPIO_ReadOutputDataBit(MOTOR_DIR_PORT, MOTOR_DIR_PIN);
    if (current_dir == MOTOR_DIR_REVERSE)
        GPIO_SetBits(MOTOR_DIR_PORT, MOTOR_DIR_PIN);
    else
        GPIO_ResetBits(MOTOR_DIR_PORT, MOTOR_DIR_PIN);
}


/* ---------- 速度控制 ---------- */

// 设定电机目标速度
// 根据电机状态选择不同路径:
//   running=1 (运行中, ISR使用): 通过ISR斜坡加减速, 同时取消之前的停机请求
//   running=0 (未启动/已停机):  直接修改变量, start时生效
void motor_set_speed(uint32_t hz)
{
    g_motor.user_speed_hz = clamp_hz(hz);

    if (g_motor.running) {
        g_motor.target_hz     = g_motor.user_speed_hz;
        g_motor.speed_changing = 1;
        g_motor.stop_pending   = 0;
    } else {
        g_motor.current_hz = g_motor.user_speed_hz;
    }
}

// 软停机: 斜坡减速到 MOTOR_MIN_HZ, 然后在ISR中自动关闭TIM5
void motor_stop(void)
{
    if (g_motor.running) {
        g_motor.target_hz     = MOTOR_MIN_HZ;
        g_motor.speed_changing = 1;
        g_motor.stop_pending   = 1;

        TIM_Cmd(MOTOR_TIM, ENABLE);
        TIM_ITConfig(MOTOR_TIM, MOTOR_TIM_IT_CC, ENABLE);
        // ISR负责: 减速 -> 到MOTOR_MIN_HZ -> 关闭TIM5 -> 清除running
    } else {
        TIM_ITConfig(MOTOR_TIM, MOTOR_TIM_IT_CC, DISABLE);
        TIM_Cmd(MOTOR_TIM, DISABLE);
    }
}

// 软启动: 从 MOTOR_MIN_HZ 起步, 斜坡加速到 user_speed_hz
void motor_start(void)
{
    if (g_motor.running) return;

    g_motor.running        = 1;
    g_motor.current_hz     = MOTOR_MIN_HZ;
    g_motor.target_hz      = g_motor.user_speed_hz;
    g_motor.speed_changing = 1;
    g_motor.stop_pending   = 0;

    set_first_ccr(g_motor.current_hz);                // 设置首次CCR1匹配点
    TIM_Cmd(MOTOR_TIM, ENABLE);
    TIM_ITConfig(MOTOR_TIM, MOTOR_TIM_IT_CC, ENABLE);      // 使能CC1中断, ISR开始工作
}


/* ---------- 脉冲计数API ---------- */

// 重置脉冲计数器
void motor_reset_pulse_count(void)
{
    g_motor.current_pulses = 0;
}

// 获取当前已发脉冲数
uint32_t motor_get_pulse_count(void)
{
    return g_motor.current_pulses;
}

// 设定目标脉冲数, 同时自动清除 target_reached 标志
void motor_set_pulse_count(uint32_t need_pulses)
{
    g_motor.target_pulses = need_pulses;
    g_motor.target_reached = 0;
}

// 查询是否到达目标脉冲数, 读取后自动清零(一次性通知)
// 返回值: 0=未到达  1=已到达
uint8_t motor_is_target_reached(void)
{
    uint8_t temp = g_motor.target_reached;
    if (temp)
        g_motor.target_reached = 0;
    return temp;
}


/* ---------- TIM5 CC1中断服务函数: 脉冲计数 + 速度斜坡 + 软停机 ---------- */
/* 每次CC1匹配(OC硬件自动翻转电平)触发中断, ISR中动态更新CCR控制下一次翻转 */
void MOTOR_TIM_IRQ(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // 确认是CC1中断
    if (TIM_GetITStatus(MOTOR_TIM, MOTOR_TIM_IT_CC) == SET)
    {
        TIM_ClearITPendingBit(MOTOR_TIM, MOTOR_TIM_IT_CC);

        if (!g_motor.running) return;
            // 未运行状态, 忽略(可能残留的CC1中断)

        g_edge_toggle = !g_edge_toggle;               // CC1沿翻转: 每2次=1脉冲
        if (g_edge_toggle) {
            g_motor.current_pulses++;                 // 脉冲计数

            // 速度斜坡: 逐步调整 current_hz 逼近 target_hz
            if (g_motor.speed_changing)
            {
                if (g_motor.current_hz < g_motor.target_hz)
                {
                    g_motor.current_hz += MOTOR_RAMP_STEP_HZ;
                    if (g_motor.current_hz > g_motor.target_hz)
                        g_motor.current_hz = g_motor.target_hz;
                }
                else if (g_motor.current_hz > g_motor.target_hz)
                {
                    g_motor.current_hz -= MOTOR_RAMP_STEP_HZ;
                    if (g_motor.current_hz < g_motor.target_hz)
                        g_motor.current_hz = g_motor.target_hz;
                }
                g_ccr_interval = MOTOR_CALC_CCR_INTERVAL(g_motor.current_hz);
                if (g_motor.current_hz == g_motor.target_hz)
                    g_motor.speed_changing = 0;
            }

            // 脉冲目标到达
            if (!g_motor.target_reached &&
                g_motor.current_pulses >= g_motor.target_pulses)
            {
                g_motor.target_reached = 1;
                xSemaphoreGiveFromISR(MotorTargetReachedSemaphore, &xHigherPriorityTaskWoken);
            }

            // 停机请求: 斜坡结束 + stop_pending
            if (!g_motor.speed_changing && g_motor.stop_pending)
            {
                g_motor.stop_pending = 0;
                g_motor.running      = 0;
                TIM_ITConfig(MOTOR_TIM, MOTOR_TIM_IT_CC, DISABLE);
                TIM_Cmd(MOTOR_TIM, DISABLE);
            }
        }

        // 计算下一个CCR1匹配点
        {
            uint16_t new_ccr = TIM_GetCapture1(MOTOR_TIM) + g_ccr_interval;
            TIM_SetCompare1(MOTOR_TIM, new_ccr);
        }
    }

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
