#include "stm32f10x.h"                  // Device header
#include "motor.h"

// ==================== 电机参数 ====================
// 电机 , 脉冲变量
static volatile uint32_t target_pulses = 10000000;   // 目标脉冲(总脉冲数)
static volatile uint32_t current_pulses = 0;         // 当前已发脉冲数

// 标志位
static volatile uint8_t motor_running = 0;      // 0:停止, 1:运动
static volatile uint8_t target_reached_pulses = 0;  // 0:未达到, 1:已达到
static volatile uint8_t speed_changing = 0;     // 0:不需要改变速度, 1:正在改变速度

// 频率相关
static volatile uint32_t target_hz = 2000;       // 目标频率 (Hz)
static volatile uint32_t current_hz = 2000;      // 当前频率 (Hz)
static volatile uint32_t last_speed_hz = 2000;   // 上次使用的频率

// Toggle 模式专用：脉冲周期对应的定时器计数值
static volatile uint32_t current_period_ticks = 0;   // 当前脉冲周期（微秒计数值）
static volatile uint32_t target_period_ticks = 0;    // 目标脉冲周期
static volatile uint32_t last_period_ticks = 0;      // 上次的周期

// ==================== 硬件引脚定义 ====================
// PA0 --> PUL (脉冲输出，TIM5_CH1)
// PE12 --> DIR (方向控制)
// PB11 --> ENA (使能控制)

// ==================== 函数声明 ====================
static void set_freq(uint32_t hz);
static uint32_t hz_to_ticks(uint32_t hz);

// ==================== 频率转定时器计数值 ====================
// 定时器时钟 = 72MHz / 72 = 1MHz，每个计数 = 1us
// 周期(us) = 1,000,000 / 频率(Hz)
// 一个完整的脉冲周期 = 2次翻转，所以定时器比较周期 = 周期(us) / 2
static uint32_t hz_to_ticks(uint32_t hz)
{
    if (hz < 500) hz = 500;
    if (hz > 3000) hz = 3000;
    
    // 脉冲周期(us) = 1,000,000 / hz
    // Toggle模式：每次比较中断翻转一次，所以需要半个周期翻转一次
    // 比较值 = (1,000,000 / hz) / 2 = 500,000 / hz
    uint32_t half_period_us = 500000 / hz;
    
    if (half_period_us < 10) half_period_us = 10;
    if (half_period_us > 2000) half_period_us = 2000;
    
    return half_period_us;
}

// ==================== 设置输出频率 ====================
static void set_freq(uint32_t hz)
{
    if (hz < 500) hz = 500;
    if (hz > 3000) hz = 3000;
    
    // 获取对应的定时器比较值
    uint32_t ticks = hz_to_ticks(hz);
    
    // 保存当前周期
    current_period_ticks = ticks;
    
    // 设置定时器比较值
    TIM_SetCompare1(TIM5, ticks);
}

// ==================== 电机初始化 ====================
void motor_init(void)
{
    // 开启 GPIOA GPIOB GPIOE TIM5 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    
    // ==================== PB11 (ENA) PE12 (DIR) 推挽输出 ====================
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    // 默认使能（低电平使能），方向默认正转
    GPIO_ResetBits(GPIOB, GPIO_Pin_11);   // ENA = 0，使能驱动器
    GPIO_ResetBits(GPIOE, GPIO_Pin_12);   // DIR = 0，正转
    
    // ==================== PA0 (PUL) 配置为 TIM5_CH1 复用输出 ====================
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // ==================== 定时器5配置为 Toggle 模式 ====================
    // 时基单元初始化
    TIM_InternalClockConfig(TIM5);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF;        // ARR 设最大值，让定时器自由计数
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;     // 72MHz / 72 = 1MHz (1us/计数)
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);
    
    // 输出比较配置为 Toggle 模式
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;   // 关键：Toggle 模式
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1000;                 // 初始值 1000us 半周期 → 500Hz
    TIM_OC1Init(TIM5, &TIM_OCInitStructure);
    
    // 不使能预装载，立即生效
    TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Disable);
    
    // ==================== 配置定时器更新中断（用于脉冲计数和加减速）====================
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 开启捕获/比较中断（Toggle 模式需要这个中断来更新比较值）
    TIM_ITConfig(TIM5, TIM_IT_CC1, ENABLE);
    
    // 使能定时器
    TIM_Cmd(TIM5, ENABLE);
}

// ==================== 方向控制 ====================
// 设置电机方向
void motor_set_dir(uint8_t dir)
{
    uint8_t current_dir = GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_12);
    
    // 方向相同不做任何操作
    if (current_dir == dir) return;
    
    // 先停止电机（减速停止）
    motor_stop();
    
    // 等待电机完全停止
    while (motor_running);
    
    // 改变方向
    GPIO_WriteBit(GPIOE, GPIO_Pin_12, (BitAction)dir);
    
    // 重新启动
    motor_start();
}

// 改变方向
void motor_change_dir(void)
{
    uint8_t current_dir = GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_12);
    if (current_dir == 0)
        GPIO_SetBits(GPIOE, GPIO_Pin_12);
    else
        GPIO_ResetBits(GPIOE, GPIO_Pin_12);
}

// ==================== 速度控制 ====================
// 设定速度（目标频率）
void motor_set_speed(uint32_t hz)
{
    if (hz < 500) hz = 500;
    if (hz > 3000) hz = 3000;
    
    // 记录当前速度
    last_speed_hz = hz;
    
    if (motor_running)
    {
        // 运动中改变速度，使用加减速
        target_hz = hz;
        target_period_ticks = hz_to_ticks(hz);
        speed_changing = 1;
    }
    else
    {
        // 停止状态直接生效
        current_hz = hz;
        current_period_ticks = hz_to_ticks(hz);
        TIM_SetCompare1(TIM5, current_period_ticks);
    }
}

// 停止电机（减速停止）
void motor_stop(void)
{
    if (motor_running)
    {
        // 减速到500Hz后停止
        target_hz = 500;
        target_period_ticks = hz_to_ticks(500);
        speed_changing = 1;
    }
}

// 启动电机
void motor_start(void)
{
    if (motor_running) return;
    
    motor_running = 1;
    current_hz = 500;                    // 从500Hz开始
    target_hz = last_speed_hz;           // 恢复到上次的频率目标
    
    current_period_ticks = hz_to_ticks(500);
    target_period_ticks = hz_to_ticks(target_hz);
    speed_changing = 1;                  // 需要变速
    
    // 设置初始比较值
    TIM_SetCounter(TIM5, 0);
    TIM_SetCompare1(TIM5, current_period_ticks);
    
    // 清除中断标志并开启中断
    TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);
    TIM_ITConfig(TIM5, TIM_IT_CC1, ENABLE);
    TIM_Cmd(TIM5, ENABLE);
}

// ==================== 脉冲计数 ====================
// 将脉冲计数器清零
void motor_reset_pulse_count(void)
{
    current_pulses = 0;
}

// 获取当前脉冲数
uint32_t motor_get_pulse_count(void)
{
    return current_pulses;
}

// 设定目标脉冲数
void motor_set_pulse_count(uint32_t need_pulses)
{
    target_pulses = need_pulses;
}

// 检查是否到达目标脉冲（调用后自动清除标志）
uint8_t motor_is_target_reached(void)
{
    uint8_t temp = target_reached_pulses;
    if (temp)
        target_reached_pulses = 0;
    return temp;
}

// ==================== 定时器中断（核心：Toggle 模式 + 加减速）====================
void TIM5_IRQHandler(void)
{
    // ========== 捕获/比较中断（Toggle 模式核心）==========
    if (TIM_GetITStatus(TIM5, TIM_IT_CC1) == SET)
    {
        TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);
        
        if (motor_running)
        {
            // 每次翻转，脉冲数 +1
            current_pulses++;
            
            // 获取当前计数值，计算下一次比较值
            uint32_t current_cnt = TIM_GetCounter(TIM5);
            uint32_t next_compare = current_cnt + current_period_ticks;
            TIM_SetCompare1(TIM5, next_compare);
            
            // ========== 加减速处理 ==========
            if (speed_changing)
            {
                if (current_hz < target_hz)
                {
                    // 加速：每次增加 20Hz
                    current_hz += 20;
                    if (current_hz > target_hz) current_hz = target_hz;
                    current_period_ticks = hz_to_ticks(current_hz);
                }
                else if (current_hz > target_hz)
                {
                    // 减速：每次减少 20Hz
                    if (current_hz >= 20)
                        current_hz -= 20;
                    else
                        current_hz = target_hz;
                    if (current_hz < target_hz) current_hz = target_hz;
                    current_period_ticks = hz_to_ticks(current_hz);
                }
                
                // 达到目标频率
                if (current_hz == target_hz)
                {
                    speed_changing = 0;
                    
                    // 如果是停止命令（目标频率500Hz，减速完成）
                    if (target_hz == 500 && current_hz == 500)
                    {
                        motor_running = 0;
                        TIM_ITConfig(TIM5, TIM_IT_CC1, DISABLE);
                        return;
                    }
                }
            }
            
            // ========== 判断是否到达目标脉冲 ==========
            if (!target_reached_pulses && current_pulses >= target_pulses)
            {
                target_reached_pulses = 1;
                // 到达目标脉冲后自动减速停止
                target_hz = 500;
                target_period_ticks = hz_to_ticks(500);
                speed_changing = 1;
            }
        }
    }
    
    // ========== 更新中断（备用，可用于超时保护）==========
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
        // 计数器溢出，不做处理（ARR = 0xFFFF，不会溢出）
    }
}