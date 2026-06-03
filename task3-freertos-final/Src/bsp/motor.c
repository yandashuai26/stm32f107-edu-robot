#include "stm32f10x.h"                  // Device header
#include "motor.h"
#include "FreeRTOS_demo.h"

#define MOTOR_MIN_HZ       500           // 电机最低频率
#define MOTOR_MAX_HZ       3000          // 电机最高频率
#define MOTOR_RAMP_STEP    1            // 每次加减速的步进值(Hz)

static motor_t g_motor;                  // 电机全局状态

// 限制频率大小 , 等于 MOTOR_MIN_HZ 在ISR中会停下
static uint32_t clamp_hz(uint32_t hz)
{
    if (hz < MOTOR_MIN_HZ) return MOTOR_MIN_HZ;
    if (hz > MOTOR_MAX_HZ) return MOTOR_MAX_HZ;
    return hz;
}

// 突然改变速度大小 , 外部无法调用
// 直接设置TIM5的PWM频率, 占空比固定50%
// ARR = 100000/hz - 1, CCR = ARR/2
static void set_freq(uint32_t hz)
{
    uint32_t arr_add_1 = 100000 / hz;                  // ARR+1 = 定时器时钟/hz = 100k/hz
    TIM_SetAutoreload(TIM5, arr_add_1 - 1);            // 设置自动重装载值
    TIM_SetCompare1(TIM5, arr_add_1 / 2);              // 设置比较值, 占空比50%
}


/************************************************************

// 电机初始化  PA0-->PUL   PE12-->DIR   PB11-->ENA(低电平有效)
// 初始化后TIM5输出1000Hz PWM, ENA低电平使能电机驱动
// TIM5更新中断暂不开启, 待motor_start()才打开

****************************************************************/
void motor_init()
{
	// 初始化电机结构体, 设定默认值 
    memset(&g_motor, 0, sizeof(g_motor));             // 全部清零
    g_motor.target_pulses  = 10000000;                // 默认目标脉冲数(很大, 不会通过达到脉冲而停下)
    g_motor.current_hz     = 1000;                    // 当前频率初始1000Hz
    g_motor.target_hz      = 1000;                    // 目标频率初始1000Hz
    g_motor.user_speed_hz  = 1000;                    // 用户设定速度初始1000Hz
    // running/speed_changing/target_reached/stop_pending 均为0

	// 使能 GPIOA , GPIOB , GPIOE , TIM5时钟 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   // PA0  -> TIM5_CH1 PWM输出
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);   // PB11 -> ENA 使能
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);   // PE12 -> DIR 方向
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,  ENABLE);   // TIM5 -> PWM产生+脉冲计数

	// 配置控制引脚 PB11(ENA) PE12(DIR) 
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        // 推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;              // PB11 -> ENA
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;              // PE12 -> DIR
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOB, GPIO_Pin_11);                     // ENA低电平 -> 使能电机驱动
    GPIO_SetBits(GPIOE,  GPIO_Pin_12);                      // DIR高电平 -> 默认方向(往没有雷达的方向)

	// 配置TIM5时基: 72MHz/720=100kHz, 100kHz/100=1000Hz 
    TIM_InternalClockConfig(TIM5);                          // TIM5使用内部时钟
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     // 不分频
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;                 // ARR=99, PWM频率=1000Hz
    TIM_TimeBaseInitStructure.TIM_Prescaler = 720 - 1;              // PSC=719, 定时器时钟=100kHz
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;            // 高级定时器用, 这里用不到
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);

	// 配置TIM5_CH1 PWM输出, 占空比固定50%
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);                       // 先结构体赋初始值
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             // PWM模式1: CNT<CCR时输出有效电平
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     // 有效电平为高
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 输出使能
    TIM_OCInitStructure.TIM_Pulse = 50;                           // CCR=50, 占空比=50/100=50%
    TIM_OC1Init(TIM5, &TIM_OCInitStructure);

	// 配置PA0为复用推挽输出 -> TIM5_CH1(PUL) 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;              // 复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;                    // PA0
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 配置TIM5中断优先级(NVIC分组2,在FreeRTOS_demo.c)
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;              // TIM5中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;    // 抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;           // 子优先级2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;              // NVIC层使能
    NVIC_Init(&NVIC_InitStructure);

	//未使能中断

	// 使能TIM5, PWM开始输出
    TIM_Cmd(TIM5, ENABLE);                                      // 定时器开始计数, PWM输出1000Hz
    // 此时电机以1000Hz默认速度运转, 但没有脉冲计数和加减速功能
}


/************************************************************

// 方向控制

****************************************************************/

// 设置电机方向, 带加减速保护
// 步骤: 软停机 -> 等待完全停止 -> 切换方向引脚 -> 重新启动
// 阻塞约250~500ms (取决于当前速度)
void motor_set_dir(uint8_t dir)
{
    uint8_t current_dir = GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_12);  // 读取当前方向
    if (current_dir == dir) return;                                     // 方向相同, 不需要操作

    motor_stop();                                                       // 斜坡减速停机

    while (g_motor.running) {        // 等待ISR完成停机流程
        vTaskDelay(pdMS_TO_TICKS(1));                                   // 让出CPU, 1ms后重试
    }                                                                   // running==0时退出

    GPIO_WriteBit(GPIOE, GPIO_Pin_12, (BitAction)dir);                  // 切换方向引脚
    motor_start();                                                      // 斜坡加速启动
}

// 直接翻转方向, 不停机不加减速
// 由限位开关任务在触发瞬间调用, 要求快速响应
void motor_change_dir(void)
{
    uint8_t current_dir = GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_12);  // 读取当前方向
    if (current_dir == 0)
        GPIO_SetBits(GPIOE, GPIO_Pin_12);    // 当前低 -> 翻转为高
    else
        GPIO_ResetBits(GPIOE, GPIO_Pin_12);  // 当前高 -> 翻转为低
}


/************************************************************

// 速度控制

****************************************************************/

// 设置电机目标速度
// 内部根据电机状态选择不同策略:
//   running=1 (运行中, ISR使能): 启动加减速斜坡, 同时取消待处理的停机请求
//   running=0 (空闲/停机):     直接修改PWM频率立即生效
//     空闲态: TIM5运行中, ISR关闭 -> PWM频率直接改变电机转速
//     停机态: TIM5关闭         -> 仅改寄存器值, motor_start()启动时仍从500Hz斜坡起步
void motor_set_speed(uint32_t hz)
{
    g_motor.user_speed_hz = clamp_hz(hz);             // 限幅并记录用户期望速度

    if (g_motor.running) {
        // 电机正在运行, 通过ISR斜坡加减速
        g_motor.target_hz     = g_motor.user_speed_hz; // 更新斜坡目标
        g_motor.speed_changing = 1;                    // 触发ISR中的斜坡逻辑
        g_motor.stop_pending   = 0;                    // 取消之前可能触发的停机请求
    } else {
        // 电机未运行(空闲或已停机), 直接设置频率即可
        g_motor.current_hz = g_motor.user_speed_hz;    // 同步当前频率
        set_freq(g_motor.current_hz);                  // 直接写入TIM5寄存器
    }
}

// 软停机: 斜坡减速到MOTOR_MIN_HZ(500Hz), 然后在ISR中自动关闭TIM5
// 电机有3种状态, 停机策略不同:
//   running=1 (运行中): 设置停机标志, 交由ISR逐步减速到500Hz后自动关闭TIM5
//   running=0 (空闲/已停机): 直接关闭TIM5及其中断
void motor_stop(void)
{
    if (g_motor.running) {
        // 电机正在运行, 走软停机流程
        g_motor.target_hz     = MOTOR_MIN_HZ;     // 目标频率设为最低(500Hz)
        g_motor.speed_changing = 1;                // 触发ISR斜坡减速
        g_motor.stop_pending   = 1;                // 告知ISR: 减速到500Hz后执行停机

        TIM_Cmd(TIM5, ENABLE);                     // 确保TIM5在运行
        TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE); // 确保更新中断已使能
        // 后续由ISR完成: 减速->到达500Hz->关闭TIM5->清零running
    } else {
        // 电机未运行, 直接关闭TIM5即可
        TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE); // 关闭更新中断
        TIM_Cmd(TIM5, DISABLE);                     // 关闭TIM5, PWM停止输出
    }
}

// 启动电机: 从MOTOR_MIN_HZ(500Hz)起步, 逐步斜坡加速到user_speed_hz
// 已在运行中则直接返回, 避免重复启动
void motor_start(void)
{
    if (g_motor.running) return;                    // 已在运行, 不重复启动

    g_motor.running        = 1;                     // 标记运行状态
    g_motor.current_hz     = MOTOR_MIN_HZ;          // 从最低速500Hz起步
    g_motor.target_hz      = g_motor.user_speed_hz; // 斜坡目标为用户设定速度
    g_motor.speed_changing = 1;                     // 启动斜坡
    g_motor.stop_pending   = 0;                     // 清除停机标志

    set_freq(g_motor.current_hz);                   // 先输出500Hz
    TIM_Cmd(TIM5, ENABLE);                          // 确保TIM5运行
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);      // 使能更新中断, ISR开始工作
    // ISR会逐步将频率从500Hz加到user_speed_hz
}


/************************************************************

// 脉冲计数API

****************************************************************/

// 脉冲计数清零, 通常在限位触发或开始新行程时调用
void motor_reset_pulse_count(void)
{
    g_motor.current_pulses = 0;                     // 已发脉冲数归零
}

// 读取当前已发出的脉冲数
uint32_t motor_get_pulse_count(void)
{
    return g_motor.current_pulses;
}

// 设定目标脉冲数, 到达后target_reached标志自动置1
void motor_set_pulse_count(uint32_t need_pulses)
{
    g_motor.target_pulses = need_pulses;            // 设定目标值, ISR中会不断比较
}

// 查询是否到达目标脉冲数, 读取后自动清零(一次性通知)
// 返回值: 0=未到达  1=已到达
uint8_t motor_is_target_reached(void)
{
    uint8_t temp = g_motor.target_reached;          // 先读出
    if (temp)
        g_motor.target_reached = 0;                 // 读后自动清零, 防止重复通知
    return temp;
}


/************************************************************

// TIM5中断服务函数: 脉冲计数 + 速度斜坡 + 软停机
// 每次TIM5更新事件(即每个PWM周期)触发一次
// 外部函数只负责修改标志位, 真正的速度变化在此ISR内部完成

****************************************************************/
void TIM5_IRQHandler(void)
{
    // 确认是更新中断
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);       // 清除中断标志

        if (!g_motor.running) return;                     // 非运行态, 不处理
                                                          // (TIM5可能在运行但ISR不应该计数)

        g_motor.current_pulses++;                         // 本周期发出一个脉冲, 计数+1

        // 速度斜坡: 逐步调整current_hz逼近target_hz
        if (g_motor.speed_changing)
        {
            // 需要加速: current_hz < target_hz
            if (g_motor.current_hz < g_motor.target_hz)
            {
                g_motor.current_hz += MOTOR_RAMP_STEP;    // 每次+10Hz
                if (g_motor.current_hz > g_motor.target_hz)
                    g_motor.current_hz = g_motor.target_hz; // 防止超调
                set_freq(g_motor.current_hz);             // 更新TIM5频率寄存器
            }
            // 需要减速: current_hz > target_hz
            else if (g_motor.current_hz > g_motor.target_hz)
            {
                g_motor.current_hz -= MOTOR_RAMP_STEP;    // 每次-10Hz
                if (g_motor.current_hz < g_motor.target_hz)
                    g_motor.current_hz = g_motor.target_hz; // 防止超调
                set_freq(g_motor.current_hz);             // 更新TIM5频率寄存器
            }

            // 斜坡完成: 当前频率已等于目标频率
            if (g_motor.current_hz == g_motor.target_hz)
            {
                g_motor.speed_changing = 0;               // 退出斜坡模式, 速度稳定
            }
        }

        // 脉冲目标检测: 判断是否已发出足够数量的脉冲
        if (!g_motor.target_reached &&
            g_motor.current_pulses >= g_motor.target_pulses)
        {
            g_motor.target_reached = 1;                   // 置位标志, 由motor_is_target_reached()查询
        }

        // 停机完成判断: 斜坡结束 + stop_pending置位 (stop_pending置位在motor_stop置 1 )
        // 只有当减速斜坡结束(speed_changing==0) 且 停机请求有效(stop_pending==1)
        if (!g_motor.speed_changing && g_motor.stop_pending)
        {
            g_motor.stop_pending = 0;                     // 清除停机请求
            g_motor.running      = 0;                     // 标记停止
            TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);   // 关闭更新中断
            TIM_Cmd(TIM5, DISABLE);                       // 关闭TIM5, PWM停止
            // 此时motor_set_dir()中的while(g_motor.running)循环可以退出
        }
    }
}
