#include "stm32f10x.h"                  // Device header
#include "motor.h"
#include "FreeRTOS_demo.h"

// 脉冲 , 距离相关
static volatile uint32_t target_pulses = 10000000;  // 目标脉冲数(脉冲数量)
static volatile uint32_t current_pulses = 0;  // 当前已发脉冲数

// 标志位
static volatile uint8_t motor_running = 0;  // 标志位 0表示停止  1表示运动
static volatile uint8_t target_reached_pulses = 0;  // 标志位 0表示未达到目标脉冲 1表示已达到目标脉冲
static volatile uint8_t speed_changing = 0;  // 标志位 0表示不需要改变速度 1表示正在改变速度

// 频率 , 和速度有关
static volatile uint32_t target_hz = 2000;  // 目标频率
static volatile uint32_t current_hz = 2000;  // 当前频率
static volatile uint32_t last_speed_hz = 2000;  // 上次使用的频率
/************************************************************

// 引脚  PA0-->PUL   PE12-->DIR  PB11-->ENA

****************************************************************/
void motor_init()
{
// 开启 GPIOA GPIOB GPIOE TIM5 时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	
// PB11 PE12   推挽输出
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE,&GPIO_InitStructure);
	
// 默认使能低 方向高
	GPIO_ResetBits(GPIOB,GPIO_Pin_11);
	GPIO_SetBits(GPIOE,GPIO_Pin_12);
	
// 输出PUL

// 时基单元初始化
    TIM_InternalClockConfig(TIM5);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;  // 选择TIM5为内部时钟
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;  // 向上计数
    TIM_TimeBaseInitStructure.TIM_Period = 50 - 1;  // ARR
    TIM_TimeBaseInitStructure.TIM_Prescaler = 720 - 1;  // PSC
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;  // 重复计数器，高级定时器才会用到
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure); 

// 输出比较初始化
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  // PWM模式1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  // 输出极性为高
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  // 输出使能
    TIM_OCInitStructure.TIM_Pulse = 25;  // CCR
    TIM_OC1Init(TIM5, &TIM_OCInitStructure);
// 固定占空比输出频率可调 , 步进电机驱动器芯片只需要将占空比保持在 50%

// TIM5_CH1对应映射到PA0
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);  // PA0复用输出
	
// 设置定时器中断，用于计算脉冲数量
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
// 初始化NVIC优先级分组为2(main.c) 抢占 1  响应 2
 	
// TIM使能
    TIM_Cmd(TIM5, ENABLE);
}


/************************************************************

// 方向控制

****************************************************************/
// 设置电机方向
void motor_set_dir(uint8_t dir)
{
    uint8_t current_dir = GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_12);
    
    // 方向相同，不做任何操作
    if (current_dir == dir) return;
	
    motor_stop();
	
    TickType_t start_time = xTaskGetTickCount();
    while (motor_running) {
        vTaskDelay(pdMS_TO_TICKS(1));  // 让出CPU
    }
	
    GPIO_WriteBit(GPIOE, GPIO_Pin_12, (BitAction)dir);
    motor_start();
}

// 改变方向
void motor_change_dir(void)
{
//    motor_stop();
//    // 等待电机完全停止  ****危险****
//    while (motor_running);
    uint8_t current_dir = GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_12);
    if (current_dir == 0)
        GPIO_SetBits(GPIOE, GPIO_Pin_12);
    else
        GPIO_ResetBits(GPIOE, GPIO_Pin_12);
//  motor_start();
}


/************************************************************

// 速度控制

****************************************************************/
// 内部静态函数 
static void set_freq(uint32_t hz)
{
	
    if (hz < 500) hz = 500;
    if (hz > 3000) hz = 3000;
    
    uint32_t arr_add_1 = 100000 / hz;
    TIM_SetAutoreload(TIM5, arr_add_1 - 1);
    TIM_SetCompare1(TIM5, arr_add_1 / 2);
}

// 设置速度 , 设定目标频率 , 但是还未改变频率
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
        speed_changing = 1;
    }
    else
    {
		// 停止状态直接设置
        current_hz = hz;
        set_freq(hz);
    }
}


// 停止电机
void motor_stop(void)
{
    if (motor_running)
    {
        target_hz = 500;
        speed_changing = 1;
    }
    else
    {
        // motor_running==0但TIM5可能仍在运行(由motor_init使能)
        TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
        TIM_Cmd(TIM5, DISABLE);
    }
}

// 启动电机
void motor_start(void)
{
    if (motor_running) return;
    
	
    motor_running = 1;
    current_hz = 500;      // 从500Hz开始
    target_hz = last_speed_hz;      // 恢复到上次的频率目标
    speed_changing = 1;    // 需要加速
    
    set_freq(current_hz);
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM5, ENABLE);
}


/************************************************************

// 脉冲控制

****************************************************************/
// 重置脉冲计数（外部调用,不要动我的静态变量）
void motor_reset_pulse_count(void)
{
    current_pulses = 0;
}

// 获取当前脉冲计数（外部调用,不要动我的静态变量）
uint32_t motor_get_pulse_count(void)
{
    return current_pulses;
}

// 设置目标脉冲数（外部调用,不要动我的静态变量）
void motor_set_pulse_count(uint32_t need_pulses)
{
    target_pulses = need_pulses;
}

// 检查是否到达目标脉冲（调用后自动清除标志）（外部调用,不要动我的静态变量）
uint8_t motor_is_target_reached(void)
{
    uint8_t temp = target_reached_pulses;
    if (temp)
        target_reached_pulses = 0;  // 自动清除
    return temp;
}

/********************************************************

// 改变速度全放在 TIM5_IRQHandler 里 , 其他地方只改变标志位

***********************************************************/

void TIM5_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
        
        if (motor_running)
        {
            current_pulses++;
            
			// 速度渐变（每次变化10Hz，平滑加减速）
            if (speed_changing)
            {
                if (current_hz < target_hz)
                {
                    current_hz += 5;
                    if (current_hz > target_hz) current_hz = target_hz;
                    set_freq(current_hz);
                }
                else if (current_hz > target_hz)
                {
                    current_hz -= 5;
                    if (current_hz < target_hz) current_hz = target_hz;
                    set_freq(current_hz);
                }
                
				// 达到目标频率
                if (current_hz == target_hz)
                {
                    speed_changing = 0;
                    
					// 如果是停止命令（目标频率500），减速完成后停止电机
                    if (target_hz == 500 && current_hz == 500)
                    {
                        motor_running = 0;
                        TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
                        TIM_Cmd(TIM5, DISABLE);
                        return;
                    }
                }
            }
			
			// 判断是否到达目标脉冲
            if (!target_reached_pulses && current_pulses >= target_pulses)
            {
                target_reached_pulses = 1;  // 表示已达到目标脉冲
                //speed_changing = 1;

            }
        }
    }
}
