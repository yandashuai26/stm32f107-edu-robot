#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "semphr.h"

/* ================================================================
 * 电机控制宏定义 - TIM5 OC翻转脉冲输出
 * 8细分 + PSC=8预分频, 1600脉冲/圈, 默认1圈/秒
 * ARR=0xFFFF 固定, CC1中断动态更新CCR控制脉冲间隔
 * ================================================================ */

/* ---------- 系统时钟 ---------- */
#define MOTOR_TIM                   TIM5
#define MOTOR_TIM_CLK_HZ            72000000UL
#define MOTOR_TIM_PRESCALER         8
#define MOTOR_TIM_BASE_HZ           (MOTOR_TIM_CLK_HZ / MOTOR_TIM_PRESCALER)  // 72M/8=9MHz

/* ---------- 步进电机参数 ---------- */
#define MOTOR_FULL_STEPS_PER_REV    200
#define MOTOR_MICROSTEP             8
#define MOTOR_PULSES_PER_REV        (MOTOR_FULL_STEPS_PER_REV * MOTOR_MICROSTEP)  // 1600 脉冲/圈

/* ---------- 单位转换宏 ---------- */
/* 对外单位: 0.01圈(距离) 和 0.01圈/秒(速度) */
/* 1600脉冲 = 1圈 = 100单位, 1600Hz = 1圈/秒 = 100单位 */
#define MOTOR_PULSES_TO_UNIT(p)     ((uint32_t)((p) / (MOTOR_PULSES_PER_REV / 100)))  // 脉冲→0.01圈
#define MOTOR_UNIT_TO_PULSES(u)     ((uint32_t)((u) * (MOTOR_PULSES_PER_REV / 100)))  // 0.01圈→脉冲
#define MOTOR_HZ_TO_SPEED(h)        ((uint32_t)((h) / (MOTOR_PULSES_PER_REV / 100)))  // Hz→0.01圈/秒
#define MOTOR_SPEED_TO_HZ(s)        ((uint32_t)((s) * (MOTOR_PULSES_PER_REV / 100)))  // 0.01圈/秒→Hz
#define MOTOR_ACCEL_TO_HZ2(a)       ((uint32_t)((a) * (MOTOR_PULSES_PER_REV / 100)))   // 0.01圈/s²→脉冲/s²

/* ---------- 速度参数 (内部仍用Hz) ---------- */
#define MOTOR_MIN_RPS               0.5
#define MOTOR_MAX_RPS               2
#define MOTOR_SPEED_1RPS_HZ         MOTOR_PULSES_PER_REV       // 1圈/秒 = 1600 Hz
#define MOTOR_DEFAULT_SPEED_HZ      MOTOR_SPEED_1RPS_HZ        // 默认 1圈/秒
#define MOTOR_MIN_HZ                ((uint32_t)(MOTOR_SPEED_1RPS_HZ * MOTOR_MIN_RPS))   // 400 Hz
#define MOTOR_MAX_HZ                ((uint32_t)(MOTOR_SPEED_1RPS_HZ * MOTOR_MAX_RPS))   // 3200 Hz
#define MOTOR_RAMP_STEP_HZ          10                           // 加减速步进 (Hz/脉冲)

/* ---------- 距离目标 (对外单位: 0.01圈) ---------- */
#define MOTOR_DEFAULT_TARGET_UNIT   625000UL               // 默认目标 6250圈, 足够大暂不触发
#define MOTOR_MIDPOINT_UNIT         175                    // 中点 1.75圈 = 175单位

/* ---------- 内部宏 (保留用于ISR) ---------- */
#define MOTOR_MIDPOINT_REVS         1.75
#define MOTOR_MIDPOINT_REVS_NUM     7
#define MOTOR_MIDPOINT_REVS_DEN     4
#define MOTOR_MIDPOINT_PULSES       ((uint32_t)(MOTOR_PULSES_PER_REV * MOTOR_MIDPOINT_REVS_NUM / MOTOR_MIDPOINT_REVS_DEN))  // 2800 脉冲(内部用)

/* CCR间隔计算: 9MHz / hz = 每个脉冲的TIM5计数间隔 */
/* ARR=0xFFFF 固定, CC1中断中: 新CCR = 旧CCR + interval (16位自动截断) */
#define MOTOR_CALC_CCR_INTERVAL(hz) ((uint16_t)((MOTOR_TIM_BASE_HZ) / 2 / (hz)))  // 9MHz/2/hz, OC翻转2次=1脉冲

/* ---------- GPIO Init 参数 ---------- */
#define MOTOR_GPIO_MODE             GPIO_Mode_Out_PP        // 推挽输出
#define MOTOR_GPIO_SPEED            GPIO_Speed_50MHz        // 50MHz
#define MOTOR_PUL_GPIO_MODE         GPIO_Mode_AF_PP         // 复用推挽输出

/* ---------- TIM 时基 Init 参数 ---------- */
#define MOTOR_TIM_ARR               0xFFFF                  // ARR固定最大值
#define MOTOR_TIM_CKD               TIM_CKD_DIV1            // 时钟不分频
#define MOTOR_TIM_COUNTER_MODE      TIM_CounterMode_Up      // 向上计数

/* ---------- TIM OC Init 参数 ---------- */
#define MOTOR_TIM_OC_MODE           TIM_OCMode_Toggle       // OC翻转模式
#define MOTOR_TIM_OC_POLARITY       TIM_OCPolarity_Low      // 有效电平低
#define MOTOR_TIM_OC_STATE          TIM_OutputState_Enable  // 输出使能

/* ---------- NVIC Init 参数 ---------- */
#define MOTOR_TIM_IRQn              TIM5_IRQn               // TIM5中断通道
#define MOTOR_TIM_PREEMPT_PRIO      1                       // 抢占优先级
#define MOTOR_TIM_SUB_PRIO          2                       // 响应优先级

/* ---------- TIM 中断源 ---------- */
#define MOTOR_TIM_IT_CC             TIM_IT_CC1              // CC1中断源 (OC翻转匹配)
#define MOTOR_TIM_RCC               RCC_APB1Periph_TIM5     // TIM5外设时钟
#define MOTOR_TIM_IRQ               TIM5_IRQHandler
/* ---------- GPIO 引脚 ---------- */
#define MOTOR_PUL_PORT              GPIOA
#define MOTOR_PUL_PIN               GPIO_Pin_0
#define MOTOR_PUL_CLK               RCC_APB2Periph_GPIOA

#define MOTOR_DIR_PORT              GPIOE
#define MOTOR_DIR_PIN               GPIO_Pin_12
#define MOTOR_DIR_CLK               RCC_APB2Periph_GPIOE

#define MOTOR_ENA_PORT              GPIOB
#define MOTOR_ENA_PIN               GPIO_Pin_11
#define MOTOR_ENA_CLK               RCC_APB2Periph_GPIOB

/* ---------- 方向 ---------- */
#define MOTOR_DIR_FORWARD           1
#define MOTOR_DIR_REVERSE           0

/* ---------- 梯形加减速 ---------- */
#define ALPHA               ((float)(2*3.14159/MOTOR_SPEED_1RPS_HZ))       // α= 2*pi/spr
#define T1_FREQ             ((float)(0.676*MOTOR_TIM_BASE_HZ/100))         // 0.676·ft/100
#define A_SQ                ((float)(2*10000000000*ALPHA))                 // 2α·10^10
/* ================================================================
 * 电机状态结构体
 * ================================================================ */
extern SemaphoreHandle_t motor_target_reached_semaphore;  // 目标到达信号量(ISR give, vTaskMotorReached take)

/* 电机运行状态 */
typedef enum {
	MOTOR_STOP = 0,         // 停止
	MOTOR_ACCEL,            // 加速
	MOTOR_RUN,              // 匀速
	MOTOR_DECEL,            // 减速
	MOTOR_WAIT,             // 等待(到位后暂停)
} MotorState;

/* 电机控制结构体 */
typedef struct {
	/* 脉冲与位置 */
	uint32_t step_count;        // 当前已走步数(绝对值), ISR中递增
	uint32_t total_steps;       // 总目标步数(绝对值)

	/* 速度控制: 脉冲半周期(0.5个方波)的定时器计数值 */
	uint32_t PULSE;             // 当前脉冲半周期, 即 step_delay/2
	uint32_t target_pulse;      // 目标速度对应的脉冲半周期(RUN阶段使用)
	uint32_t min_pulse;         // 最小脉冲周期(对应最大速度)
	uint32_t user_speed_hz;     // 用户设定速度 (内部: Hz)

	/* 梯形加减速参数 */
	uint32_t accel_count;       // 加速阶段公式计数器(ISR中递增, 用于分母)
	uint32_t accel_limit;       // 加速阶段总步数(固定值, 用于状态切换判断)
	uint32_t decel_start;       // 开始减速的步进位置(绝对值)
	uint32_t decel_val;         // 减速递减计数器(ISR中--, <=1时退出, 含off-by-one补偿)
	uint32_t accel;             // 加速度 (脉冲/s²)
	uint32_t decel;             // 减速度 (脉冲/s²)

	/* 状态标志 */
	uint8_t  run_state;         // MotorState: STOP/ACCEL/RUN/DECEL/WAIT
	uint8_t  dir;               // 方向: 0=反向 1=正向
	uint8_t  target_reached;    // 目标到达标志, 读取时自动清零
} motor_t;


/* ================================================================
 * 函数声明
 * ================================================================ */
void vMotorTargetSemaphoreCreate(void);
void MotorInit(void);
void MotorSetDir(uint8_t dir);
void MotorChangeDir(void);
void MotorSetSpeed(uint32_t speed);      // 参数: 0.01圈/秒
void MotorStop(void);
void MotorStart(void);
void MotorResetUnitCount(void);          // 重置距离计数
uint32_t MotorGetUnitCount(void);        // 获取已走距离, 返回0.01圈
void MotorSetTargetUnit(uint32_t unit);  // 设定目标距离, 参数: 0.01圈
void MotorSetAccel(uint32_t accel);       // 设定加速度
void MotorSetDecel(uint32_t decel);       // 设定减速度
uint8_t MotorIsTargetReached(void);

#endif
