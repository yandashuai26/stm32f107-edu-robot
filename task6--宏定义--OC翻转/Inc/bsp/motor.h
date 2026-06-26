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

/* ---------- 速度参数 ---------- */
#define MOTOR_MIN_RPS               0.5
#define MOTOR_MAX_RPS               2
#define MOTOR_SPEED_1RPS_HZ         MOTOR_PULSES_PER_REV       // 1圈/秒 = 1600 Hz
#define MOTOR_DEFAULT_SPEED_HZ      MOTOR_SPEED_1RPS_HZ        // 默认 1圈/秒
#define MOTOR_MIN_HZ                ((uint32_t)(MOTOR_SPEED_1RPS_HZ * MOTOR_MIN_RPS))   // 400 Hz
#define MOTOR_MAX_HZ                ((uint32_t)(MOTOR_SPEED_1RPS_HZ * MOTOR_MAX_RPS))   // 3200 Hz
#define MOTOR_RAMP_STEP_HZ          10                           // 加减速步进 (Hz/脉冲)

/* ---------- 脉冲目标 ---------- */
#define MOTOR_DEFAULT_TARGET_PULSES 10000000UL              // 默认目标 - 足够大暂不触发

/* ---------- 中点定位 (1.75圈) ---------- */
#define MOTOR_MIDPOINT_REVS         1.75
#define MOTOR_MIDPOINT_REVS_NUM     7
#define MOTOR_MIDPOINT_REVS_DEN     4
#define MOTOR_MIDPOINT_PULSES       ((uint32_t)(MOTOR_PULSES_PER_REV * MOTOR_MIDPOINT_REVS_NUM / MOTOR_MIDPOINT_REVS_DEN))  // 2800 脉冲

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


/* ================================================================
 * 电机状态结构体
 * ================================================================ */
extern SemaphoreHandle_t MotorTargetReachedSemaphore;  // 目标到达信号量(ISR give, vTaskMotorReached take)

typedef struct {
    uint32_t target_pulses;    // 目标脉冲数
    uint32_t current_pulses;   // 已发脉冲数 (ISR中递增)
    uint32_t target_hz;        // 加减速目标频率
    uint32_t current_hz;       // 当前实际频率
    uint32_t user_speed_hz;    // 用户设定速度, stop/start间保持
    uint8_t  running;          // 0=停止 1=运行中(需要ISR使用)
    uint8_t  speed_changing;   // 0=速度稳定 1=加减速中
    uint8_t  target_reached;   // 脉冲目标到达标志, 读取时自动清零
    uint8_t  stop_pending;     // 停机请求标志: motor_stop()置1, ISR减速到最低后停机
} motor_t;


/* ================================================================
 * 函数声明
 * ================================================================ */
void motor_target_semaphore_create(void);
void motor_init(void);
void motor_set_dir(uint8_t dir);
void motor_change_dir(void);
void motor_set_speed(uint32_t hz);
void motor_stop(void);
void motor_start(void);
void motor_reset_pulse_count(void);
uint32_t motor_get_pulse_count(void);
void motor_set_pulse_count(uint32_t need_pulses);
uint8_t motor_is_target_reached(void);

#endif
