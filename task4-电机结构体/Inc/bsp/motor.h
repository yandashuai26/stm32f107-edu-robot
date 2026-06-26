#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

// 电机状态结构体
typedef struct {
    uint32_t target_pulses;    // 目标脉冲数
    uint32_t current_pulses;   // 已发脉冲数 (ISR中递增)
    uint32_t target_hz;        // 加减速目标频率
    uint32_t current_hz;       // 当前实际频率
    uint32_t user_speed_hz;    // 用户设定速度, stop/start间保持
    uint8_t  running;          // 0=停止 1=运行中(需要ISR使能)
    uint8_t  speed_changing;   // 0=速度稳定 1=加减速中
    uint8_t  target_reached;   // 脉冲目标到达标志, 读取时自动清零
    uint8_t  stop_pending;     // 停机请求标志: motor_stop()置1, ISR减速到最低速后清零并停机
} motor_t;

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
