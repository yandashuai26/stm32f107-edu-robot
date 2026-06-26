#ifndef __MOTOR_H
#define __MOTOR_H

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
