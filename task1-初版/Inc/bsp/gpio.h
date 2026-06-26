#ifndef __GPIO_H
#define __GPIO_H

void LED_Init(void);
void limit_Init(void);
uint8_t motor_is_limit_reached(void);
void disable_limit_interrupt(void);
void enable_limit_interrupt(void);
#endif
