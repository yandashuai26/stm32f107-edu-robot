#ifndef __BSP_H
#define __BSP_H

//自己的头文件
#include "stm32f10x.h"                  // Device header
#include "led.h"
#include "485.h"
#include "oled.h"
#include "stdlib.h"
//#include "oledfont.h"  
#include "gpio.h"
#include "motor.h"
#include "timer.h"
#include "usart.h"  
#include "stdio.h"
#include "stdint.h"
#include "485.h"
#include <string.h>

void bsp_init(void);
void delay_ms(uint32_t ms);
#endif
