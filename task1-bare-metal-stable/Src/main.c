// main.c
#include "stm32f10x.h"                  // Device header
#include "bsp.h"

// 1ms定时器
volatile uint32_t sysTick_ms = 0;
volatile uint8_t LED_MODE = 1;  // LED模式  0:灭  1:亮  2:慢闪   3:快闪
extern volatile uint8_t  USART3_RxBuffer[];  // 接收不定长字符串的缓冲区
extern volatile uint16_t USART3_RxIndex;    // 中断函数中使用,记录接收到的字符数量
extern volatile uint8_t  USART3_RxFinished; // 1 表示接收到一个完整数据帧
extern volatile uint16_t USART3_RxCount;  // 接收到的字符数量

static uint8_t motor_is_runing = 1; // 0表示电机没有运动  1表示电机在运动

// 读取数据的变量
uint32_t temp = 20;
uint32_t dis = 200;
char display_buffer[20];  // 用于字符串显示

// 滤波相关变量
static uint8_t motor_stopped_by_obstacle = 0;  // 0表示没有因为障碍物停止  1表示因为障碍物停止
static uint8_t obstacle_clear_count = 0;       // 障碍物清除计数（连续3次都>10）
int main(void)
{
	// 初始化NVIC优先级分组为2
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	bsp_init();
	delay_ms(100);
	motor_set_speed(1000); 
	// 设定目标脉冲,表示到达中点(还未到达)
	motor_set_pulse_count(10000000);
	motor_start();
	while(1)
	{
		// 每 5s 读取数据
		if(sysTick_ms == 1000)
		{
			// 读取数据
			temp = get_tempture();
			dis = get_distance();
			
			// 打印数据
			printf("temp:%d\r\n", temp);
			printf("dis:%d\r\n", dis);

			// 打印到OLED
			sprintf(display_buffer, "dis:%04d", dis);
			OLED_ShowString(2, 2, (uint8_t*)display_buffer, 16); 
			sprintf(display_buffer, "temp:%04d", temp);
			OLED_ShowString(2, 4, (uint8_t*)display_buffer, 16); 			
		}
		
		// 限位开关处理
		if( motor_is_limit_reached())
		{

			// 停止期间禁用限位中断,失能中断
			disable_limit_interrupt();
			
			// 暂时停止
			motor_stop();
			printf("stop\r\n");
			OLED_ShowString(2, 6, (uint8_t*)"stop", 16);
			delay_ms(300);
			
			// 读取数据
			temp = get_tempture();
			dis = get_distance();
			
			// 打印数据
			printf("temp:%d\r\n", temp);
			printf("dis:%d\r\n", dis);

			// 打印到OLED
			sprintf(display_buffer, "dis:%04d", dis);
			OLED_ShowString(2, 2, (uint8_t*)display_buffer, 16); 
			sprintf(display_buffer, "temp:%04d", temp);
			OLED_ShowString(2, 4, (uint8_t*)display_buffer, 16); 	
			delay_ms(300);
			
			// 清除显示
			OLED_ShowString(2, 6, (uint8_t*)"    ", 16);
			// 在停止状态下安全换向
			motor_change_dir();
			motor_set_pulse_count(3050);
			motor_start();
			//delay_ms(300);
			motor_reset_pulse_count();
			delay_ms(300);
				
			// 使能中断
			enable_limit_interrupt();
			
			// 恢复电机运动
			motor_is_runing = 1;
			
		}
		// 达到目标脉冲(中点) 
		if( motor_is_target_reached())
		{
			// 清除脉冲计数
			motor_reset_pulse_count();
			// 重新设定目标,达不到
			motor_set_pulse_count(10000000);

			// 停止期间禁用限位中断,失能中断
			disable_limit_interrupt();
			// 暂时停止
			motor_stop();
			printf("stop\r\n");
			OLED_ShowString(2, 0, (uint8_t*)"stop", 16);
			delay_ms(300);
			
			// 读取数据
			temp = get_tempture();
			dis = get_distance();
			
			// 打印数据
			printf("temp:%d\r\n", temp);
			printf("dis:%d\r\n", dis);

			// 打印到OLED
			sprintf(display_buffer, "dis:%04d", dis);
			OLED_ShowString(2, 2, (uint8_t*)display_buffer, 16); 
			sprintf(display_buffer, "temp:%04d", temp);
			OLED_ShowString(2, 4, (uint8_t*)display_buffer, 16); 	
			delay_ms(300);
			
			// 清除显示
			OLED_ShowString(2, 0, (uint8_t*)"     ", 16);
			motor_start();
			delay_ms(300);

			// 使能中断
			enable_limit_interrupt();
			
			// 恢复电机运动
			motor_is_runing = 1;
			
		}
		if(temp > 500)// 某温度超过预设阈值
		{
			LED_MODE = 2;
		}
		else if(temp <= 500)
		{
			// 温度恢复后恢复LED模式
			if(LED_MODE == 2)
				LED_MODE = 1;
		}
		if(dis < 40  && !motor_stopped_by_obstacle)// 距离传感器识别障碍物停车
		{
			motor_stop();
			// 重新设定目标,达不到
			motor_set_pulse_count(10000000);
			LED_MODE = 3;
			motor_stopped_by_obstacle = 1;
			obstacle_clear_count = 0;  // 清除滤波计数
			// 停止电机运动
			motor_is_runing = 0;
		}
		// 连续3次测量距离都大于40认为障碍物清除
		if(motor_stopped_by_obstacle)
		{
			dis = get_distance();
			sprintf(display_buffer, "dis:%04d", dis);
			OLED_ShowString(2, 2, (uint8_t*)display_buffer, 16); 
			if(dis > 40)
			{
				obstacle_clear_count++;
					
				// 连续3次都大于40清除障碍标志
				if(obstacle_clear_count >= 3)
				{
					motor_start();
					LED_MODE = 1;
					motor_stopped_by_obstacle = 0;
					obstacle_clear_count = 0;
					// 恢复电机运动
					motor_is_runing = 1;
				}
			}
			else
			{
				// 只要有一次距离<=40就清除计数
				if(obstacle_clear_count > 0)
				{
					obstacle_clear_count = 0;
				}
			}
		}
		
		// 改变LED显示
		switch(LED_MODE)
		{
			case 0:  // 关闭
				LED_OFF();
				break;
				
			case 1:  // 常亮
				LED_ON();
				break;
				
			case 2:  // 闪烁--某温度超过预设阈值
				if(sysTick_ms % 1000 < 500)
					LED_ON();
				else
					LED_OFF();
				break;
				
			case 3:  // 闪烁--距离传感器识别障碍物停车
				if(sysTick_ms % 200 < 100)
					LED_ON();
				else
					LED_OFF();
				break;
		}
		
		// 控制电机运动
		if (motor_is_runing == 0 )
		{
			motor_stop();
		}
		else
		{
			GPIO_ResetBits(GPIOB,GPIO_Pin_11);
			motor_start();
		}
	}
}

void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        sysTick_ms++;  // 1ms加1
		// 5s循环
		if(sysTick_ms>5000)
		{
			sysTick_ms = 0;
		}
    }
}

