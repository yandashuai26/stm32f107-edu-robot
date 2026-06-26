#include <stdio.h>
#include <string.h>
#include "FreeRTOS_demo.h"
#include "bsp.h"

extern volatile uint8_t limit_pending = 0;  // 限位开关中断挂起标志

// 异常情况信号量, vTaskCollect的give, vTaskAbnormalSituation的take
SemaphoreHandle_t abnormal_situation_semaphore;

// 消息队列
QueueHandle_t temp_queue;
QueueHandle_t dis_queue;
QueueHandle_t abnormal_situation_dis_queue;
QueueHandle_t led_mode_queue; // 0-->关闭 1-->常亮 2-->慢闪 3-->快闪

// USART3接收缓冲区
extern volatile uint8_t  usart3_rx_buffer[];
extern volatile uint8_t  usart3_rx_finished;
extern volatile uint16_t usart3_rx_count;

// 任务函数声明
void vTaskLimitSwitch(void *pvParameters);
void vTaskControl(void *pvParameters);
void vTaskCollect(void *pvParameters);
void vTaskCommunication(void *pvParameters);
void vTaskAbnormalSituation(void *pvParameters);
void vTaskLedMode(void *pvParameters);
void vTaskMotorReached(void *pvParameters);

void vFreeRtosStart(void)
{
	// 创建限位开关信号量, 必须在创建任务前创建
	LimitSemaphoreCreate();
	Usart3SemaphoreCreate();
	vMotorTargetSemaphoreCreate();

	// 创建异常情况信号量
	abnormal_situation_semaphore = xSemaphoreCreateBinary();
	configASSERT(abnormal_situation_semaphore != NULL);

	// 创建消息队列, 20个单元, 每个uint32_t
	temp_queue = xQueueCreate(20, sizeof(uint32_t));
	dis_queue = xQueueCreate(20, sizeof(uint32_t));
	abnormal_situation_dis_queue = xQueueCreate(20, sizeof(uint32_t));
	led_mode_queue = xQueueCreate(20, sizeof(uint32_t));
	configASSERT(temp_queue != NULL);
	configASSERT(dis_queue != NULL);
	configASSERT(abnormal_situation_dis_queue != NULL);
	configASSERT(led_mode_queue != NULL);

	// 发送初始数据到LED模式队列(初始化)
	BaseType_t ret;
	uint8_t LedMode = 1;
	ret = xQueueSend(led_mode_queue, &LedMode, 0);

	// 创建任务: 限位开关 > 异常 > 控制 > 采集 > 通信
//	xTaskCreate(vTaskLimitSwitch,  "vTaskLimitSwitch",  256, NULL, 4, NULL);
//	xTaskCreate(vTaskAbnormalSituation,  "vTaskAbnormalSituation",  256, NULL, 4, NULL);
//	xTaskCreate(vTaskControl,      "vTaskControl",      256, NULL, 3, NULL);
//	xTaskCreate(vTaskCollect,      "vTaskCollect",      256, NULL, 2, NULL);
//	xTaskCreate(vTaskMotorReached,  "vTaskMotorReached",    256, NULL, 2, NULL);
//	xTaskCreate(vTaskCommunication,"vTaskCommunication", 256, NULL, 1, NULL);
//	xTaskCreate(vTaskLedMode,            "vTaskLedMode",            256, NULL, 1, NULL);

	// 启动调度器
	vTaskStartScheduler();
}

// 限位开关任务实现, 优先级4
void vTaskLimitSwitch(void *pvParameters)
{
	while(1)
	{
		// 等待限位开关信号量(由EXTI中断释放)
		if(xSemaphoreTake(limit_semaphore, portMAX_DELAY) == pdTRUE)
		{
			OledShowString(2, 6, (uint8_t*)"Limit", 16);
			// 50ms消抖延时, 防止一次限位触发两次EXTI
			vTaskDelay(pdMS_TO_TICKS(50));

			// 从ISR记录中获取触发引脚, 避免重新读取GPIO导致沿触发丢失
			uint8_t trigger_pin = GetLimitTriggerPin();

			if(trigger_pin == 13)
			{
				// PC13限位触发, 反转方向, 回中点
				MotorResetUnitCount();
				MotorChangeDir();
				MotorSetTargetUnit(MOTOR_MIDPOINT_UNIT);
				SetLimitReachedFlag();

				// 进入冷却300ms, 防止ISR中重复释放信号量
				LimitEnterCooldown();
				vTaskDelay(pdMS_TO_TICKS(300));
				LimitExitCooldown();
			}
			else if(trigger_pin == 14)
			{
				// PC14限位触发, 反转方向, 回中点
				MotorResetUnitCount();
				MotorChangeDir();
				MotorSetTargetUnit(MOTOR_MIDPOINT_UNIT);
				SetLimitReachedFlag();

				// 进入冷却300ms, 防止ISR中重复释放信号量
				LimitEnterCooldown();
				vTaskDelay(pdMS_TO_TICKS(300));
				LimitExitCooldown();
			}
			vTaskDelay(pdMS_TO_TICKS(1000));
			// 清除显示
			OledShowString(2, 6, (uint8_t*)"          ", 16);
		}
	}
}

// 异常情况处理任务实现, 优先级4
void vTaskAbnormalSituation(void *pvParameters)
{
	BaseType_t ret;
	uint8_t LedMode;
	uint32_t dis = 0;
	uint8_t motor_stopped_by_obstacle = 0;  // 0=没有因为障碍物停止 1=因为障碍物停止
	uint8_t obstacle_clear_count = 0;       // 障碍物清除计数器, 连续3次读取>10cm
	char display_buffer[20];  // 显示字符串缓冲区
	while(1)
	{
		// 等待异常情况信号量(由vTaskCollect释放)
		if(xSemaphoreTake(abnormal_situation_semaphore, portMAX_DELAY) == pdTRUE)
		{
			if(xQueueReceive(abnormal_situation_dis_queue, &dis, 0) == pdTRUE)  // 非阻塞接收
			{
				sprintf(display_buffer, "dis:%04d", dis);
				OledShowString(2, 4, (uint8_t*)display_buffer, 16);
				if(dis < 50)  // 检测到障碍物
				{
					obstacle_clear_count = 0;
					LedMode = 3;
					ret = xQueueSend(led_mode_queue, &LedMode, 0);
					if(!motor_stopped_by_obstacle)
					{
						MotorStop();
						motor_stopped_by_obstacle = 1;
					}
				}
				else  // 无障碍物
				{
					if(motor_stopped_by_obstacle)
					{
						obstacle_clear_count++;
						if(obstacle_clear_count >= 3)
						{
							MotorStart();
							LedMode = 1;
							ret = xQueueSend(led_mode_queue, &LedMode, 0);
							motor_stopped_by_obstacle = 0;
							obstacle_clear_count = 0;
						}
					}
				}
				vTaskDelay(300);  // 让出CPU
			}
		}
	}
}

// 控制任务实现, 优先级3
void vTaskControl(void *pvParameters)
{
	char cmd_buffer[256];

	while(1)
	{
		// 等待USART3接收信号量(由USART3 ISR释放)
		if(xSemaphoreTake(usart3_rx_semaphore, portMAX_DELAY) == pdTRUE)
		{
			taskENTER_CRITICAL();
			uint16_t rx_count = usart3_rx_count;
			memcpy(cmd_buffer, (char*)usart3_rx_buffer, rx_count);
			cmd_buffer[rx_count] = '\0';

			// 去除回车换行
			for(int i = 0; i < rx_count; i++)
			{
				if(cmd_buffer[i] == '\r' || cmd_buffer[i] == '\n')
				{
					cmd_buffer[i] = '\0';
					break;
				}
			}

			// 清除状态标志和缓冲区
			usart3_rx_finished = 0;
			memset((void*)usart3_rx_buffer, 0, rx_count);
			taskEXIT_CRITICAL();

			// 执行对应指令
			if(rx_count > 0)
			{
				// 收到STOP
				if(strcmp(cmd_buffer, "STOP") == 0)
				{
					MotorStop();
					printf("STOP\r\n");
				}
				// 收到START
				else if(strcmp(cmd_buffer, "START") == 0)
				{
					MotorStart();
					printf("START\r\n");
				}
			}
		}
	}
}

// 采集任务实现, 优先级2
void vTaskCollect(void *pvParameters)
{
	uint32_t temp = 0;
	uint32_t dis = 0;
	BaseType_t ret;

	while(1)
	{
		// 获取温度
		temp = GetTempture();
		dis = GetDistance();

		// 通知异常情况处理任务
		xSemaphoreGive(abnormal_situation_semaphore);
		ret = xQueueSend(abnormal_situation_dis_queue, &dis, 0);

		// 发送温度
		ret = xQueueSend(temp_queue, &temp, 0);
		if(ret != pdTRUE)
		{
			printf("TempQueue is full!\r\n");
		}

		ret = xQueueSend(dis_queue, &dis, 0);
		if(ret != pdTRUE)
		{
			printf("DisQueue is full!\r\n");
		}

		// 每100ms采集一次
		vTaskDelay(100);
	}
}

// 电机到位任务实现, 优先级2
void vTaskMotorReached(void *pvParameters)
{
	while(1)
	{
		// 等待电机目标到达信号量(由TIM5 ISR释放)
		if(xSemaphoreTake(motor_target_reached_semaphore, portMAX_DELAY) == pdTRUE)
		{
			MotorStop();
			OledShowString(2, 0, (uint8_t*)"Reached", 16);
			// 重置距离计数, 设置目标为默认值
			MotorResetUnitCount();
			MotorSetTargetUnit(MOTOR_DEFAULT_TARGET_UNIT);

			vTaskDelay(1000);
			MotorStart();
			// 清除显示
			OledShowString(2, 0, (uint8_t*)"        ", 16);
		}
		vTaskDelay(100);
	}
}

// 通信任务实现, 优先级1
void vTaskCommunication(void *pvParameters)
{
	char display_buffer[20];
	uint32_t temp = 0;
	uint32_t dis = 0;

	while(1)
	{
		// 从队列接收, 在串口和OLED显示
		if(xQueueReceive(temp_queue, &temp, 0) == pdTRUE)  // 非阻塞接收
		{
			printf("temp:%d\r\n", temp);
			sprintf(display_buffer, "temp:%04d", temp);
			OledShowString(2, 2, (uint8_t*)display_buffer, 16);
		}

		if(xQueueReceive(dis_queue, &dis, 0) == pdTRUE)  // 非阻塞接收
		{
			printf("dis:%d\r\n", dis);
			sprintf(display_buffer, "dis:%04d", dis);
			OledShowString(2, 4, (uint8_t*)display_buffer, 16);
		}

		// 让出CPU
		vTaskDelay(200);
	}
}

// LED模式任务实现, 优先级1
void vTaskLedMode(void *pvParameters)
{
	uint8_t LedMode;
	while(1)
	{
		xQueueReceive(led_mode_queue, &LedMode, 0);
		switch(LedMode)
		{
		case 0:  // 关闭
			LedOff();
			vTaskDelay(1000); // 让出CPU
			break;

		case 1:  // 常亮
			LedOn();
			vTaskDelay(1000); // 让出CPU
			break;

		case 2:  // 慢闪
			LedOn();
			vTaskDelay(700);
			LedOff();
			vTaskDelay(700);
			break;

		case 3:  // 快闪
			LedOn();
			vTaskDelay(100);
			LedOff();
			vTaskDelay(100);
			break;
		}
	}
}
