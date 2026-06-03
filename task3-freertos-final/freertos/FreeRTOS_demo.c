#include "FreeRTOS_demo.h"
#include "bsp.h"
#include <string.h>
#include <stdio.h>

extern volatile uint8_t limit_pending = 0;  // 限位开关中断挂起标志

// 限位开关专用信号量（vTaskCollect中give，vTaskAbnormalSituation中take）
SemaphoreHandle_t AbnormalSituationSemaphore;

// 消息队列
QueueHandle_t TempQueue;
QueueHandle_t DisQueue;
QueueHandle_t AbnormalSituationDisQueue;
QueueHandle_t LedModeQueue; // 0-->关闭 1-->亮 2-->慢闪 3-->快闪

// 串口接收缓冲区
extern volatile uint8_t  USART3_RxBuffer[];
extern volatile uint8_t  USART3_RxFinished;
extern volatile uint16_t USART3_RxCount;

// 任务函数声明
void vTaskLimitSwitch(void *pvParameters);
void vTaskControl(void *pvParameters);
void vTaskCollect(void *pvParameters);
void vTaskCommunication(void *pvParameters);
void vTaskAbnormalSituation(void *pvParameters);
void vTaskLedMode(void *pvParameters);
void vTaskMotorReached(void *pvParameters);

void FreeRTOS_Start(void)
{
    // 创建限位开关信号量（必须在任务创建前创建）
    limit_semaphore_create();
    usart3_semaphore_create();
    motor_target_semaphore_create();

    // 创建异常情况信号量
    AbnormalSituationSemaphore = xSemaphoreCreateBinary();
    configASSERT(AbnormalSituationSemaphore != NULL);

    // 创建消息队列，长度 20个单元，每个 uint32_t
    TempQueue = xQueueCreate(20, sizeof(uint32_t));
    DisQueue = xQueueCreate(20, sizeof(uint32_t));
    AbnormalSituationDisQueue = xQueueCreate(20, sizeof(uint32_t));
	LedModeQueue = xQueueCreate(20, sizeof(uint32_t));
    configASSERT(TempQueue != NULL);
    configASSERT(DisQueue != NULL);
    configASSERT(AbnormalSituationDisQueue != NULL);
	configASSERT(LedModeQueue != NULL);

    // 创建任务：限位开关 > 控制 > 采集 > 通信

	//发送初始数据到消息队列(初始化)
	BaseType_t ret;
	uint8_t LedMode = 1;
	ret = xQueueSend(LedModeQueue, &LedMode, 0);

    xTaskCreate(vTaskLimitSwitch,  "vTaskLimitSwitch",  256, NULL, 4, NULL);
    xTaskCreate(vTaskAbnormalSituation,  "vTaskAbnormalSituation",  256, NULL, 4, NULL);
    xTaskCreate(vTaskControl,      "vTaskControl",      256, NULL, 3, NULL);
    xTaskCreate(vTaskCollect,      "vTaskCollect",      256, NULL, 2, NULL);
    xTaskCreate(vTaskMotorReached,  "vTaskMotorReached",    256, NULL, 2, NULL);
    xTaskCreate(vTaskCommunication,"vTaskCommunication", 256, NULL, 1, NULL);
	xTaskCreate(vTaskLedMode,            "vTaskLedMode",            256, NULL, 1, NULL);

    // 启动调度器
    vTaskStartScheduler();
}

// 限位开关任务实现（优先级4)
void vTaskLimitSwitch(void *pvParameters)
{
    while(1)
    {
        // 阻塞等待限位开关信号量（由EXTI中断释放）
        if(xSemaphoreTake(LimitSemaphore, portMAX_DELAY) == pdTRUE)
        {
			OLED_ShowString(2, 6, (uint8_t*)"Limit", 16);
            // 50ms消抖延时（防止一次碰触触发多次EXTI）
            vTaskDelay(pdMS_TO_TICKS(50));

            // 从ISR记录中获取触发引脚（不回读GPIO，避免机械开关弹回导致丢失触发）
            uint8_t trigger_pin = get_limit_trigger_pin();

            if(trigger_pin == 13)
            {
                // PC13限位开关触发，反转电机方向并清零脉冲
                motor_reset_pulse_count();
                motor_change_dir();
                motor_set_pulse_count(3050);
                set_limit_reached_flag();

                // 进入冷却期300ms，屏蔽开关释放抖动（ISR中不给信号量）
                limit_enter_cooldown();
                vTaskDelay(pdMS_TO_TICKS(300));
                limit_exit_cooldown();
            }
            else if(trigger_pin == 14)
            {
                // PC14限位开关触发，反转电机方向并清零脉冲
                motor_reset_pulse_count();
                motor_change_dir();
				motor_set_pulse_count(3050);
                set_limit_reached_flag();

                // 进入冷却期300ms，屏蔽开关释放抖动（ISR中不给信号量）
                limit_enter_cooldown();
                vTaskDelay(pdMS_TO_TICKS(300));
                limit_exit_cooldown();
            }
			vTaskDelay(pdMS_TO_TICKS(1000));
			// 清除显示
			OLED_ShowString(2, 6, (uint8_t*)"          ", 16);
        }
    }
}

// 异常情况处理任务实现（优先级4)
void vTaskAbnormalSituation(void *pvParameters)
{
    BaseType_t ret;
    uint8_t LedMode;
    uint32_t dis = 0;
    uint8_t motor_stopped_by_obstacle = 0;  // 0表示没有因为障碍物停止  1表示因为障碍物停止
    uint8_t obstacle_clear_count = 0;       // 障碍物清除计数，连续3次都>10cm
    char display_buffer[20];  // 用于字符串显示
    while(1)
    {
        // 阻塞等待异常情况信号量（由vTaskCollect释放）
        if(xSemaphoreTake(AbnormalSituationSemaphore, portMAX_DELAY) == pdTRUE)
        {
            if(xQueueReceive(AbnormalSituationDisQueue, &dis, 0) == pdTRUE)  // 非阻塞接收
            {
                sprintf(display_buffer, "dis:%04d", dis);
                OLED_ShowString(2, 4, (uint8_t*)display_buffer, 16);
                if(dis < 50)  // 检测到障碍物
                {
                    obstacle_clear_count = 0;
				LedMode = 3;
                ret = xQueueSend(LedModeQueue, &LedMode, 0);
                    if(!motor_stopped_by_obstacle)
                    {
                        motor_stop();
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
                            motor_start();
                            LedMode = 1;
                            ret = xQueueSend(LedModeQueue, &LedMode, 0);
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

// 控制任务实现（优先级3）
void vTaskControl(void *pvParameters)
{
    char cmd_buffer[256];

    while(1)
    {
        // 阻塞等待USART3接收完成信号量（由USART3 ISR释放）
        if(xSemaphoreTake(USART3_RxSemaphore, portMAX_DELAY) == pdTRUE)
        {
            taskENTER_CRITICAL();
            uint16_t rx_count = USART3_RxCount;
            memcpy(cmd_buffer, (char*)USART3_RxBuffer, rx_count);
            cmd_buffer[rx_count] = '\0';

            // 去除换行符和回车符
            for(int i = 0; i < rx_count; i++)
            {
                if(cmd_buffer[i] == '\r' || cmd_buffer[i] == '\n')
                {
                    cmd_buffer[i] = '\0';
                    break;
                }
            }

            // 清除标志位和缓冲区
            USART3_RxFinished = 0;
            memset((void*)USART3_RxBuffer, 0, rx_count);
            taskEXIT_CRITICAL();

            // 执行对应命令
            if(rx_count > 0)
            {
                // 收到 STOP
                if(strcmp(cmd_buffer, "STOP") == 0)
                {
                    motor_stop();
                    printf("STOP\r\n");
                }
                // 收到 START
                else if(strcmp(cmd_buffer, "START") == 0)
                {
                    motor_start();
                    printf("START\r\n");
                }
            }
        }
    }
}

// 采集任务实现（优先级2）
void vTaskCollect(void *pvParameters)
{
    uint32_t temp = 0;
    uint32_t dis = 0;
    BaseType_t ret;

    while(1)
    {

        // 获取温度
        temp = get_tempture();
        dis = get_distance();

        // 始终通知异常情况监视任务
        xSemaphoreGive(AbnormalSituationSemaphore);
        ret = xQueueSend(AbnormalSituationDisQueue, &dis, 0);

        // 发送到队列
        ret = xQueueSend(TempQueue, &temp, 0);
        if(ret != pdTRUE)
        {
            printf("TempQueue is full!\r\n");
        }

        ret = xQueueSend(DisQueue, &dis, 0);
        if(ret != pdTRUE)
        {
            printf("DisQueue is full!\r\n");
        }

        // 每隔100ms采集一次
        vTaskDelay(100);
    }
}


// 电机到达检测任务实现（优先级2）
void vTaskMotorReached(void *pvParameters)
{
    while(1)
    {
        // 阻塞等待电机目标到达信号量（由TIM5 ISR释放）
        if(xSemaphoreTake(MotorTargetReachedSemaphore, portMAX_DELAY) == pdTRUE)
        {
			OLED_ShowString(2, 0, (uint8_t*)"Reached", 16);
            // 已到达中点(3050脉冲)，清除脉冲计数，重设目标为极大值继续运行
            motor_reset_pulse_count();
            motor_set_pulse_count(10000000);
			
			vTaskDelay(1000);
			// 清除显示
			OLED_ShowString(2, 0, (uint8_t*)"        ", 16);
        }
		vTaskDelay(100);
    }
}

// 通信任务实现（优先级1）
void vTaskCommunication(void *pvParameters)
{
    char display_buffer[20];
    uint32_t temp = 0;
    uint32_t dis = 0;

    while(1)
    {
        // 从队列中取数据在串口和OLED显示
        if(xQueueReceive(TempQueue, &temp, 0) == pdTRUE)  // 非阻塞接收
        {
            printf("temp:%d\r\n", temp);
            sprintf(display_buffer, "temp:%04d", temp);
            OLED_ShowString(2, 2, (uint8_t*)display_buffer, 16);
        }

        if(xQueueReceive(DisQueue, &dis, 0) == pdTRUE)  // 非阻塞接收
        {
            printf("dis:%d\r\n", dis);
            sprintf(display_buffer, "dis:%04d", dis);
            OLED_ShowString(2, 4, (uint8_t*)display_buffer, 16);
        }

        // 让出CPU
        vTaskDelay(200);
    }
}

// 改变LED状态任务实现（优先级1）
void vTaskLedMode(void *pvParameters)
{
	uint8_t LedMode;
	while(1)
	{
		xQueueReceive(LedModeQueue, &LedMode, 0);
		switch(LedMode)
		{
		case 0:  // 关闭
			LED_OFF();
			vTaskDelay(1000); // 让出CPU
			break;
			
		case 1:  // 常亮
			LED_ON();
			vTaskDelay(1000); // 让出CPU
			break;
			
		case 2:  // 慢闪
			LED_ON();
			vTaskDelay(700);
			LED_OFF();
			vTaskDelay(700);
			break;
			
		case 3:  // 快闪
			LED_ON();
			vTaskDelay(100);
			LED_OFF();
			vTaskDelay(100);
			break;
		}
	}
}