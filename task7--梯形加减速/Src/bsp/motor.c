#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "string.h"
#include "bsp.h"
#include "FreeRTOS_demo.h"
#include "motor.h"

static motor_t g_motor;						// 电机全局状态

// 电机目标到达信号量(ISR中give, vTaskMotorReached中take)
SemaphoreHandle_t motor_target_reached_semaphore;


// 整数平方根 (Newton-Raphson), 避免使用 math.h
static uint32_t usqrt(uint32_t x)
{
	if (x == 0) return 0;
	uint32_t r = x;
	uint32_t c = (x + 1) / 2;
	while (c < r)
	{
		r = c;
		c = (r + x / r) / 2;
	}
	return r;
}

//计算从起始速度到目标速度所需的实际步数
static uint32_t prvSimulateAccelSteps(uint32_t c0, uint32_t target_pulse, uint32_t max_steps)
{
	uint32_t PULSE = c0;                           
	int32_t rest = 0;                           
	uint32_t n;

	for (n = 1; n <= max_steps; n++) {
		uint32_t delta = ((2 * PULSE) + (uint32_t)rest) / (4 * n + 1);
		rest = (int32_t)(((2 * PULSE) + (uint32_t)rest) % (4 * n + 1));
		PULSE = PULSE - delta;
		if (PULSE <= target_pulse)
			return n;
	}

	return max_steps;
}

// 钳制频率大小, 低于 MOTOR_MIN_HZ 或高于 MOTOR_MAX_HZ 会被限制
static uint32_t prvClampHz(uint32_t hz)
{
	if (hz < MOTOR_MIN_HZ)
	{
		return MOTOR_MIN_HZ;
	}
	if (hz > MOTOR_MAX_HZ)
	{
		return MOTOR_MAX_HZ;
	}
	return hz;
}



// 设置首次CCR1匹配点: 读取当前CNT, 计算第一次翻转时机
// 仅在MotorStart()中调用, 用于启动脉冲输出
static void prvSetFirstCcr(uint32_t hz)
{
	uint16_t cnt = TIM_GetCounter(MOTOR_TIM);
	TIM_SetCompare1(MOTOR_TIM, cnt + g_motor.PULSE);
}


/* ---------- 电机初始化  PA0-->PUL   PE12-->DIR   PB11-->ENA(低电平有效)  ---------- */
void MotorInit(void)
{
	// 初始化电机结构体, 设定默认值
	memset(&g_motor, 0, sizeof(g_motor));
	g_motor.total_steps    = MOTOR_UNIT_TO_PULSES(MOTOR_DEFAULT_TARGET_UNIT);
	g_motor.PULSE          = MOTOR_CALC_CCR_INTERVAL(MOTOR_DEFAULT_SPEED_HZ);
	g_motor.min_pulse      = MOTOR_CALC_CCR_INTERVAL(MOTOR_MAX_HZ);
	g_motor.user_speed_hz  = MOTOR_DEFAULT_SPEED_HZ;
	g_motor.run_state      = MOTOR_STOP;

	// 使能 GPIOA, GPIOB, GPIOE, TIM5 时钟
	RCC_APB2PeriphClockCmd(MOTOR_PUL_CLK, ENABLE);		// PA0  -> TIM5_CH1 复用输出
	RCC_APB2PeriphClockCmd(MOTOR_ENA_CLK, ENABLE);		// PB11 -> ENA 使能
	RCC_APB2PeriphClockCmd(MOTOR_DIR_CLK, ENABLE);		// PE12 -> DIR 方向
	RCC_APB1PeriphClockCmd(MOTOR_TIM_RCC,  ENABLE);

	// 配置控制引脚 PB11(ENA) PE12(DIR)
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = MOTOR_GPIO_MODE;
	GPIO_InitStructure.GPIO_Pin = MOTOR_ENA_PIN;
	GPIO_InitStructure.GPIO_Speed = MOTOR_GPIO_SPEED;
	GPIO_Init(MOTOR_ENA_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = MOTOR_GPIO_MODE;
	GPIO_InitStructure.GPIO_Pin = MOTOR_DIR_PIN;
	GPIO_InitStructure.GPIO_Speed = MOTOR_GPIO_SPEED;
	GPIO_Init(MOTOR_DIR_PORT, &GPIO_InitStructure);

	GPIO_ResetBits(MOTOR_ENA_PORT, MOTOR_ENA_PIN);		// ENA低电平 -> 使能电机驱动
	GPIO_SetBits(MOTOR_DIR_PORT,  MOTOR_DIR_PIN);		// DIR高电平 -> 默认正方向

	// TIM5 时基: 72MHz/PSC=8=9MHz, ARR=0xFFFF (固定最大值, CNT自由运行)
	TIM_InternalClockConfig(MOTOR_TIM);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = MOTOR_TIM_CKD;
	TIM_TimeBaseInitStructure.TIM_CounterMode = MOTOR_TIM_COUNTER_MODE;
	TIM_TimeBaseInitStructure.TIM_Period = MOTOR_TIM_ARR;				// ARR=0xFFFF
	TIM_TimeBaseInitStructure.TIM_Prescaler = MOTOR_TIM_PRESCALER - 1;	// PSC=8
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(MOTOR_TIM, &TIM_TimeBaseInitStructure);

	// 配置 TIM5_CH1 输出比较翻转模式
	// ARR=0xFFFF固定, CCR在ISR中动态更新以控制脉冲间隔
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = MOTOR_TIM_OC_MODE;
	TIM_OCInitStructure.TIM_OCPolarity = MOTOR_TIM_OC_POLARITY;
	TIM_OCInitStructure.TIM_OutputState = MOTOR_TIM_OC_STATE;
	TIM_OCInitStructure.TIM_Pulse = 0;									// 初始CCR=0, MotorStart()设置
	TIM_OC1Init(MOTOR_TIM, &TIM_OCInitStructure);

	// 配置 PA0 为复用推挽输出 -> TIM5_CH1(PUL)
	GPIO_InitStructure.GPIO_Mode = MOTOR_PUL_GPIO_MODE;
	GPIO_InitStructure.GPIO_Pin = MOTOR_PUL_PIN;
	GPIO_InitStructure.GPIO_Speed = MOTOR_GPIO_SPEED;
	GPIO_Init(MOTOR_PUL_PORT, &GPIO_InitStructure);

	// 配置 TIM5 CC1 中断优先级 (NVIC group 2, 在 FreeRTOS_demo.c 中配置)
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = MOTOR_TIM_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = MOTOR_TIM_PREEMPT_PRIO;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = MOTOR_TIM_SUB_PRIO;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// TIM5使能, CNT开始自由运行; CC1中断暂不使能(MotorStart()中开启)
	TIM_Cmd(MOTOR_TIM, ENABLE);
	
	//初始化电机参数
	g_motor.accel = 1600;
	g_motor.decel = 1600;		// 加减速的加速度为 1圈 / s
}



// 计算梯形加减速参数 (内部函数)
// step: 总步数(脉冲), accel/decel: 脉冲/s², speed: Hz
// accel/decel/speed 为0时默认 1600
static void MotorSetConstant(uint32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{
	if (speed == 0) speed = 1600;
	if (accel == 0) accel = 1600;
	if (decel == 0) decel = 1600;

	// c₀: 初始脉冲间隔(完整周期) = T1_FREQ · √(A_SQ / accel) / 100
	uint32_t c0_full = (uint32_t)(T1_FREQ * usqrt((uint32_t)(A_SQ / accel)) / 100);
	uint32_t c0 = c0_full / 2;  // OC翻转: 半周期

	// target_pulse: 目标速度对应半周期(提前计算, 供模拟函数使用)
	uint32_t target_pulse = MOTOR_CALC_CCR_INTERVAL(speed);

	// max_s_lim: 连续域近似(仅作模拟上限), 实际步数由离散模拟决定
	uint32_t max_s_lim = (uint32_t)((uint64_t)speed * speed / (2 * ALPHA * accel * 100));

	// sim_accel_limit: 模拟离散加速公式的真实收敛步数
	uint32_t sim_accel_limit = prvSimulateAccelSteps(c0, target_pulse, max_s_lim);

	// accel_lim: 减速开始前步数 = step·decel / (accel+decel)
	uint32_t accel_lim = (uint32_t)((uint64_t)step * decel / (accel + decel));

	// decel_val: 减速步数
	uint32_t decel_val;
	if (sim_accel_limit < accel_lim)
	{
		// 梯形: decel_val = sim_accel_limit · accel / decel + 1 (补偿 off-by-one)
		decel_val = (uint32_t)((uint64_t)sim_accel_limit * accel / decel) + 1;
	}
	else
	{
		// 三角形: decel_val = step - accel_lim
		decel_val = step - accel_lim;
	}
	if (decel_val >= step) decel_val = step / 2;
	if (decel_val == 0) decel_val = 1;

	// 设置梯形参数
	g_motor.PULSE        = c0;
	g_motor.target_pulse = target_pulse;
	g_motor.accel_count  = 0;
	g_motor.accel_limit  = sim_accel_limit;  // 加速阶段总步数(离散模拟值)
	g_motor.decel_start  = step - decel_val;
	g_motor.decel_val    = decel_val;
	g_motor.total_steps  = step;
	g_motor.step_count   = 0;
	g_motor.accel        = accel;
	g_motor.decel        = decel;
	g_motor.user_speed_hz = speed;
}

// 创建电机目标到达信号量(在vFreeRtosStart中调用)
void vMotorTargetSemaphoreCreate(void)
{
	motor_target_reached_semaphore = xSemaphoreCreateBinary();
	configASSERT(motor_target_reached_semaphore != NULL);
}


/* ---------- 方向控制 ---------- */

// 切换电机方向, 带加减速保护
// 流程: 先停机 -> 等待完全停止 -> 切换方向引脚 -> 重新启动
void MotorSetDir(uint8_t dir)
{
	uint8_t current_dir = GPIO_ReadOutputDataBit(MOTOR_DIR_PORT, MOTOR_DIR_PIN);

	if (current_dir == dir)
	{
		return;
	}

	MotorStop();

	while (g_motor.run_state != MOTOR_STOP)
	{
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	GPIO_WriteBit(MOTOR_DIR_PORT, MOTOR_DIR_PIN, (BitAction)dir);
	MotorStart();
}

// 直接反转方向, 不停机不加减速
void MotorChangeDir(void)
{
	uint8_t current_dir = GPIO_ReadOutputDataBit(MOTOR_DIR_PORT, MOTOR_DIR_PIN);

	if (current_dir == MOTOR_DIR_REVERSE)
	{
		GPIO_SetBits(MOTOR_DIR_PORT, MOTOR_DIR_PIN);
	}
	else
	{
		GPIO_ResetBits(MOTOR_DIR_PORT, MOTOR_DIR_PIN);
	}
}


/* ---------- 速度控制 ---------- */

// 设定电机目标速度
// 根据电机状态选择不同路径:
//   run_state!=MOTOR_STOP (运行中, ISR使用): 直接更新速度
//   run_state==MOTOR_STOP (未启动/已停机):  直接修改变量, start时生效
// 参数: speed - 0.01圈/秒
void MotorSetSpeed(uint32_t speed)
{
	uint32_t hz = MOTOR_SPEED_TO_HZ(speed);
	g_motor.user_speed_hz = hz;
	g_motor.target_pulse = MOTOR_CALC_CCR_INTERVAL(hz);
	g_motor.PULSE = g_motor.target_pulse;
}

// 停机: 直接停止TIM5输出
void MotorStop(void)
{
	g_motor.run_state = MOTOR_STOP;
	TIM_ITConfig(MOTOR_TIM, MOTOR_TIM_IT_CC, DISABLE);
	TIM_Cmd(MOTOR_TIM, DISABLE);
}

// 启动电机
void MotorStart(void)
{
	if (g_motor.run_state != MOTOR_STOP)
	{
		return;
	}

	g_motor.run_state   = MOTOR_ACCEL;

	prvSetFirstCcr(g_motor.user_speed_hz);							// 设置首次CCR1匹配点
	TIM_Cmd(MOTOR_TIM, ENABLE);
	TIM_ITConfig(MOTOR_TIM, MOTOR_TIM_IT_CC, ENABLE);			// 使能CC1中断, ISR开始工作
}


/* ---------- 距离计数API (对外单位: 0.01圈) ---------- */

// 重置距离计数器
void MotorResetUnitCount(void)
{
	g_motor.step_count = 0;
	g_motor.accel_count = 0;
}

// 获取当前已走距离, 返回0.01圈
uint32_t MotorGetUnitCount(void)
{
	return MOTOR_PULSES_TO_UNIT(g_motor.step_count);
}

// 设定目标距离, 参数: 0.01圈, 同时自动清除 target_reached 标志
void MotorSetTargetUnit(uint32_t unit)
{
	uint32_t pulses = MOTOR_UNIT_TO_PULSES(unit);
	g_motor.target_reached = 0;
	MotorSetConstant(pulses, g_motor.accel, g_motor.decel, g_motor.user_speed_hz);
}

// 查询是否到达目标脉冲数, 读取后自动清零
uint8_t MotorIsTargetReached(void)
{
	uint8_t temp = g_motor.target_reached;

	if (temp)
	{
		g_motor.target_reached = 0;
	}
	return temp;
}

// 设定加速度, 参数: 0.01圈/s²
void MotorSetAccel(uint32_t accel)
{
	g_motor.accel = MOTOR_ACCEL_TO_HZ2(accel);
}

// 设定减速度, 参数: 0.01圈/s²
void MotorSetDecel(uint32_t decel)
{
	g_motor.decel = MOTOR_ACCEL_TO_HZ2(decel);
}

uint32_t acccount = 0;
uint32_t runcount = 0;
uint32_t deccount = 0;
/* ---------- TIM5 CC1中断服务函数: 脉冲计数 + 梯形加减速 ---------- */
/* 每次CC1匹配(OC硬件自动翻转电平)触发中断, ISR中动态更新CCR控制下一次翻转 */
void MOTOR_TIM_IRQ(void)
{
	static uint32_t cnt;
	static uint8_t i = 0;
	static uint32_t next_step_pulse;
	static int32_t rest = 0;

	if (TIM_GetITStatus(MOTOR_TIM, MOTOR_TIM_IT_CC) != RESET)
	{
		TIM_ClearITPendingBit(MOTOR_TIM, MOTOR_TIM_IT_CC);

		cnt = MOTOR_TIM->CNT;
		i++;

		if (i == 2) // 一个脉冲(2次翻转)
		{
			i = 0;
			
			switch (g_motor.run_state)
			{
			case MOTOR_STOP:
				g_motor.step_count = 0;	// 清零步数
				rest = 0;        // 清零余数
				MotorStop();
				break;

			case MOTOR_ACCEL:
				acccount++;
				// 加速阶段
				g_motor.step_count++;	// 步数加1
				g_motor.accel_count++;	// 公式计数器加1
				next_step_pulse = g_motor.PULSE - (((2 *g_motor.PULSE) + rest)/(4 * g_motor.accel_count + 1));		//计算新(下)一步脉冲周期(时间间隔)
				rest = ((2 * g_motor.PULSE)+rest)%(4 * g_motor.accel_count + 1);	// 计算余数
				/* 状态切换 */
				if(g_motor.step_count >= g_motor.decel_start)
				{
					// 直接减速(三角形), 使用预计算的固定减速步数
					g_motor.run_state = MOTOR_DECEL;
				}
				else if(next_step_pulse <= g_motor.target_pulse || g_motor.step_count >= g_motor.accel_limit)
				{
					// 加速完成: PULSE已到目标 或 达加速度极限
					if(next_step_pulse < g_motor.target_pulse)
						next_step_pulse = g_motor.target_pulse;
					rest = 0;
					g_motor.run_state = MOTOR_RUN;
				}
				break;

			case MOTOR_RUN:
				runcount++;
				// 匀速阶段
				g_motor.step_count++;

				/* 状态切换 */
				if(g_motor.step_count >= g_motor.decel_start)
				{
					// 开始减速, 使用预计算的固定减速步数
					g_motor.run_state = MOTOR_DECEL;
				}
				break;

			case MOTOR_DECEL:
				deccount++;
				// 减速阶段
				g_motor.step_count++;
				g_motor.decel_val--;

				next_step_pulse = g_motor.PULSE + (((2 * g_motor.PULSE) + rest)/(4 * g_motor.decel_val - 1));
				rest = ((2 * g_motor.PULSE) + rest)%(4 * g_motor.decel_val - 1);

				if(g_motor.decel_val <= 1)
				{
					next_step_pulse = g_motor.PULSE;
					g_motor.run_state = MOTOR_STOP;
					g_motor.target_reached = 1;
//					printf("%d\r\n",acccount);
//					printf("%d\r\n",deccount);
//					printf("%d\r\n",runcount);
				}
				break;
				}

			// 更新脉冲周期
			g_motor.PULSE = next_step_pulse;
		}

		// 计算下一个CCR1匹配点
		MOTOR_TIM->CCR1 = 0xFFFF & (cnt + g_motor.PULSE);
	}

	// portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
