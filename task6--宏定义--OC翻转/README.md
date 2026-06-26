# 任务6 — 宏定义 OC 翻转脉冲版

基于 STM32F107VCT6 的教育机器人控制系统，在任务5 FreeRTOS 7任务架构基础上将所有电机控制参数重构为宏定义，采用 TIM5 OC（Output Compare）翻转模式输出脉冲。步进电机参数（细分、预分频、脉冲/圈）和速度参数均可通过头文件宏定义统一配置。

## 版本概述

| 项目 | 说明 |
|------|------|
| **架构** | FreeRTOS 实时多任务 (7 任务) |
| **电机速度** | 1600 Hz (1 圈/秒，8细分×200步) |
| **状态管理** | 结构体 (`motor_t`) |
| **IPC 机制** | 二进制信号量 + 消息队列 |
| **脉冲输出** | TIM5 OC 翻转模式 (CC1 中断动态更新 CCR) |
| **障碍物检测** | 超声波测距 < 50mm 触发停机，连续 3 次 > 50mm 自动恢复 |
| **LED 模式** | 4 种模式（关闭/常亮/慢闪/快闪），独立任务控制 |
| **编码** | GBK |

## 与任务5的主要区别

| 对比项 | 任务5 (7任务) | 任务6 (宏定义OC翻转) |
|--------|-------------|---------------------|
| 电机参数 | 硬编码数值 | **全部宏定义** (`MOTOR_TIM`, `MOTOR_PULSES_PER_REV` 等) |
| 脉冲输出方式 | PWM 模式 | **TIM5 OC 翻转模式** (ARR=0xFFFF 固定, CC1 中断更新 CCR) |
| 电机速度 | 1000 pps | **1600 Hz** (1圈/秒 = 200步×8细分) |
| 中点回退 | 3050 脉冲 | **2800 脉冲** (1.75圈) |
| 函数命名风格 | `motor_set_speed()` 小写下划线 | 相同 (`motor_set_speed()`) |
| 结构体字段 | `target_pulses`, `current_hz` 等 | 相同 |

## 宏定义参数一览

### 系统时钟与定时器

| 宏定义 | 值 | 说明 |
|--------|-----|------|
| `MOTOR_TIM` | TIM5 | 电机脉冲定时器 |
| `MOTOR_TIM_CLK_HZ` | 72000000 | 系统时钟 72MHz |
| `MOTOR_TIM_PRESCALER` | 8 | 预分频, 72M/8 = 9MHz |
| `MOTOR_TIM_BASE_HZ` | 9000000 | 定时器基准频率 |

### 步进电机参数

| 宏定义 | 值 | 说明 |
|--------|-----|------|
| `MOTOR_FULL_STEPS_PER_REV` | 200 | 全步/圈 |
| `MOTOR_MICROSTEP` | 8 | 细分数 |
| `MOTOR_PULSES_PER_REV` | 1600 | 脉冲/圈 (200×8) |

### 速度参数

| 宏定义 | 值 | 说明 |
|--------|-----|------|
| `MOTOR_MIN_RPS` | 0.5 | 最低转速 (圈/秒) |
| `MOTOR_MAX_RPS` | 2.0 | 最高转速 (圈/秒) |
| `MOTOR_SPEED_1RPS_HZ` | 1600 | 1圈/秒对应的脉冲频率 |
| `MOTOR_DEFAULT_SPEED_HZ` | 1600 | 默认速度 (1圈/秒) |
| `MOTOR_MIN_HZ` | 800 | 最低脉冲频率 |
| `MOTOR_MAX_HZ` | 3200 | 最高脉冲频率 |
| `MOTOR_RAMP_STEP_HZ` | 10 | 加减速步进 (Hz/脉冲) |

### 距离参数

| 宏定义 | 值 | 说明 |
|--------|-----|------|
| `MOTOR_DEFAULT_TARGET_PULSES` | 10000000 | 默认目标脉冲 (极大值) |
| `MOTOR_MIDPOINT_PULSES` | 2800 | 中点回退脉冲 (1.75圈) |

### OC 翻转参数

| 宏定义 | 值 | 说明 |
|--------|-----|------|
| `MOTOR_TIM_OC_MODE` | TIM_OCMode_Toggle | OC 翻转模式 |
| `MOTOR_TIM_ARR` | 0xFFFF | ARR 固定最大值 |
| `MOTOR_CALC_CCR_INTERVAL(hz)` | 9M / 2 / hz | CCR 间隔计算 (OC 翻转 2 次 = 1 脉冲) |

## 任务架构

```
优先级 4:  vTaskLimitSwitch       限位开关处理 (信号量触发)
优先级 4:  vTaskAbnormalSituation  异常情况处理 (障碍物检测与恢复)
优先级 3:  vTaskControl            串口指令解析 (信号量阻塞)
优先级 2:  vTaskCollect            传感器数据采集 (温度 + 距离)
优先级 2:  vTaskMotorReached       中点到达检测 (信号量触发)
优先级 1:  vTaskCommunication      数据显示 (OLED + 串口)
优先级 1:  vTaskLedMode            LED 模式控制
```

### IPC 数据流

```
EXTI 中断 ──(信号量)──> vTaskLimitSwitch ──> motor_change_dir()
                                          ──> motor_reset_pulse_count()
                                          ──> motor_set_pulse_count(2800)

USART3 IDLE中断 ──(信号量)──> vTaskControl ──> STOP/START 指令处理

TIM5 CC1中断 ──(信号量)──> vTaskMotorReached ──> motor_set_pulse_count(10000000)

vTaskCollect ──(信号量)──> vTaskAbnormalSituation
              ──(队列)──> AbnormalSituationDisQueue  ──> 距离数据
              ──(队列)──> TempQueue                  ──> 温度数据
              ──(队列)──> DisQueue                   ──> 距离数据

vTaskCollect ──(队列)──> vTaskCommunication ──> OLED + printf

vTaskAbnormalSituation ──(队列)──> LedModeQueue    ──> vTaskLedMode
```

## 硬件平台

| 组件 | 规格 |
|------|------|
| **主控芯片** | STM32F107VCT6 (Cortex-M3, 72MHz) |
| **开发板** | YS-F1PRO |
| **电机驱动** | UIM240 (PWM + DIR + ENA) |
| **距离传感器** | DYP-A02 超声波 (RS485 Modbus) |
| **温度传感器** | RS485 Modbus |
| **显示屏** | 0.96" OLED SSD1306 (I2C) |
| **限位开关** | PC13 / PC14 (EXTI 下降沿) |
| **LED** | PE3 (LED3) / PE4 (LED2) |

## 引脚分配

| 引脚 | 功能 | 说明 |
|------|------|------|
| PA0 | TIM5_CH1 | 电机脉冲输出 (OC 翻转) |
| PE12 | GPIO OUT | 电机方向控制 (DIR) |
| PB11 | GPIO OUT | 电机使能 (ENA, 低有效) |
| PC13 | EXTI13 | 限位开关 1 (下降沿) |
| PC14 | EXTI14 | 限位开关 2 (下降沿) |
| PC10 | USART3 TX | printf 调试输出 |
| PC11 | USART3 RX | 串口指令接收 |
| PA9 | USART1 TX | RS485 通信 |
| PA10 | USART1 RX | RS485 通信 |
| PA12 | GPIO OUT | RS485 方向控制 |
| PE3 | GPIO OUT | LED3 |
| PE4 | GPIO OUT | LED2 |
| PB6 | I2C1 SCL | OLED 时钟 |
| PB7 | I2C1 SDA | OLED 数据 |

## 电机控制

### OC 翻转脉冲输出原理

采用 TIM5 OC (Output Compare) 翻转模式产生步进电机脉冲:
- **ARR 固定 0xFFFF**: 定时器自由运行
- **CC1 匹配中断**: 每次 CCR1 与 CNT 匹配时翻转 PA0 电平
- **脉冲频率控制**: 动态更新 CCR1 = 旧 CCR1 + 间隔值
- **OC 翻转 2 次 = 1 个完整脉冲**: 所以间隔 = 9MHz / 2 / 频率

```
PA0 输出波形 (OC翻转):
     ┌──┐  ┌──┐  ┌──┐  ┌──┐
  ───┘  └──┘  └──┘  └──┘  └──
     ←1周期→
     9MHz / 2 / hz = CCR间隔
```

### 电机状态结构体

```c
typedef struct {
    uint32_t target_pulses;    // 目标脉冲数
    uint32_t current_pulses;   // 已发脉冲数 (ISR中递增)
    uint32_t target_hz;        // 加减速目标频率
    uint32_t current_hz;       // 当前实际频率
    uint32_t user_speed_hz;    // 用户设定速度, stop/start间保持
    uint8_t  running;          // 0=停止 1=运行中
    uint8_t  speed_changing;   // 0=速度稳定 1=加减速中
    uint8_t  target_reached;   // 脉冲目标到达标志
    uint8_t  stop_pending;     // 停机请求标志
} motor_t;
```

- **加减速**: 步进 MOTOR_RAMP_STEP_HZ (10) Hz/脉冲，从 MOTOR_MIN_HZ (800) 起步
- **软停机**: 减速至 MOTOR_MIN_HZ 后自动关闭 TIM5
- **方向切换**: 先软停机 → 等待完全停止 → 切换 DIR → 软启动

## 信号量

| 信号量 | 类型 | 释放者 | 获取者 |
|--------|------|--------|--------|
| `LimitSemaphore` | 二进制 | EXTI ISR | vTaskLimitSwitch |
| `AbnormalSituationSemaphore` | 二进制 | vTaskCollect | vTaskAbnormalSituation |
| `USART3_RxSemaphore` | 二进制 | USART3 ISR | vTaskControl |
| `MotorTargetReachedSemaphore` | 二进制 | TIM5 CC1 ISR | vTaskMotorReached |

## 消息队列

| 队列名称 | 元素类型 | 容量 | 生产者 | 消费者 |
|----------|----------|------|--------|--------|
| `TempQueue` | uint32_t | 20 | vTaskCollect | vTaskCommunication |
| `DisQueue` | uint32_t | 20 | vTaskCollect | vTaskCommunication |
| `AbnormalSituationDisQueue` | uint32_t | 20 | vTaskCollect | vTaskAbnormalSituation |
| `LedModeQueue` | uint32_t | 20 | vTaskAbnormalSituation, FreeRTOS_Start | vTaskLedMode |

## 编译与烧录

### 环境要求

- **Keil MDK-ARM v5** + ARM Compiler 5 (AC5)
- **STM32F1xx_DFP v2.3.0** 设备包
- **ST-LINK V2** 调试器

### 步骤

1. 打开 `task6--宏定义--OC翻转/F107VCT6.uvprojx`
2. 按 **F7** 编译 (0 错误 0 警告)
3. 连接 ST-Link V2 (SWCLK, SWDIO, GND)
4. 按 **F8** 烧录
5. 复位开发板

### 串口监控

- 端口: USART3 (PC10 TX, PC11 RX)
- 波特率: 9600 bps
- 数据位: 8, 停止位: 1, 校验: 无
- 发送 `STOP\r\n` 停止电机，`START\r\n` 启动电机

## 模块依赖

```
main.c
├── bsp.h
│   ├── led.h       → PE3/PE4 LED 控制
│   ├── gpio.h      → PC13/PC14 限位开关 + LimitSemaphore
│   ├── motor.h     → motor_t 结构体 + TIM5 OC 翻转脉冲 + 宏定义参数
│   ├── timer.h     → TIM1 1ms 时基
│   ├── usart.h     → USART3 printf + 指令接收
│   ├── 485.h       → USART1 RS485 传感器读取
│   └── oled.h      → SSD1306 OLED 显示
└── FreeRTOS_demo.h
    ├── FreeRTOS.h
    ├── task.h
    ├── queue.h
    └── semphr.h
```

## 版本演进

本版本 (任务6) 相对于任务5 的主要变更:

| 变更 | 说明 |
|------|------|
| 宏定义参数化 | 所有电机硬件参数、速度参数、距离参数改为宏定义 |
| OC 翻转脉冲 | TIM5 从 PWM 模式改为 OC 翻转模式 (ARR=0xFFFF, CC1 中断) |
| 电机速度 | 1000 pps → 1600 Hz (1圈/秒) |
| 中点回退 | 3050 脉冲 → 2800 脉冲 (1.75圈) |
| `MOTOR_CALC_CCR_INTERVAL` | 新增 OC 翻转间隔计算宏 |

---

> **作者**: Tom
> **项目**: 毕业设计 — 教育机器人控制系统
> **最后更新**: 2026-06-23
