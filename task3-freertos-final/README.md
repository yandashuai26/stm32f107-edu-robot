# 任务3 — FreeRTOS 最终版

基于 STM32F107VCT6 的教育机器人控制系统最终版本，在任务2 FreeRTOS 架构基础上新增障碍物检测恢复、LED 模式控制和结构体状态管理。

## 版本概述

| 项目 | 说明 |
|------|------|
| **架构** | FreeRTOS 实时多任务 (6 任务) |
| **电机速度** | 1000 pps |
| **状态管理** | 结构体 (`motor_t`) |
| **IPC 机制** | 二进制信号量 + 消息队列 |
| **障碍物检测** | 超声波测距 < 50mm 触发停机，连续 3 次 > 50mm 自动恢复 |
| **LED 模式** | 4 种模式（关闭/常亮/慢闪/快闪），独立任务控制 |
| **编码** | GBK |

## 任务架构

```
优先级 4:  vTaskLimitSwitch       限位开关处理 (信号量触发)
优先级 4:  vTaskAbnormalSituation  异常情况处理 (障碍物检测与恢复)
优先级 3:  vTaskControl            串口指令解析 (STOP/START)
优先级 2:  vTaskCollect            传感器数据采集 (温度 + 距离)
优先级 1:  vTaskCommunication      数据显示 (OLED + 串口)
优先级 1:  vTaskLedMode            LED 模式控制
```

### IPC 数据流

```
EXTI 中断 ──(信号量)──> vTaskLimitSwitch ──> motor_change_dir()
                                          ──> motor_reset_pulse_count()

vTaskCollect ──(信号量)──> vTaskAbnormalSituation
              ──(队列)──> AbnormalSituationDisQueue  ──> 距离数据
              ──(队列)──> TempQueue                  ──> 温度数据
              ──(队列)──> DisQueue                   ──> 距离数据

vTaskCollect ──(队列)──> vTaskCommunication ──> OLED + printf

vTaskAbnormalSituation ──(队列)──> LedModeQueue    ──> vTaskLedMode
vTaskControl (START)    ──(队列)──> LedModeQueue    ──> vTaskLedMode
FreeRTOS_Start (初始化) ──(队列)──> LedModeQueue    ──> vTaskLedMode
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
| PA0 | TIM5_CH1 | 电机 PWM 脉冲输出 |
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

## 任务详细说明

### vTaskLimitSwitch (优先级 4)

- **功能**: 处理限位开关触发事件
- **触发**: `LimitSemaphore` 二进制信号量 (EXTI 中断 ISR 中释放)
- **行为**: 
  1. 等待信号量 (portMAX_DELAY)
  2. 50ms 去抖延时
  3. 读取触发引脚 (PC13 或 PC14)
  4. 清零脉冲计数 + 翻转电机方向 + 设置限位标志
  5. 进入 300ms 冷却期 (防止重复触发)

### vTaskAbnormalSituation (优先级 4)

- **功能**: 障碍物检测与自动恢复
- **触发**: `AbnormalSituationSemaphore` 二进制信号量 (vTaskCollect 每 100ms 释放)
- **状态机**:

```
正常状态 ──(dis < 50)──> 障碍物状态: motor_stop(), 设置标志
障碍物状态 ──(dis >= 50)──> 递增清除计数
        ├── 清除计数 < 3: 继续等待
        └── 清除计数 >= 3: motor_start(), 恢复 LED 模式, 回到正常状态
```

- **滞后滤波**: 连续 3 次距离 >= 50mm 才恢复，避免传感器噪声导致误恢复

### vTaskControl (优先级 3)

- **功能**: 解析串口指令
- **触发**: 轮询 (10ms 周期)，检查 USART3 接收完成标志
- **指令**:
  - `STOP` → `motor_stop()`
  - `START` → `motor_start()`

### vTaskCollect (优先级 2)

- **功能**: 传感器数据采集与分发
- **周期**: 100ms
- **数据流**:
  - 温度 → RS485 读取 → TempQueue → vTaskCommunication
  - 距离 → RS485 读取 → DisQueue → vTaskCommunication
  - 距离 → AbnormalSituationDisQueue → vTaskAbnormalSituation
  - 信号量 → AbnormalSituationSemaphore → vTaskAbnormalSituation

### vTaskCommunication (优先级 1)

- **功能**: 数据展示
- **周期**: 200ms
- **输出**:
  - OLED 第 2 行: 温度值
  - OLED 第 4 行: 距离值
  - 串口: `temp:XXX
` / `dis:XXX
`

### vTaskLedMode (优先级 1)

- **功能**: LED 状态控制
- **触发**: 从 `LedModeQueue` 接收模式指令 (非阻塞)
- **模式**:

| 模式值 | 行为 | 说明 |
|--------|------|------|
| 0 | LED_OFF, 延时 1000ms | 关闭 |
| 1 | LED_ON, 延时 1000ms | 常亮 |
| 2 | 700ms ON / 700ms OFF | 慢闪 |
| 3 | 100ms ON / 100ms OFF | 快闪 |

- **初始模式**: 1 (常亮)，由 `FreeRTOS_Start()` 初始化
- **障碍物恢复**: 自动切回模式 1

## 消息队列

| 队列名称 | 元素类型 | 容量 | 生产者 | 消费者 |
|----------|----------|------|--------|--------|
| `TempQueue` | uint32_t | 20 | vTaskCollect | vTaskCommunication |
| `DisQueue` | uint32_t | 20 | vTaskCollect | vTaskCommunication |
| `AbnormalSituationDisQueue` | uint32_t | 20 | vTaskCollect | vTaskAbnormalSituation |
| `LedModeQueue` | uint32_t | 20 | vTaskAbnormalSituation, FreeRTOS_Start | vTaskLedMode |

## 信号量

| 信号量 | 类型 | 创建位置 | 释放者 | 获取者 |
|--------|------|----------|--------|--------|
| `LimitSemaphore` | 二进制 | `gpio.c:limit_semaphore_create()` | EXTI ISR | vTaskLimitSwitch |
| `AbnormalSituationSemaphore` | 二进制 | `FreeRTOS_Start()` | vTaskCollect | vTaskAbnormalSituation |

## 中断向量分配

| 中断 | 抢占优先级 | 子优先级 | 说明 |
|------|:---:|:---:|------|
| EXTI15_10 | 0 | 0 | 限位开关 (最高优先级) |
| USART1 | 1 | 2 | RS485 通信 |
| USART3 | 1 | 2 | 串口指令 |
| TIM5 | 1 | 2 | 电机 PWM + 脉冲计数 |
| TIM1_UP | 3 | 2 | 1ms 系统时基 |
| SysTick | — | — | FreeRTOS 内核时基 |

NVIC 优先级分组: 2 (2 位抢占 + 2 位子优先级)

## 电机控制

电机模块采用结构体状态管理 (`motor_t`)，包含以下字段:

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

- **斜坡加减速**: 步进 1Hz/脉冲，从 500Hz 起步加速至目标速度
- **软停机**: 斜坡减速至 500Hz 后自动关闭 TIM5
- **方向切换**: 先软停机 → 等待完全停止 → 切换 DIR → 软启动

## 编译与烧录

### 环境要求

- **Keil MDK-ARM v5** + ARM Compiler 5 (AC5)
- **STM32F1xx_DFP v2.3.0** 设备包
- **ST-LINK V2** 调试器

### 步骤

1. 打开 `task3-freertos-final/F107VCT6.uvprojx`
2. 按 **F7** 编译 (0 错误 0 警告)
3. 连接 ST-Link V2 (SWCLK, SWDIO, GND)
4. 按 **F8** 烧录
5. 复位开发板

### 串口监控

- 端口: USART3 (PC10 TX, PC11 RX)
- 波特率: 9600 bps
- 数据位: 8, 停止位: 1, 校验: 无
- 发送 `STOP
` 停止电机，`START
` 启动电机

## 模块依赖

```
main.c
├── bsp.h
│   ├── led.h       → PE3/PE4 LED 控制
│   ├── gpio.h      → PC13/PC14 限位开关 + LimitSemaphore
│   ├── motor.h     → motor_t 结构体 + TIM5 PWM
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

本版本 (任务3) 相对于任务2 的主要变更:

| 新增 | 说明 |
|------|------|
| vTaskAbnormalSituation | 障碍物检测与自动恢复任务 |
| vTaskLedMode | LED 4 模式独立控制任务 |
| AbnormalSituationSemaphore | 异常情况信号量 |
| AbnormalSituationDisQueue | 异常情况距离队列 |
| LedModeQueue | LED 模式指令队列 |
| motor_t 结构体 | 电机状态统一管理 |

---

> **作者**: Tom
> **项目**: 毕业设计 — 教育机器人控制系统
> **最后更新**: 2026-06-03
