# STM32F107VCT6 电机控制与传感器采集系统 — 任务2（稳定运行版 · 结构体保存状态）

## 一、项目概述

本项目基于 STM32F107VCT6 微控制器和 **FreeRTOS 实时操作系统**，实现了一套多任务并行的直流电机控制系统。与任务1的裸机架构不同，本版本将系统功能拆分为四个独立 RTOS 任务，通过 FreeRTOS 抢占式调度器实现并行处理；任务间通过消息队列和信号量进行数据传递与同步。

本版本是任务2的**结构体保存状态增强版**，相对于基础 FreeRTOS 版，增加了系统运行状态的结构体化管理机制，电机运行速度设置为 1500 pps。系统功能包括：电机限位自动往复、上位机串口指令控制（STOP/START）、温度与距离传感器数据采集、OLED 实时显示、LED 状态指示等。

## 二、硬件平台

| 项目 | 规格 |
|------|------|
| **主控芯片** | STM32F107VCT6 |
| **内核** | ARM 32-bit Cortex-M3 |
| **主频** | 72 MHz |
| **Flash** | 256 KB |
| **SRAM** | 64 KB |
| **调试接口** | ST-LINK V2（SWD） |
| **开发板型号** | YS-F1PRO |

### 板载外设资源

| 外设 | 接口/引脚 | 说明 |
|------|-----------|------|
| **电机驱动** | TIM PWM + GPIO | PWM 脉冲控制，支持方向切换、启停、脉冲计数 |
| **限位开关 1** | PC13 (EXTI) | 电机行程限位检测，外部中断触发 |
| **限位开关 2** | PC14 (EXTI) | 电机行程限位检测，外部中断触发 |
| **温度传感器** | ADC 通道 | 模拟温度采集 |
| **超声波距离传感器** | GPIO Trig + Echo | 超声波测距（40kHz） |
| **OLED 显示屏** | I2C | 0.96 寸 SSD1306，128×64 |
| **USART3** | TX/RX | 串口通信 + 上位机指令接收 |
| **RS485** | 板载 485 芯片 | 工业总线通信接口（备用） |
| **LED** | GPIO | 系统状态指示灯 |

## 三、软件架构

### 3.1 架构概览 — FreeRTOS 多任务系统

```
┌────────────────────────────────────────────────────────────┐
│                       main()                                │
│  bsp_init() → motor_set_speed(1500) → FreeRTOS_Start()     │
│                       │                                     │
│              vTaskStartScheduler()                          │
│                       │                                     │
│         ┌─────────────┼─────────────┐                      │
│         ▼             ▼             ▼                      │
│  ┌────────────┐ ┌──────────┐ ┌────────────┐               │
│  │ 限位任务    │ │ 控制任务  │ │ 采集任务    │               │
│  │ (优先级4)  │ │ (优先级3) │ │ (优先级2)  │               │
│  │            │ │          │ │            │               │
│  │ 信号量等待 │ │ 指令解析  │ │ 传感器采集  │               │
│  │ 换向处理   │ │ STOP/STR │ │ 队列发送    │               │
│  │ 冷却期管理 │ │ 串口回复  │ │ 100ms周期  │               │
│  └─────┬──────┘ └────┬─────┘ └─────┬──────┘               │
│        │              │             │                       │
│        │     ┌────────┼─────────────┤                      │
│        │     │        │    TempQueue│                      │
│        │     │        │    DisQueue │                      │
│        │     │        ▼             ▼                      │
│        │     │  ┌────────────────────────┐                │
│        │     │  │     通信任务 (优先级1)   │                │
│        │     │  │   队列接收 → printf     │                │
│        │     │  │   OLED 显示更新         │                │
│        │     │  │   80ms 周期             │                │
│        │     │  └────────────────────────┘                │
│        │     │                                             │
│   ┌────┴─────┴────┐                                       │
│   │ 中断服务层     │                                       │
│   │ TIM1 时基     │                                       │
│   │ EXTI 限位中断  │ ──→ 信号量释放                        │
│   │ USART3 接收   │ ──→ 缓冲区填充                        │
│   └───────────────┘                                       │
└────────────────────────────────────────────────────────────┘
```

### 3.2 任务设计方案

| 任务名称 | 优先级 | 栈大小 | 周期/触发方式 | 核心职责 |
|----------|--------|--------|--------------|----------|
| `vTaskLimitSwitch` | **4**（最高） | 256 words | 信号量触发（EXTI 中断释放） | 限位开关响应、电机换向、冷却期管理 |
| `vTaskControl` | **3** | 256 words | 10ms 周期轮询 | 串口指令解析（STOP/START）、电机启停控制 |
| `vTaskCollect` | **2** | 256 words | 100ms 周期 | 温度与距离传感器数据采集、队列发送 |
| `vTaskCommunication` | **1**（最低） | 256 words | 80ms 周期 | 队列数据消费、printf 输出、OLED 显示更新 |

优先级设计原则：**限位保护 > 指令控制 > 数据采集 > 通信显示**，确保安全相关的限位处理优先于常规数据流。

### 3.3 任务间通信机制

```
┌──────────────┐     信号量       ┌──────────────┐
│ EXTI 限位ISR  │ ──Release──→   │ 限位任务      │
│              │   (Binary Sem)  │ (优先级4)     │
└──────────────┘                 └──────────────┘

┌──────────────┐     全局缓冲区   ┌──────────────┐
│ USART3 ISR   │ ──写入──→       │ 控制任务      │
│              │   RxBuffer[]    │ (优先级3)     │
└──────────────┘                 └──────────────┘

┌──────────────┐   TempQueue     ┌──────────────┐
│ 采集任务      │ ──Send──→      │ 通信任务      │
│ (优先级2)     │   DisQueue     │ (优先级1)     │
└──────────────┘                 └──────────────┘
```

### 3.4 目录结构

```
项目根目录/
├── Drivers/
│   ├── Start/          # 启动文件 (.s) 与 CMSIS 核心文件
│   └── Library/        # STM32F10x 标准外设库
├── Inc/
│   ├── bsp/            # BSP 模块头文件
│   │   ├── motor.h     # 电机驱动头文件（含信号量/冷却期/限位标志接口）
│   │   ├── 485.h / bsp.h / gpio.h / led.h / oled.h / oledfont.h / timer.h / usart.h
│   ├── stm32f10x_conf.h
│   └── stm32f10x_it.h
├── Src/
│   ├── bsp/            # BSP 模块源文件
│   │   ├── motor.c     # 电机驱动核心（含信号量/冷却期/状态结构体）
│   │   ├── 485.c / bsp.c / gpio.c / led.c / oled.c / timer.c / usart.c
│   ├── main.c          # 主函数（初始化 + 启动调度器）
│   └── stm32f10x_it.c  # 中断服务函数实现（限位 EXTI + USART3）
├── freertos/           # FreeRTOS 内核源码
│   ├── inc/            # FreeRTOS 头文件 (FreeRTOS.h, FreeRTOSConfig.h, task.h, queue.h...)
│   ├── src/            # FreeRTOS 源文件 (tasks.c, queue.c, heap_4.c, port.c...)
│   ├── FreeRTOS_demo.c # FreeRTOS 应用层入口（任务创建与调度启动）
│   └── FreeRTOS_demo.h # 应用层头文件
├── docs/               # 说明文档与数据手册
├── F107VCT6.uvprojx    # Keil MDK 工程文件
└── F107VCT6.uvoptx     # Keil MDK 工程配置
```

### 3.5 编译环境与配置

| 项目 | 说明 |
|------|------|
| **IDE** | Keil MDK-ARM v5 |
| **编译器** | ARM Compiler 5 (AC5) |
| **设备包** | Keil::STM32F1xx_DFP v2.3.0 |
| **RTOS** | FreeRTOS (集成于 freertos/ 目录) |
| **预定义宏** | `USE_STDPERIPH_DRIVER`, `STM32F10X_CL` |
| **内存管理** | heap_4.c（支持碎片合并） |
| **调试器** | ST-Link Debugger |

## 四、核心功能模块详解

### 4.1 FreeRTOS 应用层 (`FreeRTOS_demo.c`)

#### 初始化流程

```
FreeRTOS_Start()
  ├── limit_semaphore_create()     # 创建限位信号量（二进制信号量）
  ├── xQueueCreate(20, uint32_t)  # 创建温度消息队列 (TempQueue)
  ├── xQueueCreate(20, uint32_t)  # 创建距离消息队列 (DisQueue)
  ├── xTaskCreate() × 4           # 创建 4 个任务
  └── vTaskStartScheduler()       # 启动 FreeRTOS 调度器（此后不再返回）
```

#### 任务 1：限位开关任务 (`vTaskLimitSwitch` — 优先级 4)

```
while(1):
  xSemaphoreTake(LimitSemaphore, portMAX_DELAY)  ← 阻塞等待限位信号量
  vTaskDelay(50ms)                                ← 去抖延时
  trigger_pin = get_limit_trigger_pin()           ← 读取触发引脚号
  if trigger_pin == 13 (PC13):
    motor_reset_pulse_count()                     ← 清零脉冲计数
    motor_change_dir()                            ← 电机换向
    set_limit_reached_flag()                      ← 设置限位到达标志
    limit_enter_cooldown()                        ← 进入冷却期（抑制重复触发）
    vTaskDelay(300ms)                             ← 冷却期等待
    limit_exit_cooldown()                         ← 退出冷却期
  if trigger_pin == 14 (PC14):
    (同上处理逻辑)
```

**设计要点**：
- 使用**二进制信号量**实现 ISR → Task 的事件传递，避免在 ISR 中执行耗时操作
- 50ms 去抖延时防止限位开关机械弹跳导致的多次触发
- 300ms 冷却期机制防止电机在换向过程中再次触发同一限位开关
- 优先级最高（4），确保限位事件得到最及时响应

#### 任务 2：指令控制任务 (`vTaskControl` — 优先级 3)

```
while(1):
  进入临界区 → 读取 USART3_RxFinished / USART3_RxCount → 退出临界区
  if 接收到完整帧:
    进入临界区 → 拷贝数据到 cmd_buffer → 清除接收标志 → 退出临界区
    剥离 \r \n 换行符
    if cmd == "STOP":  motor_stop()  + printf("STOP\r\n")
    if cmd == "START": motor_start() + printf("START\r\n")
  vTaskDelay(10ms)
```

**设计要点**：
- 使用 `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()` 保护共享资源（USART3 接收缓冲区）的原子访问
- 支持的指令集：`STOP`（停止电机）、`START`（启动电机）
- 10ms 轮询周期保证指令响应及时性

#### 任务 3：传感器采集任务 (`vTaskCollect` — 优先级 2)

```
while(1):
  temp = get_tempture()          ← ADC 读取温度
  dis  = get_distance()          ← 超声波读取距离
  xQueueSend(TempQueue, &temp, 0) ← 非阻塞发送到温度队列
  xQueueSend(DisQueue,  &dis,  0) ← 非阻塞发送到距离队列
  vTaskDelay(100ms)               ← 100ms 采集周期
```

**设计要点**：
- 100ms 采集周期（10Hz 采样率），满足温度与距离的实时性要求
- 非阻塞队列发送（`xQueueSend(..., 0)`），队列满时丢弃旧数据并打印警告

#### 任务 4：通信显示任务 (`vTaskCommunication` — 优先级 1)

```
while(1):
  if xQueueReceive(TempQueue, &temp, 0):  ← 非阻塞接收
    printf("temp:%d\r\n", temp)
    OLED_ShowString(行2, "temp:XXXX")
  if xQueueReceive(DisQueue, &dis, 0):
    printf("dis:%d\r\n", dis)
    OLED_ShowString(行4, "dis:XXXX")
  vTaskDelay(80ms)
```

**设计要点**：
- 消费采集任务产生的数据，实现**生产者-消费者模式**
- 80ms 处理周期略快于采集周期（100ms），确保数据不堆积
- 同时输出到串口（调试监控）和 OLED（本地显示）

### 4.2 电机控制模块 (`motor.c` / `motor.h`)

本版本为 FreeRTOS 集成做了以下增强：

- **信号量支持**：`limit_semaphore_create()` 创建限位信号量，在 EXTI 中断中通过 `xSemaphoreGiveFromISR()` 释放
- **冷却期管理**：`limit_enter_cooldown()` / `limit_exit_cooldown()` 实现 300ms 触发抑制窗口
- **限位触发引脚记录**：`get_limit_trigger_pin()` 在 ISR 中记录具体触发的 GPIO 引脚号，供任务层精确判断
- **状态结构体**：将电机运行状态（方向、脉冲计数、限位标志、运行标志等）封装为结构体，便于状态保存与恢复

### 4.3 传感器采集模块

#### 温度传感器
- ADC 外设采集，`get_tempture()` 函数返回当前温度 ADC 原始值
- 采集任务中每 100ms 调用一次，数据通过 `TempQueue` 传递到通信任务

#### 距离传感器（超声波）
- GPIO 触发 + 回波脉宽测量
- `get_distance()` 函数返回距离值
- 采集任务中每 100ms 调用一次，数据通过 `DisQueue` 传递到通信任务

### 4.4 串口通信模块 (`usart.c` / `usart.h`)

- **USART3** 中断接收框架：
  - 全局缓冲区 `USART3_RxBuffer[]` 存储接收数据
  - `USART3_RxIndex` 记录当前写入位置
  - `USART3_RxCount` 记录本帧接收字节数
  - `USART3_RxFinished` 标志帧接收完成（检测到帧结束符 `\r\n`）
- **临界区保护**：控制任务中访问缓冲区时必须进入临界区，防止与 ISR 冲突
- **printf 重定向**：重写 `fputc()` 映射到 USART3

### 4.5 OLED 显示模块 (`oled.c` / `oled.h`)

- I2C 通信，驱动 SSD1306
- 显示布局：
  - 第 2 行：温度值（`temp:XXXX`）
  - 第 4 行：距离值（`dis:XXXX`）
- 更新时间由通信任务控制（80ms 周期）

### 4.6 定时器与中断管理

| 中断源 | 服务函数 | 功能 |
|--------|----------|------|
| TIM1_UP | `TIM1_UP_IRQHandler()` | 1ms 系统时基（`sysTick_ms`），5s 循环 |
| EXTI (PC13) | `EXTI15_10_IRQHandler()` | 记录触发引脚为 13，释放限位信号量 |
| EXTI (PC14) | `EXTI15_10_IRQHandler()` | 记录触发引脚为 14，释放限位信号量 |
| USART3 | `USART3_IRQHandler()` | 接收上位机指令字节 |

## 五、FreeRTOS 配置 (`FreeRTOSConfig.h`)

| 配置项 | 说明 |
|--------|------|
| `configUSE_PREEMPTION` | 1（抢占式调度） |
| `configUSE_TIME_SLICING` | 1（同优先级时间片轮转） |
| `configTICK_RATE_HZ` | 1000（1ms 系统节拍） |
| `configMAX_PRIORITIES` | ≥ 5 |
| `configMINIMAL_STACK_SIZE` | 128 words |
| `configUSE_MUTEXES` | 1 |
| `configUSE_COUNTING_SEMAPHORES` | 1 |

## 六、版本特点与对比

### 本版本（结构体保存状态版）特点

1. **FreeRTOS 多任务架构**：系统功能分解为 4 个独立任务，各司其职
2. **状态结构体管理**：电机运行状态封装为结构体，便于整体保存、恢复与调试
3. **电机速度 1500 pps**：相比基础版（1000 pps）运行速度更高
4. **信号量 + 冷却期机制**：可靠解决限位开关抖动与重复触发问题
5. **消息队列解耦**：采集任务与显示任务通过队列通信，互不阻塞
6. **串口指令控制**：支持上位机发送 STOP/START 实时控制电机

### 与任务2基础版的差异

| 对比项 | 任务2 基础版 | 本版本（结构体保存状态） |
|--------|-------------|-------------------------|
| 电机速度 | `motor_set_speed(1000)` | `motor_set_speed(1500)` |
| 状态管理 | 分散全局变量 | **结构体封装管理** |
| 状态持久化 | 无 | **支持状态结构体整体保存/恢复** |

### 与任务1（裸机版）的架构差异

| 维度 | 任务1（裸机） | 本版本（FreeRTOS） |
|------|-------------|-------------------|
| 调度方式 | 前后台轮询 | FreeRTOS 抢占式 |
| 任务划分 | 单循环顺序执行 | 4 个独立任务并行 |
| 限位处理 | 轮询标志位 | 信号量驱动 + 独立任务 |
| 数据传递 | 全局变量 | 消息队列（Queue） |
| 指令控制 | 未实现 | STOP/START 指令 |
| CPU 效率 | 100% 忙等 | 阻塞时释放 CPU |
| 实时性 | 依赖循环速度 | 优先级抢占保证 |

## 七、编译与烧录步骤

1. 使用 Keil MDK-ARM v5 打开 `F107VCT6.uvprojx`
2. 确认 STM32F1xx_DFP v2.3.0 设备包已安装
3. 检查工程包含 `freertos/` 目录下的所有源文件
4. 编译（F7），确保无错误
5. ST-Link V2 连接 YS-F1PRO 开发板
6. 下载（F8）并复位运行
7. 打开串口助手（115200/9600 波特率）观察输出
8. 发送 `STOP` / `START` 指令测试电机控制

## 八、模块依赖关系图

```
FreeRTOS_demo.c (应用层入口)
  ├── FreeRTOS 内核
  │   ├── task.c/h        ← 任务管理
  │   ├── queue.c/h       ← 消息队列
  │   ├── semphr.h        ← 信号量
  │   ├── heap_4.c        ← 内存管理
  │   └── port.c/macro.h  ← Cortex-M3 移植
  └── bsp.h (BSP 层)
      ├── motor.h/c       ← 电机驱动 + 信号量 + 冷却期
      ├── gpio.h/c        ← GPIO 配置
      ├── led.h/c         ← LED 控制
      ├── timer.h/c       ← 定时器
      ├── usart.h/c       ← 串口通信
      ├── oled.h/c        ← OLED 显示
      └── 485.h/c         ← RS485 驱动
```

---

> 本文件为项目工程说明文档。版本：任务2 稳定运行版 · 结构体保存状态。最后更新日期：2026-06-03

---

# STM32F107VCT6 Motor Control & Sensor Acquisition System — Task 2 (Stable · Struct State Management)

## 1. Overview

This project implements a **multi-tasking parallel motor control system** based on the STM32F107VCT6 MCU and **FreeRTOS RTOS**. Unlike Task 1's bare-metal architecture, this version splits system functionality into four independent RTOS tasks with preemptive scheduling. Inter-task communication uses message queues and semaphores.

This is the **struct-state-management enhanced version of Task 2**. Compared to the basic FreeRTOS version, it adds struct-based system state management, with motor speed set to 1500 pps. Features include: automatic motor reciprocation with limit switches, host serial command control (STOP/START), temperature and distance sensor acquisition, OLED real-time display, and LED status indication.

## 2. Hardware Platform

| Item | Specification |
|------|--------------|
| **MCU** | STM32F107VCT6 |
| **Core** | ARM 32-bit Cortex-M3 @ 72 MHz |
| **Flash** | 256 KB |
| **SRAM** | 64 KB |
| **Debugger** | ST-LINK V2 (SWD) |
| **Dev Board** | YS-F1PRO |

### Onboard Peripherals

| Peripheral | Interface | Description |
|------------|-----------|-------------|
| Motor Driver | TIM PWM + GPIO | Pulse, direction, start/stop |
| Limit Switch 1 | PC13 (EXTI) | Travel limit |
| Limit Switch 2 | PC14 (EXTI) | Travel limit |
| Temperature Sensor | ADC | Analog acquisition |
| Ultrasonic Sensor | GPIO Trig/Echo | 40kHz ranging |
| OLED | I2C | 0.96" SSD1306, 128×64 |
| USART3 | TX/RX | Serial + host commands |
| RS485 | Onboard chip | Industrial bus (reserved) |
| LED | GPIO | Status indicator |

## 3. Software Architecture

### 3.1 FreeRTOS Multi-Task System

Four tasks with preemptive priority scheduling. Priority: **limit protection > command control > data acquisition > communication display**.

| Task | Priority | Stack | Trigger | Responsibility |
|------|----------|-------|---------|----------------|
| `vTaskLimitSwitch` | 4 (highest) | 256 words | Semaphore (EXTI) | Limit response, direction change, cooldown |
| `vTaskControl` | 3 | 256 words | 10ms polling | Command parsing (STOP/START) |
| `vTaskCollect` | 2 | 256 words | 100ms periodic | Sensor acquisition, queue send |
| `vTaskCommunication` | 1 (lowest) | 256 words | 80ms periodic | Queue consume, printf, OLED |

### 3.2 Inter-Task Communication

| Object | Type | Direction | Purpose |
|--------|------|-----------|---------|
| `LimitSemaphore` | Binary semaphore | ISR → Limit Task | Limit event |
| `USART3_RxBuffer[]` | Global buffer | ISR → Control Task | Serial commands |
| `TempQueue` | Queue (20×uint32) | Collect → Communication | Temperature |
| `DisQueue` | Queue (20×uint32) | Collect → Communication | Distance |

### 3.3 Directory Structure

```
project-root/
├── Drivers/          # CMSIS startup + STM32F10x SPL
├── Inc/bsp/          # BSP headers (motor.h with semaphore/cooldown/struct interfaces)
├── Src/bsp/          # BSP sources (motor.c with semaphore/cooldown/state struct)
├── freertos/         # FreeRTOS kernel + FreeRTOS_demo.c (4 tasks)
├── docs/             # Documentation & datasheets
├── F107VCT6.uvprojx  # Keil MDK project
└── F107VCT6.uvoptx   # Keil MDK project options
```

### 3.4 Build Environment

| Item | Value |
|------|-------|
| **IDE** | Keil MDK-ARM v5 |
| **Compiler** | ARM Compiler 5 (AC5) |
| **Device Pack** | Keil::STM32F1xx_DFP v2.3.0 |
| **RTOS** | FreeRTOS (source-integrated) |
| **Defines** | `USE_STDPERIPH_DRIVER`, `STM32F10X_CL` |
| **Memory** | heap_4.c (coalescing) |

## 4. Core Module Details

### 4.1 FreeRTOS Application Layer

**Task 1 — Limit Switch** (prio 4): Blocks on `LimitSemaphore`. On trigger: 50ms debounce → get trigger pin → reset pulses → change direction → enter cooldown (300ms) → exit cooldown. ISR is minimal (record pin + `xSemaphoreGiveFromISR`).

**Task 2 — Control** (prio 3): Polls every 10ms in critical sections. Supports `STOP` and `START` commands via `strcmp()`.

**Task 3 — Collect** (prio 2): Every 100ms: `get_tempture()` + `get_distance()` → non-blocking `xQueueSend()`.

**Task 4 — Communication** (prio 1): Every 80ms: non-blocking `xQueueReceive()` → `printf()` + `OLED_ShowString()`.

### 4.2 Motor Control (`motor.c` / `motor.h`)

FreeRTOS enhancements:
- **Semaphore support**: `limit_semaphore_create()` + `xSemaphoreGiveFromISR()`
- **Cooldown management**: 300ms trigger suppression window
- **Trigger pin recording**: ISR records pin → task queries via `get_limit_trigger_pin()`
- **State struct**: Motor state (direction, pulse count, limit flags, run flags) encapsulated in a struct for save/restore

### 4.3 Sensors, Serial, OLED

**Temperature**: ADC, 10Hz, via `get_tempture()`. **Distance**: Ultrasonic, 10Hz, via `get_distance()`. **Serial**: USART3 with printf redirection + interrupt reception with critical section protection. **OLED**: I2C SSD1306, 128×64, rows 2/4 for temp/distance.

### 4.4 Interrupts

| Source | ISR | Function |
|--------|-----|----------|
| TIM1_UP | `TIM1_UP_IRQHandler()` | 1ms tick, 5s cycle |
| EXTI (PC13) | `EXTI15_10_IRQHandler()` | Record pin 13, give semaphore |
| EXTI (PC14) | `EXTI15_10_IRQHandler()` | Record pin 14, give semaphore |
| USART3 | `USART3_IRQHandler()` | Byte reception |

## 5. FreeRTOS Configuration

| Config | Value | Description |
|--------|-------|-------------|
| `configUSE_PREEMPTION` | 1 | Preemptive scheduling |
| `configTICK_RATE_HZ` | 1000 | 1ms tick |
| `configMAX_PRIORITIES` | ≥ 5 | Priorities 0-4 |
| `configMINIMAL_STACK_SIZE` | 128 words | 512 bytes |
| `configUSE_MUTEXES` | 1 | Mutex enabled |

## 6. Version Characteristics

### Key Features
1. **4-task FreeRTOS** with priority preemption
2. **Struct-based state management** — motor state encapsulated for save/restore/debug
3. **Motor speed 1500 pps** — faster than basic version (1000)
4. **Semaphore + cooldown** — reliable limit switch debouncing
5. **Queue-based producer-consumer** — decoupled acquisition & display
6. **Serial commands** — real-time STOP/START control

### vs. Basic Task 2 Version

| Item | Basic | This (Struct-State) |
|------|-------|---------------------|
| Motor Speed | 1000 pps | **1500 pps** |
| State Management | Scattered globals | **Struct encapsulation** |
| State Persistence | None | **Struct save/restore** |

### vs. Task 1 (Bare Metal)

| Aspect | Task 1 | This Version |
|--------|--------|-------------|
| Scheduling | Polling loop | Preemptive RTOS |
| Tasks | 1 (main loop) | 4 independent |
| Limit Handling | Poll flags | Semaphore + dedicated task |
| Data Flow | Global variables | Message queues |
| Commands | Not implemented | STOP/START |
| CPU Efficiency | 100% busy-wait | Blocks when idle |

## 7. Build & Flash

1. Open `F107VCT6.uvprojx` in Keil MDK-ARM v5
2. Confirm `STM32F1xx_DFP` v2.3.0 installed
3. Verify `freertos/` sources included
4. F7 build, F8 flash via ST-Link V2
5. Serial monitor (115200/9600), send `STOP`/`START` to test

## 8. Module Dependencies

```
FreeRTOS_demo.c
  ├── FreeRTOS Kernel (task, queue, semphr, heap_4, port)
  └── bsp.h
      ├── motor.h/c   ← Motor + semaphore + cooldown + state struct
      ├── gpio.h/c    ← GPIO config
      ├── led.h/c     ← LED control
      ├── timer.h/c   ← Timer
      ├── usart.h/c   ← Serial
      ├── oled.h/c    ← OLED display
      └── 485.h/c     ← RS485 driver
```

---

> Project engineering documentation. Version: Task 2 Stable · Struct State Management. Last updated: 2026-06-03
