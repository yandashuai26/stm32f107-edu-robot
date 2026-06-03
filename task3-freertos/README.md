# STM32F107VCT6 电机控制与传感器采集系统 — 任务3（FreeRTOS 最终版）

## 一、项目概述

本项目是基于 STM32F107VCT6 微控制器和 FreeRTOS 实时操作系统的**教育机器人控制系统任务3版本**。它延续了任务2建立的多任务 RTOS 架构，将限位开关处理、串口指令控制（STOP/START）、传感器数据采集和 OLED 显示集成到四个协作的 FreeRTOS 任务中。

本版本是 FreeRTOS 实现的**最终整合版本**，电机速度设置为 1000 pps。

## 二、硬件平台

| 项目 | 规格 |
|------|------|
| **主控芯片** | STM32F107VCT6 |
| **内核** | ARM 32-bit Cortex-M3 @ 72 MHz |
| **Flash** | 256 KB |
| **SRAM** | 64 KB |
| **调试器** | ST-LINK V2 (SWD) |
| **开发板** | YS-F1PRO |

### 板载外设资源

| 外设 | 接口 | 说明 |
|------|------|------|
| 电机驱动 | TIM PWM + GPIO | 脉冲控制、方向切换、启停 |
| 限位开关 1 | PC13 (EXTI) | 行程限位检测 |
| 限位开关 2 | PC14 (EXTI) | 行程限位检测 |
| 温度传感器 | ADC | 模拟温度采集 |
| 超声波距离传感器 | GPIO Trig/Echo | 距离测量 |
| OLED 显示屏 | I2C | 0.96" SSD1306, 128×64 |
| USART3 | TX/RX | 串口调试输出 + 上位机指令输入 |
| RS485 | 板载芯片 | 工业总线（预留） |
| LED | GPIO | 系统状态指示 |

## 三、软件架构

### 3.1 FreeRTOS 任务设计

| 任务 | 优先级 | 栈 | 周期 | 职责 |
|------|--------|-----|------|------|
| `vTaskLimitSwitch` | 4（最高） | 256 words | 信号量触发 | 限位开关响应、电机换向、冷却期管理 |
| `vTaskControl` | 3 | 256 words | 10ms 轮询 | 串口指令解析（STOP/START） |
| `vTaskCollect` | 2 | 256 words | 100ms 周期 | 温度 + 距离采集、队列发送 |
| `vTaskCommunication` | 1（最低） | 256 words | 80ms 周期 | 队列数据消费、printf 输出、OLED 刷新 |

### 3.2 任务间通信

- **ISR → 限位任务**：二进制信号量（`LimitSemaphore`），由 EXTI ISR 释放
- **ISR → 控制任务**：全局缓冲区 `USART3_RxBuffer[]`，临界区保护
- **采集任务 → 通信任务**：消息队列（`TempQueue`, `DisQueue`，各 20×uint32）

### 3.3 目录结构

```
task3-freertos/
├── README.md
├── Drivers/
│   ├── Start/          # 启动文件 & CMSIS 核心
│   └── Library/        # STM32F10x 标准外设库
├── Inc/
│   ├── bsp/            # BSP 模块头文件
│   ├── stm32f10x_conf.h
│   └── stm32f10x_it.h
├── Src/
│   ├── bsp/            # BSP 模块源文件
│   ├── main.c          # 入口点
│   └── stm32f10x_it.c  # ISR 实现
├── freertos/           # FreeRTOS 内核
│   ├── inc/
│   ├── src/
│   └── FreeRTOS_demo.c # 应用层（4 个任务）
├── docs/
│   ├── dev-environment.txt
│   └── datasheets/     # 参考手册与数据手册
├── F107VCT6.uvprojx    # Keil 工程
└── F107VCT6.uvoptx     # Keil 工程选项
```

### 3.4 编译环境

| 项目 | 值 |
|------|-----|
| **IDE** | Keil MDK-ARM v5 |
| **编译器** | ARM Compiler 5 (AC5) |
| **设备包** | Keil::STM32F1xx_DFP v2.3.0 |
| **RTOS** | FreeRTOS（源码集成） |
| **预定义宏** | `USE_STDPERIPH_DRIVER`, `STM32F10X_CL` |
| **内存管理** | heap_4.c |

## 四、关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 电机速度 | 1000 pps | 通过 `motor_set_speed(1000)` 设置 |
| 限位去抖 | 50 ms | 机械开关去抖 |
| 限位冷却期 | 300 ms | 防止换向期间重复触发 |
| 传感器采样 | 100 ms (10 Hz) | 温度 + 距离采集周期 |
| 显示刷新 | 80 ms | OLED 更新周期 |

## 五、版本关系

| 版本 | 架构 | 电机速度 | 核心特点 |
|------|------|----------|----------|
| task1-bare-metal-compare | 裸机（无 RTOS） | 默认 | 调试/对比基线，限位回退=5900 |
| task1-bare-metal-stable | 裸机（无 RTOS） | 1000 pps | 验证 60min+ 稳定运行，限位回退=3050 |
| task2-freertos-stable | FreeRTOS 4 任务 | 1000 pps | 多任务，信号量 + 队列 IPC |
| task2-freertos-struct-state | FreeRTOS 4 任务 | 1500 pps | 结构体状态管理 |
| **task3-freertos**（本版本） | **FreeRTOS 4 任务** | **1000 pps** | **最终整合版本** |

## 六、编译与烧录

1. 在 Keil MDK-ARM v5 中打开 `F107VCT6.uvprojx`
2. 确保已安装 STM32F1xx_DFP v2.3.0
3. 确认预定义宏：`USE_STDPERIPH_DRIVER,STM32F10X_CL`
4. 按 **F7** 编译（确保 0 错误）
5. ST-Link V2 连接 YS-F1PRO 开发板（SWD）
6. 按 **F8** 烧录
7. 打开串口监视器（115200/9600 波特率）查看传感器输出
8. 发送 `STOP` / `START` 指令控制电机

## 七、串口指令

| 指令 | 动作 |
|------|------|
| `STOP\r\n` | 立即停止电机 |
| `START\r\n` | 启动/恢复电机 |

## 八、模块依赖关系

```
main.c
  ├── bsp.h (BSP 入口)
  │   ├── motor.h/c   ← 电机驱动 + 信号量 + 冷却期
  │   ├── gpio.h/c    ← GPIO 配置
  │   ├── led.h/c     ← LED 控制
  │   ├── timer.h/c   ← 系统时基定时器
  │   ├── usart.h/c   ← USART3 通信
  │   ├── oled.h/c    ← OLED 显示 (SSD1306)
  │   └── 485.h/c     ← RS485 驱动
  └── FreeRTOS_demo.h/c ← RTOS 应用层
      └── FreeRTOS 内核
          ├── task.c/h    ← 任务管理
          ├── queue.c/h   ← 消息队列
          ├── semphr.h    ← 信号量
          └── heap_4.c    ← 内存管理
```

---

> 本文件为项目工程说明文档。版本：任务3 FreeRTOS 最终版。最后更新日期：2026-06-03

---

# STM32F107VCT6 Motor Control & Sensor Acquisition System — Task 3 (FreeRTOS Final)

## 1. Overview

This is the **Task 3** version of the educational robot control system based on STM32F107VCT6 and FreeRTOS. It continues the multi-task RTOS architecture established in Task 2, integrating limit switch handling, serial command control (STOP/START), sensor data acquisition, and OLED display into four cooperative FreeRTOS tasks.

This version represents the **consolidated final iteration** of the FreeRTOS-based implementation, with motor speed set to 1000 pps.

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
| Motor Driver | TIM PWM + GPIO | Pulse control, direction switch, start/stop |
| Limit Switch 1 | PC13 (EXTI) | Travel limit detection |
| Limit Switch 2 | PC14 (EXTI) | Travel limit detection |
| Temperature Sensor | ADC | Analog acquisition |
| Ultrasonic Distance Sensor | GPIO Trig/Echo | Distance measurement |
| OLED Display | I2C | 0.96" SSD1306, 128×64 |
| USART3 | TX/RX | Serial debug output + host command input |
| RS485 | Onboard chip | Industrial bus (reserved) |
| LED | GPIO | System status indicator |

## 3. Software Architecture

### FreeRTOS Task Design

| Task | Priority | Stack | Period | Responsibility |
|------|----------|-------|--------|----------------|
| `vTaskLimitSwitch` | 4 (highest) | 256 words | Semaphore-triggered | Limit switch response, motor direction change, cooldown management |
| `vTaskControl` | 3 | 256 words | 10ms polling | Serial command parsing (STOP/START) |
| `vTaskCollect` | 2 | 256 words | 100ms periodic | Temperature + distance acquisition, queue send |
| `vTaskCommunication` | 1 (lowest) | 256 words | 80ms periodic | Queue data consumption, printf output, OLED refresh |

### Inter-Task Communication

- **ISR → Limit Task**: Binary semaphore (`LimitSemaphore`) released from EXTI ISR
- **ISR → Control Task**: Global buffer `USART3_RxBuffer[]` with critical section protection
- **Collect Task → Communication Task**: Message queues (`TempQueue`, `DisQueue`, 20×uint32 each)

### Directory Structure

```
task3-freertos/
├── README.md
├── Drivers/
│   ├── Start/          # Startup files & CMSIS core
│   └── Library/        # STM32F10x Standard Peripheral Library
├── Inc/
│   ├── bsp/            # BSP module headers
│   ├── stm32f10x_conf.h
│   └── stm32f10x_it.h
├── Src/
│   ├── bsp/            # BSP module sources
│   ├── main.c          # Entry point
│   └── stm32f10x_it.c  # ISR implementations
├── freertos/           # FreeRTOS kernel
│   ├── inc/
│   ├── src/
│   └── FreeRTOS_demo.c # Application layer (4 tasks)
├── docs/
│   ├── dev-environment.txt
│   └── datasheets/     # Reference manuals & datasheets
├── F107VCT6.uvprojx    # Keil project
└── F107VCT6.uvoptx     # Keil project options
```

### Build Environment

| Item | Value |
|------|-------|
| **IDE** | Keil MDK-ARM v5 |
| **Compiler** | ARM Compiler 5 (AC5) |
| **Pack** | Keil::STM32F1xx_DFP v2.3.0 |
| **RTOS** | FreeRTOS (source-integrated) |
| **Preprocessor Defines** | `USE_STDPERIPH_DRIVER`, `STM32F10X_CL` |
| **Memory Management** | heap_4.c |

## 4. Key Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Motor Speed | 1000 pps | Set via `motor_set_speed(1000)` |
| Limit Debounce | 50 ms | Mechanical switch debounce |
| Limit Cooldown | 300 ms | Prevents repeated triggering during direction change |
| Sensor Sampling | 100 ms (10 Hz) | Temperature + distance acquisition period |
| Display Refresh | 80 ms | OLED update period |

## 5. Relationship to Previous Versions

| Version | Architecture | Motor Speed | Key Feature |
|---------|-------------|-------------|-------------|
| task1-bare-metal-compare | Bare metal (no RTOS) | Default | Debug/comparison baseline, limit retract=5900 |
| task1-bare-metal-stable | Bare metal (no RTOS) | 1000 pps | Verified 60min+ stable run, limit retract=3050 |
| task2-freertos-stable | FreeRTOS 4 tasks | 1000 pps | Multi-task, semaphore + queue IPC |
| task2-freertos-struct-state | FreeRTOS 4 tasks | 1500 pps | Struct-based state management |
| **task3-freertos** (this) | **FreeRTOS 4 tasks** | **1000 pps** | **Final consolidated version** |

## 6. How to Build & Flash

1. Open `F107VCT6.uvprojx` in Keil MDK-ARM v5
2. Ensure STM32F1xx_DFP v2.3.0 is installed
3. Confirm preprocessor defines: `USE_STDPERIPH_DRIVER,STM32F10X_CL`
4. Press **F7** to build (ensure 0 errors)
5. Connect ST-Link V2 to YS-F1PRO board (SWD)
6. Press **F8** to flash
7. Open serial monitor (115200/9600 baud) to view sensor output
8. Send `STOP` / `START` commands to control the motor

## 7. Serial Commands

| Command | Action |
|---------|--------|
| `STOP\r\n` | Stop the motor immediately |
| `START\r\n` | Start/resume the motor |

## 8. Module Dependencies

```
main.c
  ├── bsp.h (BSP entry)
  │   ├── motor.h/c   ← Motor driver + semaphore + cooldown
  │   ├── gpio.h/c    ← GPIO config
  │   ├── led.h/c     ← LED control
  │   ├── timer.h/c   ← System tick timer
  │   ├── usart.h/c   ← USART3 communication
  │   ├── oled.h/c    ← OLED display (SSD1306)
  │   └── 485.h/c     ← RS485 driver
  └── FreeRTOS_demo.h/c ← RTOS application layer
      └── FreeRTOS Kernel
          ├── task.c/h    ← Task management
          ├── queue.c/h   ← Message queues
          ├── semphr.h    ← Semaphores
          └── heap_4.c    ← Memory management
```

---

> Project engineering documentation. Version: Task 3 FreeRTOS Final. Last updated: 2026-06-03
