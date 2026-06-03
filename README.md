# STM32F107VCT6 教育机器人控制系统 — 毕业设计项目

一套基于 STM32F107VCT6 (ARM Cortex-M3) 平台的嵌入式电机控制与传感器采集系统，使用 YS-F1PRO 开发板，涵盖从裸机轮询到 FreeRTOS 多任务的完整演进过程。

## 项目概述

本仓库包含教育机器人控制系统的五个递进版本，展示了从前后台裸机架构到 FreeRTOS 实时多任务架构的完整演进过程。每个版本在前一版本基础上逐步优化，涵盖了嵌入式软件开发中常用的设计模式和技术要点。

### 核心功能

- **直流电机控制**：PWM 脉冲驱动，带位置反馈的脉冲计数
- **双路限位保护**：EXTI 外部中断触发，自动换向
- **多传感器融合**：ADC 温度采集 + 超声波测距
- **OLED 实时显示**：I2C SSD1306 128×64 显示屏
- **串口通信**：USART3 printf 调试输出 + RS485 工业总线
- **LED 状态指示**：4 种模式（灭/常亮/慢闪/快闪）
- **障碍物检测**：基于距离的自动停机与滞后滤波恢复

## 硬件平台

| 组件 | 规格 |
|------|------|
| **主控芯片** | STM32F107VCT6 (ARM 32-bit Cortex-M3) |
| **主频** | 72 MHz |
| **Flash** | 256 KB |
| **SRAM** | 64 KB |
| **开发板** | YS-F1PRO |
| **调试器** | ST-LINK V2 (SWD) |
| **电机** | UIM240 系列 (PWM + 方向控制) |
| **温度传感器** | ADC 模拟采集 |
| **距离传感器** | DYP-A02 超声波模块 |
| **显示屏** | 0.96" OLED (SSD1306, I2C, 128×64) |
| **通信接口** | USART3 (printf 调试), RS485 (预留) |

## 版本演进

### 架构对比

| 版本 | 目录 | 架构 | 核心亮点 |
|------|------|------|----------|
| **任务1 — 比较版** | [`task1-bare-metal-compare/`](task1-bare-metal-compare/) | 裸机轮询 | 基础调试版本，限位回退 5900 脉冲 |
| **任务1 — 稳定版** | [`task1-bare-metal-stable/`](task1-bare-metal-stable/) | 裸机轮询 | 60min+ 稳定运行验证，限位回退 3050，速度 1000 |
| **任务2 — 稳定版** | [`task2-freertos-stable/`](task2-freertos-stable/) | FreeRTOS (4 任务) | 信号量 + 队列 IPC，STOP/START 指令，速度 1000 |
| **任务2 — 结构体版** | [`task2-freertos-struct-state/`](task2-freertos-struct-state/) | FreeRTOS (4 任务) | 结构体状态管理，速度 1500 |
| **任务3 — 最终版** | [`task3-freertos-final/`](task3-freertos-final/) | FreeRTOS (6 任务) | 障碍物检测恢复 + LED 模式控制 + 结构体状态管理，速度 1000 |

### 详细版本对比

| 功能 | 任务1比较 | 任务1稳定 | 任务2稳定 | 任务2结构体 | 任务3 |
|------|:---:|:---:|:---:|:---:|:---:|
| RTOS | ✗ | ✗ | ✓ FreeRTOS | ✓ FreeRTOS | ✓ FreeRTOS |
| 任务数 | 1 (主循环) | 1 (主循环) | 4 | 4 | 6 |
| 电机速度 (pps) | 默认 | 1000 | 1000 | 1500 | 1000 |
| 限位回退 (脉冲) | 5900 | 3050 | — | — | — |
| 串口指令 | ✗ | ✗ | STOP/START | STOP/START | STOP/START |
| 结构体状态管理 | ✗ | ✗ | ✗ | ✓ | ✓ |
| 信号量 IPC | ✗ | ✗ | ✓ | ✓ | ✓ |
| 消息队列 | ✗ | ✗ | ✓ | ✓ | ✓ |
| 障碍物检测恢复 | ✗ | ✗ | ✗ | ✗ | ✓ (3次确认) |
| LED 模式控制 | ✗ | ✗ | ✗ | ✗ | ✓ (4模式) |
| 稳定性验证 | ✗ | ✓ (60min+) | ✓ | ✓ | ✓ |

## 仓库结构

```
stm32f107-edu-robot/
│
├── README.md                        ← 本文件
├── LICENSE                          ← MIT 许可证
├── .gitignore                       ← Keil + 嵌入式忽略规则
│
├── task1-bare-metal-compare/        ← 任务1：裸机比较/调试版
├── task1-bare-metal-stable/         ← 任务1：裸机稳定版 (60min 验证)
├── task2-freertos-stable/           ← 任务2：FreeRTOS 稳定版
├── task2-freertos-struct-state/     ← 任务2：FreeRTOS 结构体状态管理版
└── task3-freertos-final/            ← 任务3：FreeRTOS 最终整合版
```

每个子项目包含：
```
<project>/
├── README.md            ← 详细工程说明文档
├── Drivers/             ← CMSIS 启动文件 + STM32F10x 标准外设库
├── Inc/bsp/             ← BSP 层头文件 (gpio, led, motor, oled, usart, timer, rs485)
├── Src/bsp/             ← BSP 层实现
├── Src/main.c           ← 应用入口
├── freertos/            ← FreeRTOS 内核源码 (仅任务2、3)
├── docs/                ← 补充文档与数据手册
├── F107VCT6.uvprojx     ← Keil MDK 工程文件
└── F107VCT6.uvoptx      ← Keil MDK 工程选项
```

## 快速开始

### 环境要求

- **Keil MDK-ARM v5** + ARM Compiler 5 (AC5)
- **STM32F1xx_DFP v2.3.0** 设备包
- **ST-LINK V2** 调试器
- **YS-F1PRO** 开发板 (或兼容的 STM32F107VCT6 板)

### 编译与烧录

1. 在 Keil MDK 中打开目标工程的 `F107VCT6.uvprojx`
2. 选择 Target：`YS-F1PRO` → `Application/MDK-ARM`
3. 按 **F7** 编译，确保 0 错误
4. ST-Link V2 连接开发板 (SWCLK, SWDIO, GND)
5. 按 **F8** 烧录
6. 复位开发板，观察 OLED 和串口输出

### 串口监控

- 连接到开发板的 USART3 端口
- 波特率：115200 (或 9600，取决于配置)
- 预期输出格式：
  ```
  temp:XXX
  dis:XXX
  ```
- 任务2/3 可发送 `STOP` 或 `START` + `\r\n` 控制电机

## 开发工具

| 工具 | 版本 | 用途 |
|------|------|------|
| Keil MDK-ARM | v5 | 主要 IDE，编译，调试 |
| ARM Compiler | AC5 | C/C++ 编译 |
| EIDE (可选) | — | 替代构建系统 |
| VS Code (可选) | — | 代码编辑 (`.code-workspace`) |

## 文档

每个子项目包含详细的 `README.md`，涵盖：
- 硬件平台详情
- 软件架构图
- 模块级描述 (电机、传感器、OLED、串口、LED、定时器)
- 状态机/任务流程图
- 参数表
- 中断向量分配
- 编译烧录说明
- 模块依赖关系图

补充的数据手册和参考文档位于各项目的 `docs/datasheets/` 目录。

## 许可证

本项目采用 MIT 许可证 — 详见 [LICENSE](LICENSE) 文件。

---

> **作者**：Tom
> **学位**：机器人工程学士
> **项目类型**：毕业设计
> **最后更新**：2026-06-03

---

# STM32F107VCT6 Educational Robot Control System — Graduation Project

A comprehensive embedded motor control and sensor acquisition system based on STM32F107VCT6 (ARM Cortex-M3) using the YS-F1PRO development board, demonstrating the full evolution from bare-metal polling to FreeRTOS multi-tasking architecture.

## Project Overview

This repository contains five progressive versions of an educational robot control system, demonstrating the evolution from a bare-metal foreground/background architecture to a FreeRTOS-based real-time multi-tasking system. Each version iterates on its predecessor, covering embedded software design patterns commonly used in industrial and educational robotics.

### Key Features

- **DC Motor Control**: PWM-driven with position feedback via pulse counting
- **Dual Limit Switch Protection**: EXTI-triggered automatic motor direction reversal
- **Multi-Sensor Fusion**: ADC temperature sensing + ultrasonic distance measurement
- **OLED Real-Time Display**: I2C SSD1306 128×64
- **Serial Communication**: USART3 printf debugging + RS485 industrial bus (reserved)
- **LED Status Indication**: 4-mode LED (Off/On/Slow-flash/Fast-flash)
- **Obstacle Detection**: Distance-based auto-stop with hysteresis filtering

## Hardware Platform

| Component | Specification |
|-----------|--------------|
| **MCU** | STM32F107VCT6 (ARM 32-bit Cortex-M3) |
| **Clock** | 72 MHz |
| **Flash** | 256 KB |
| **SRAM** | 64 KB |
| **Dev Board** | YS-F1PRO |
| **Debugger** | ST-LINK V2 (SWD) |
| **Motor** | UIM240 series (PWM + direction control) |
| **Temperature Sensor** | ADC-based analog |
| **Distance Sensor** | DYP-A02 ultrasonic module |
| **Display** | 0.96" OLED (SSD1306, I2C, 128×64) |
| **Communication** | USART3 (printf), RS485 (reserved) |

## Version Evolution

### Architecture Comparison

| Version | Directory | Architecture | Key Highlights |
|---------|-----------|-------------|----------------|
| **Task 1 — Compare** | [`task1-bare-metal-compare/`](task1-bare-metal-compare/) | Bare metal (polling) | Baseline debug version, limit retract=5900 pulses |
| **Task 1 — Stable** | [`task1-bare-metal-stable/`](task1-bare-metal-stable/) | Bare metal (polling) | Verified 60min+ stable run, limit retract=3050, speed=1000 |
| **Task 2 — Stable** | [`task2-freertos-stable/`](task2-freertos-stable/) | FreeRTOS (4 tasks) | Semaphore + Queue IPC, STOP/START commands, speed=1000 |
| **Task 2 — Struct** | [`task2-freertos-struct-state/`](task2-freertos-struct-state/) | FreeRTOS (4 tasks) | Struct-based state management, speed=1500 |
| **Task 3 — Final** | [`task3-freertos-final/`](task3-freertos-final/) | FreeRTOS (6 tasks) | Obstacle recovery + LED modes + struct state mgmt, speed=1000 |

### Detailed Feature Comparison

| Feature | Task1 Compare | Task1 Stable | Task2 Stable | Task2 Struct | Task3 |
|---------|:---:|:---:|:---:|:---:|:---:|
| RTOS | ✗ | ✗ | ✓ FreeRTOS | ✓ FreeRTOS | ✓ FreeRTOS |
| Task Count | 1 (main loop) | 1 (main loop) | 4 | 4 | 6 |
| Motor Speed (pps) | Default | 1000 | 1000 | 1500 | 1000 |
| Limit Retract (pulses) | 5900 | 3050 | — | — | — |
| Serial Commands | ✗ | ✗ | STOP/START | STOP/START | STOP/START |
| Struct State Mgmt | ✗ | ✗ | ✗ | ✓ | ✓ |
| Semaphore IPC | ✗ | ✗ | ✓ | ✓ | ✓ |
| Message Queues | ✗ | ✗ | ✓ | ✓ | ✓ |
| Obstacle Recovery | ✗ | ✗ | ✗ | ✗ | ✓ (3-count) |
| LED Modes | ✗ | ✗ | ✗ | ✗ | ✓ (4 modes) |
| Stable Run Verified | ✗ | ✓ (60min+) | ✓ | ✓ | ✓ |

## Repository Structure

```
stm32f107-edu-robot/
│
├── README.md                        ← This file
├── LICENSE                          ← MIT License
├── .gitignore                       ← Keil + embedded project ignore rules
│
├── task1-bare-metal-compare/        ← Task 1: Bare metal comparison/debug version
├── task1-bare-metal-stable/         ← Task 1: Bare metal stable (60min verified)
├── task2-freertos-stable/           ← Task 2: FreeRTOS stable version
├── task2-freertos-struct-state/     ← Task 2: FreeRTOS with struct state management
└── task3-freertos-final/            ← Task 3: Final consolidated FreeRTOS version
```

Each sub-project contains:
```
<project>/
├── README.md            ← Detailed engineering documentation
├── Drivers/             ← CMSIS startup + STM32F10x Standard Peripheral Library
├── Inc/bsp/             ← BSP layer headers (gpio, led, motor, oled, usart, timer, rs485)
├── Src/bsp/             ← BSP layer implementations
├── Src/main.c           ← Application entry point
├── freertos/            ← FreeRTOS kernel source (Task 2 & 3 only)
├── docs/                ← Additional documentation & datasheets
├── F107VCT6.uvprojx     ← Keil MDK project file
└── F107VCT6.uvoptx      ← Keil MDK project options
```

## Quick Start

### Prerequisites

- **Keil MDK-ARM v5** with ARM Compiler 5 (AC5)
- **STM32F1xx_DFP v2.3.0** device pack
- **ST-LINK V2** debugger
- **YS-F1PRO** development board (or compatible STM32F107VCT6 board)

### Build & Flash

1. Open the desired project's `F107VCT6.uvprojx` in Keil MDK
2. Select target: `YS-F1PRO` → `Application/MDK-ARM`
3. Build (**F7**) — ensure 0 errors
4. Connect ST-Link V2 to the board (SWCLK, SWDIO, GND)
5. Flash (**F8**)
6. Reset the board and observe OLED display + serial output

### Serial Monitor

- Connect to the board's USART3 port
- Baud rate: 115200 (or 9600 depending on configuration)
- Expected output format:
  ```
  temp:XXX
  dis:XXX
  ```
- For Task 2/3, send `STOP` or `START` followed by `\r\n` to control the motor

## Development Tools

| Tool | Version | Usage |
|------|---------|-------|
| Keil MDK-ARM | v5 | Primary IDE, compilation, debugging |
| ARM Compiler | AC5 | C/C++ compilation |
| EIDE (optional) | — | Alternative build system |
| VS Code (optional) | — | Code editing with `.code-workspace` |

## Documentation

Each sub-project includes a detailed `README.md` with:
- Hardware platform details
- Software architecture diagrams
- Module-level descriptions (motor, sensors, OLED, serial, LED, timer)
- State machine / task flow diagrams
- Parameter tables
- Interrupt vector assignments
- Build & flash instructions
- Module dependency graphs

Additional datasheets and reference manuals are available in each project's `docs/datasheets/` directory.

## License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.

---

> **Author**: Tom
> **Degree**: Bachelor of Engineering in Robotics
> **Project Type**: Graduation Design Project
> **Last Updated**: 2026-06-03
