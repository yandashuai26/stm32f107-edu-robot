# STM32F107VCT6 电机控制与传感器采集系统 — 任务1（比较输出版）

## 一、项目概述

本项目基于 STM32F107VCT6 微控制器，实现了一套**前后台（裸机）架构**的直流电机控制系统。系统通过定时器产生 PWM 脉冲驱动步进/直流电机，同时集成温度传感器、超声波距离传感器和 OLED 显示屏，实现了电机的自动往复运动、限位检测、障碍物检测以及传感器数据的实时显示与串口输出。

本版本为**任务1的前期调试版本**，采用裸机轮询架构，所有业务逻辑集中在 `main()` 函数的主循环中执行，未引入实时操作系统（FreeRTOS 代码已集成但未实际启用）。

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

| 外设 | 说明 |
|------|------|
| **电机驱动** | 通过 GPIO + PWM 控制，支持方向切换、启停、脉冲计数 |
| **限位开关** | PC13 / PC14 双路限位，EXTI 外部中断触发 |
| **温度传感器** | 通过 ADC 采集模拟温度数据 |
| **超声波距离传感器** | 通过 GPIO 触发测距，返回脉冲宽度计算距离 |
| **OLED 显示屏** | I2C 接口 0.96 寸 OLED（SSD1306），128×64 分辨率 |
| **USART3** | 串口通信，printf 输出传感器数据，中断接收上位机指令 |
| **RS485** | 板载 485 接口驱动（备用） |
| **LED** | 状态指示灯，支持常亮/闪烁/熄灭模式 |

## 三、软件架构

### 3.1 整体架构

```
┌─────────────────────────────────────────────┐
│                  main()                      │
│  ┌───────────────────────────────────────┐  │
│  │         bsp_init() 初始化              │  │
│  │  - GPIO / TIM / USART / ADC / I2C    │  │
│  │  - OLED 初始化                        │  │
│  │  - 电机初始化, 设置目标脉冲数          │  │
│  └───────────────────────────────────────┘  │
│  ┌───────────────────────────────────────┐  │
│  │         while(1) 主循环               │  │
│  │  - 定时采集温度/距离 (每1s)           │  │
│  │  - 限位开关处理                       │  │
│  │  - 目标到达处理                       │  │
│  │  - 温度超限告警 (>500)                │  │
│  │  - 障碍物检测 (<40cm)                 │  │
│  │  - LED 模式控制                       │  │
│  │  - 电机运行状态控制                    │  │
│  └───────────────────────────────────────┘  │
│  ┌───────────────────────────────────────┐  │
│  │    TIM1_UP_IRQHandler (中断)          │  │
│  │  - 1ms 系统时基                       │  │
│  │  - 5s 循环复位                        │  │
│  └───────────────────────────────────────┘  │
└─────────────────────────────────────────────┘
```

### 3.2 目录结构

```
项目根目录/
├── Drivers/
│   ├── Start/          # 启动文件 (.s) 与 CMSIS 核心文件
│   └── Library/        # STM32F10x 标准外设库
├── Inc/
│   ├── bsp/            # BSP 模块头文件
│   │   ├── 485.h       # RS485 通信头文件
│   │   ├── bsp.h       # 总 BSP 头文件（汇总包含）
│   │   ├── gpio.h      # GPIO 初始化头文件
│   │   ├── led.h       # LED 控制头文件
│   │   ├── motor.h     # 电机驱动头文件
│   │   ├── oled.h      # OLED 显示驱动头文件
│   │   ├── oledfont.h  # OLED 字库
│   │   ├── timer.h     # 定时器配置头文件
│   │   └── usart.h     # 串口配置头文件
│   ├── stm32f10x_conf.h  # 标准外设库配置文件
│   └── stm32f10x_it.h    # 中断服务函数声明
├── Src/
│   ├── bsp/            # BSP 模块源文件
│   │   ├── 485.c       # RS485 通信实现
│   │   ├── bsp.c       # BSP 统一初始化入口
│   │   ├── gpio.c      # GPIO 配置实现
│   │   ├── led.c       # LED 驱动实现
│   │   ├── motor.c     # 电机驱动核心（PWM/方向/脉冲计数）
│   │   ├── oled.c      # OLED 驱动实现（I2C/SSD1306）
│   │   ├── timer.c     # 定时器初始化实现
│   │   └── usart.c     # 串口初始化实现
│   ├── main.c          # 主函数（核心业务逻辑）
│   └── stm32f10x_it.c  # 中断服务函数实现
├── freertos/           # FreeRTOS 源码（本版本未启用）
│   ├── inc/            # FreeRTOS 头文件
│   ├── src/            # FreeRTOS 源文件
│   ├── FreeRTOS_demo.c # FreeRTOS 入口（空实现）
│   └── FreeRTOS_demo.h
├── docs/               # 说明文档与数据手册
├── F107VCT6.uvprojx    # Keil MDK 工程文件
└── F107VCT6.uvoptx     # Keil MDK 工程配置
```

### 3.3 编译环境与配置

| 项目 | 说明 |
|------|------|
| **IDE** | Keil MDK-ARM v5 |
| **编译器** | ARM Compiler 5 (AC5) |
| **设备包** | Keil::STM32F1xx_DFP v2.3.0 |
| **预定义宏** | `USE_STDPERIPH_DRIVER`, `STM32F10X_CL` |
| **优化等级** | 默认（-O0 或 -O1） |
| **调试器** | ST-Link Debugger |

## 四、核心功能模块详解

### 4.1 电机控制模块 (`motor.c` / `motor.h`)

电机采用**脉冲计数 + 方向控制**的方式实现位置控制：

- **速度控制**：通过定时器 PWM 频率调节电机转速
- **方向控制**：通过 `motor_change_dir()` 切换 GPIO 电平实现正反转
- **位置控制**：`motor_set_pulse_count()` 设定目标脉冲数，定时器中断中累计脉冲，到达目标后触发 `motor_is_target_reached()` 标志
- **限位保护**：
  - PC13 / PC14 接入两路限位开关，通过 EXTI 外部中断触发
  - 限位触发后：停止电机 → 延时 300ms → 换向 → 回退 5900 个脉冲 → 清除脉冲计数 → 重新使能限位中断
- **障碍物保护**：距离传感器检测到距离 < 40cm 时停止电机，需连续 3 次检测距离 > 40cm 才恢复运行

### 4.2 传感器采集模块

#### 温度传感器
- 通过 ADC 外设采集模拟信号
- `get_tempture()` 函数返回当前温度值（ADC 原始值映射）
- 阈值：温度 > 500 触发告警（LED 闪烁模式 2）
- 温度恢复后自动解除告警

#### 距离传感器（超声波）
- 通过 GPIO 触发超声波模块发射信号
- 测量回波脉冲宽度，换算为距离值（单位：mm/cm）
- `get_distance()` 函数返回当前距离值
- 阈值：距离 < 40（约 40cm）触发障碍物停止保护

### 4.3 OLED 显示模块 (`oled.c` / `oled.h`)

- 基于 I2C 通信协议，驱动 SSD1306 控制器
- 分辨率：128 × 64 像素
- 显示内容：
  - 第 0/6 行：限位/停止状态提示（"stop" 字符）
  - 第 2 行：距离值实时显示（格式：`dis:XXXX`）
  - 第 4 行：温度值实时显示（格式：`temp:XXXX`）
- 字库：`oledfont.h` 包含 8×16 ASCII 字库

### 4.4 串口通信模块 (`usart.c` / `usart.h`)

- **USART3**：与上位机通信
- **波特率**：115200（或 9600，具体见代码配置）
- **printf 重定向**：通过 `fputc()` 重定向到 USART3，方便调试输出
- **中断接收**：
  - 使用中断方式接收上位机指令
  - `USART3_RxBuffer[]` 存储接收数据
  - `USART3_RxFinished` 标志一帧接收完成
  - `USART3_RxCount` 记录接收字节数
- **输出内容**：每秒打印一次温度值和距离值（格式：`temp:xxx\r\ndis:xxx\r\n`）

### 4.5 LED 状态指示模块 (`led.c` / `led.h`)

系统通过 4 种 LED 模式反映系统运行状态：

| 模式 | 值 | 行为 | 含义 |
|------|-----|------|------|
| 关 | 0 | LED 熄灭 | 系统待机/初始化 |
| 常亮 | 1 | LED 常亮 | 系统正常运行 |
| 慢闪 | 2 | 500ms 亮 / 500ms 灭（1Hz） | 温度超限告警 |
| 快闪 | 3 | 100ms 亮 / 100ms 灭（5Hz） | 障碍物检测停止 |

### 4.6 定时器模块 (`timer.c` / `timer.h`)

- **TIM1**：系统时基定时器
  - 配置为 1ms 中断一次
  - 中断服务函数：`TIM1_UP_IRQHandler()`
  - 维护 `sysTick_ms` 计数器（0~5000 循环）
  - 用于 LED 闪烁周期控制和定时采集触发

### 4.7 RS485 模块 (`485.c` / `485.h`)

- 板载 RS485 收发器驱动
- 方向控制：通过 GPIO 控制 485 芯片的收发使能引脚
- 为工业现场总线通信预留接口

## 五、主循环业务逻辑（状态机）

```
┌──────────┐
│ 系统初始化 │ → bsp_init() → motor_set_pulse_count(10000000) → motor_start()
└────┬─────┘
     ▼
┌──────────────────────────────────────────────────────┐
│                   主循环 while(1)                      │
│                                                       │
│  [1] 每1s采集温度/距离 → 串口打印 → OLED更新          │
│                                                       │
│  [2] 限位开关触发? ──YES──→ 停电机 → 换向 → 回退5900脉冲│
│                                                       │
│  [3] 到达目标脉冲? ──YES──→ 重置脉冲 → 停电机 → 重启  │
│                                                       │
│  [4] 温度>500? ──YES──→ LED_MODE=2 (慢闪告警)        │
│                                                       │
│  [5] 距离<40? ──YES──→ 停电机, LED_MODE=3 (快闪告警)  │
│      距离恢复>40×3次? ──YES──→ 恢复电机运行           │
│                                                       │
│  [6] LED模式切换 (根据LED_MODE)                       │
│                                                       │
│  [7] 电机状态保持 (motor_is_runing标志)               │
└──────────────────────────────────────────────────────┘
```

### 关键参数说明

| 参数 | 值 | 说明 |
|------|-----|------|
| 目标脉冲数 | 10000000 | 电机单向运行的目标脉冲（极大值，等效于无限运行） |
| 限位回退脉冲 | 5900 | 限位触发后电机反向回退的脉冲数 |
| 温度告警阈值 | 500 | ADC 温度值超过此值触发告警 |
| 障碍物距离阈值 | 40 | 距离小于此值（cm）触发停止 |
| 障碍物恢复确认次数 | 3 | 连续3次检测距离>40才恢复 |
| 数据采集周期 | 1000ms | 每1秒采集一次传感器数据 |

## 六、中断向量分配

| 中断源 | 中断函数 | 功能 |
|--------|----------|------|
| TIM1_UP | `TIM1_UP_IRQHandler()` | 1ms 系统时基 |
| EXTI (PC13/PC14) | 外部中断 | 限位开关触发 |
| USART3 | USART3 中断 | 串口指令接收 |

## 七、版本特点与注意事项

### 本版本特点
1. **裸机前后台架构**：所有业务逻辑在 `main()` 循环中轮询执行，无 RTOS 任务调度
2. **FreeRTOS 未启用**：`FreeRTOS_demo.c` 中的 `FreeRTOS_Start()` 为空实现，调度器未启动
3. **限位回退脉冲 = 5900**：相较于稳定版的 3050 更大，回退距离更长
4. **未设置电机速度**：main.c 中未显式调用 `motor_set_speed()`，使用默认速度
5. **USART 指令接收框架已就绪但 main 循环中无指令解析逻辑**：中断接收缓冲区已配置但主循环未处理指令

### 与稳定版的差异
| 对比项 | 本版本（比较输出） | 稳定运行 60min 版 |
|--------|-------------------|-------------------|
| 电机速度设置 | 未显式设置（默认值） | `motor_set_speed(1000)` |
| 限位回退脉冲 | 5900 | 3050 |
| 目标到达处理 | `enable_limit_interrupt()` 在 `motor_start()` 之前 | `enable_limit_interrupt()` 在 `delay_ms(300)` 之后 |
| 运行稳定性 | 未充分验证 | 已验证可稳定运行 60min+ |

## 八、编译与烧录

1. 使用 Keil MDK v5 打开 `F107VCT6.uvprojx` 工程文件
2. 确认 STM32F1xx_DFP 设备包已安装（v2.3.0）
3. 选择 Target：`YS-F1PRO` → `Application/MDK-ARM`
4. 编译（F7），确认 0 Error / 0 Warning
5. 通过 ST-Link V2 连接开发板，下载（F8）并调试

## 九、依赖关系

```
main.c
  ├── bsp.h (系统初始化总入口)
  │   ├── gpio.h/c    ── GPIO 引脚配置
  │   ├── led.h/c     ── LED 状态控制
  │   ├── motor.h/c   ── 电机驱动（PWM/方向/脉冲/限位）
  │   ├── timer.h/c   ── TIM1 时基 + 电机 PWM 定时器
  │   ├── usart.h/c   ── USART3 串口通信
  │   ├── oled.h/c    ── OLED 显示
  │   └── 485.h/c     ── RS485 通信
  ├── stm32f10x.h     ── CMSIS 设备头文件
  └── stm32f10x_it.h/c ── 中断服务函数
```

---

> 本文件为项目工程说明文档。最后更新日期：2026-06-23

---

# STM32F107VCT6 Motor Control & Sensor Acquisition System — Task 1 (Comparison Version)

## 1. Overview

This project implements a **bare-metal (foreground/background) architecture** DC motor control system based on the STM32F107VCT6 MCU. It uses timer-generated PWM pulses to drive a stepper/DC motor, integrated with a temperature sensor, ultrasonic distance sensor, and OLED display. The system supports automatic motor reciprocation, limit switch detection, obstacle detection, and real-time sensor data display via serial output.

This is the **early debugging version of Task 1**, using a bare-metal polling architecture where all business logic executes within the `main()` function's main loop. FreeRTOS source code is included in the project but **not actually enabled** (the scheduler is never started).

## 2. Hardware Platform

| Item | Specification |
|------|--------------|
| **MCU** | STM32F107VCT6 |
| **Core** | ARM 32-bit Cortex-M3 |
| **Clock** | 72 MHz |
| **Flash** | 256 KB |
| **SRAM** | 64 KB |
| **Debugger** | ST-LINK V2 (SWD) |
| **Dev Board** | YS-F1PRO |

### Onboard Peripherals

| Peripheral | Description |
|------------|-------------|
| **Motor Driver** | GPIO + PWM control: direction, start/stop, pulse counting |
| **Limit Switches** | PC13 / PC14, dual-channel, EXTI external interrupt |
| **Temperature Sensor** | ADC analog acquisition |
| **Ultrasonic Distance Sensor** | GPIO-triggered ranging, pulse-width to distance conversion |
| **OLED Display** | I2C 0.96" OLED (SSD1306), 128×64 |
| **USART3** | Serial communication: printf output + interrupt-based command reception |
| **RS485** | Onboard 485 driver (reserved) |
| **LED** | Status indicator: on, off, slow flash, fast flash |

## 3. Software Architecture

### 3.1 Architecture Overview

A single `main()` function with a `while(1)` super-loop handles all business logic. A TIM1 interrupt provides the 1ms system tick. All sensor polling, motor control, LED management, and serial output are executed sequentially within the main loop.

### 3.2 Directory Structure

```
project-root/
├── Drivers/
│   ├── Start/          # Startup files (.s) & CMSIS core
│   └── Library/        # STM32F10x Standard Peripheral Library
├── Inc/
│   ├── bsp/            # BSP headers: 485.h, bsp.h, gpio.h, led.h, motor.h, oled.h, oledfont.h, timer.h, usart.h
│   ├── stm32f10x_conf.h
│   └── stm32f10x_it.h
├── Src/
│   ├── bsp/            # BSP sources: 485.c, bsp.c, gpio.c, led.c, motor.c, oled.c, timer.c, usart.c
│   ├── main.c          # Core business logic
│   └── stm32f10x_it.c  # ISR implementations
├── freertos/           # FreeRTOS source (not enabled in this version)
├── docs/               # Documentation & datasheets
├── F107VCT6.uvprojx    # Keil MDK project
└── F107VCT6.uvoptx     # Keil MDK project options
```

### 3.3 Build Environment

| Item | Value |
|------|-------|
| **IDE** | Keil MDK-ARM v5 |
| **Compiler** | ARM Compiler 5 (AC5) |
| **Device Pack** | Keil::STM32F1xx_DFP v2.3.0 |
| **Preprocessor Defines** | `USE_STDPERIPH_DRIVER`, `STM32F10X_CL` |
| **Debugger** | ST-Link Debugger |

## 4. Core Module Details

### 4.1 Motor Control (`motor.c` / `motor.h`)

Pulse counting + direction control for position management:

- **Speed Control**: Timer PWM frequency adjustment
- **Direction Control**: `motor_change_dir()` switches GPIO level for forward/reverse
- **Position Control**: `motor_set_pulse_count()` sets target pulse count; ISR accumulates pulses; `motor_is_target_reached()` signals arrival
- **Limit Protection** (auto-reciprocation):
  1. PC13 or PC14 EXTI interrupt triggers
  2. Stop motor → delay 300ms → change direction → retract 5900 pulses → clear pulse count → re-enable limit interrupt
- **Obstacle Protection**: Distance < 40 stops motor; requires 3 consecutive readings > 40 to resume

### 4.2 Sensors

**Temperature**: ADC acquisition via `get_tempture()`. Alert threshold: > 500 (triggers LED mode 2).

**Ultrasonic Distance**: GPIO trigger → echo pulse-width measurement → `get_distance()`. Threshold: < 40 triggers obstacle stop.

### 4.3 OLED Display (`oled.c` / `oled.h`)

I2C-driven SSD1306, 128×64 pixels. Displays: row 0/6 for "stop" status, row 2 for distance (`dis:XXXX`), row 4 for temperature (`temp:XXXX`). Font: 8×16 ASCII via `oledfont.h`.

### 4.4 Serial Communication (`usart.c` / `usart.h`)

USART3 with printf redirection via `fputc()`. Interrupt-based reception: `USART3_RxBuffer[]`, `USART3_RxFinished` flag, `USART3_RxCount`. Outputs sensor data every 1 second.

### 4.5 LED Status (`led.c` / `led.h`)

4 modes: Off (0), On (1), Slow flash 1Hz (2 — temperature alert), Fast flash 5Hz (3 — obstacle stop). Timing driven by `sysTick_ms` via TIM1 interrupt.

### 4.6 Timer (`timer.c` / `timer.h`)

TIM1 configured for 1ms interrupts. Maintains `sysTick_ms` (0-5000 cycle). Used for LED timing and periodic sensor sampling.

### 4.7 RS485 (`485.c` / `485.h`)

Onboard RS485 transceiver driver. Direction control via GPIO for RE/DE pins. Reserved for industrial fieldbus communication.

## 5. Main Loop State Machine

Seven sequential checks per iteration:
1. Every 1s: acquire temp/distance → printf → OLED update
2. Limit switch triggered → stop → reverse → retract 5900 pulses → re-enable
3. Target reached → reset pulses → stop → restart
4. Temp > 500 → LED_MODE = 2 (slow flash alert)
5. Distance < 40 → stop motor, LED_MODE = 3; recover after 3× distance > 40
6. LED mode execution (switch-case)
7. Motor state maintenance (`motor_is_runing` flag)

### Key Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Target Pulses | 10000000 | Effectively infinite run |
| Limit Retract | 5900 | Reverse pulses after limit trigger |
| Temp Threshold | 500 | ADC raw value alert |
| Obstacle Threshold | 40 | Distance in cm |
| Recovery Confirm | 3 counts | Consecutive distance > 40 |
| Sampling Period | 1000 ms | 1 Hz sensor acquisition |

## 6. Interrupt Vectors

| Source | ISR | Function |
|--------|-----|----------|
| TIM1_UP | `TIM1_UP_IRQHandler()` | 1ms system tick |
| EXTI (PC13/PC14) | External interrupt | Limit switch trigger |
| USART3 | USART3 interrupt | Serial command reception |

## 7. Version Characteristics

### Key Features
1. **Bare-metal architecture**: All logic in `main()` polling loop, no RTOS scheduling
2. **FreeRTOS not enabled**: `FreeRTOS_Start()` is an empty stub, scheduler never started
3. **Limit retract = 5900**: Larger retract distance than the stable version (3050)
4. **Motor speed not set**: No explicit `motor_set_speed()` call, uses default
5. **Serial command framework ready but unused**: ISR buffers configured, but main loop has no command parsing

### Differences from Stable Version

| Item | This Version | Stable 60min Version |
|------|-------------|---------------------|
| Motor Speed | Not set (default) | `motor_set_speed(1000)` |
| Limit Retract | 5900 | 3050 |
| `enable_limit_interrupt()` timing | Before `motor_start()` | After `delay_ms(300)` |
| Stability | Not verified | Verified 60min+ |

## 8. Build & Flash

1. Open `F107VCT6.uvprojx` in Keil MDK v5
2. Confirm STM32F1xx_DFP v2.3.0 is installed
3. Select Target: `YS-F1PRO` → `Application/MDK-ARM`
4. Build (F7), ensure 0 errors
5. Connect ST-Link V2, flash (F8), debug

## 9. Module Dependencies

```
main.c
  ├── bsp.h (system init entry)
  │   ├── gpio.h/c    ── GPIO configuration
  │   ├── led.h/c     ── LED control
  │   ├── motor.h/c   ── Motor driver (PWM/direction/pulses/limits)
  │   ├── timer.h/c   ── TIM1 tick + motor PWM
  │   ├── usart.h/c   ── USART3 communication
  │   ├── oled.h/c    ── OLED display
  │   └── 485.h/c     ── RS485 communication
  ├── stm32f10x.h     ── CMSIS device header
  └── stm32f10x_it.h/c ── ISR implementations
```

---

> Project engineering documentation. Version: Task 1 Comparison. Last updated: 2026-06-23
