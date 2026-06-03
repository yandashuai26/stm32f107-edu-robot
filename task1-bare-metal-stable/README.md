# STM32F107VCT6 电机控制与传感器采集系统 — 任务1（稳定运行60min版）

## 一、项目概述

本项目基于 STM32F107VCT6 微控制器，实现了一套**前后台（裸机）架构**的直流电机控制系统。系统通过定时器产生 PWM 脉冲驱动步进/直流电机，同时集成温度传感器、超声波距离传感器和 OLED 显示屏，实现了电机的自动往复运动、限位检测、障碍物检测以及传感器数据的实时显示与串口输出。

本版本为**任务1的稳定运行版本**，已经过 60 分钟以上的长时间运行验证，系统在持续运行过程中无死机、无异常复位、无状态混乱等问题。架构上仍采用裸机轮询方式，所有业务逻辑集中在 `main()` 函数的主循环中顺序执行。

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
│  │  - 电机初始化, motor_set_speed(1000)   │  │
│  │  - 设置目标脉冲数, 启动电机            │  │
│  └───────────────────────────────────────┘  │
│  ┌───────────────────────────────────────┐  │
│  │         while(1) 主循环               │  │
│  │  - 定时采集温度/距离 (每1s)           │  │
│  │  - 限位开关处理                       │  │
│  │  - 目标到达处理                       │  │
│  │  - 温度超限告警 (>500)                │  │
│  │  - 障碍物检测 (<40cm)                 │  │
│  │  - LED 模式控制                       │  │
│  │  - 电机运行状态保持                    │  │
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
│   ├── FreeRTOS_demo.c # FreeRTOS 入口（空实现，调度器未启动）
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
| **优化等级** | 默认（建议 -O1） |
| **调试器** | ST-Link Debugger |

## 四、核心功能模块详解

### 4.1 电机控制模块 (`motor.c` / `motor.h`)

电机采用**脉冲计数 + 方向控制**的方式实现精确位置控制：

- **速度控制**：`motor_set_speed(1000)` 设置电机运行速度（1000 pps）
- **方向控制**：通过 `motor_change_dir()` 切换方向 GPIO 电平实现正反转
- **位置控制**：`motor_set_pulse_count()` 设定目标脉冲数，定时器中断累计脉冲计数值，到达预设值后触发 `motor_is_target_reached()` 标志
- **限位保护机制**（自动往复）：
  1. 限位开关（PC13 或 PC14）触发 EXTI 外部中断
  2. 进入 `motor_is_limit_reached()` 分支
  3. 暂时关闭限位中断（`disable_limit_interrupt()`）：防止回退过程中再次触发
  4. 停止电机，OLED 显示 "stop"
  5. 延时 300ms 采集并更新传感器数据
  6. 电机换向（`motor_change_dir()`）
  7. 设定回退脉冲数 3050
  8. 启动电机 → 清除脉冲计数 → 延时 300ms
  9. 重新使能限位中断（`enable_limit_interrupt()`）
  10. 恢复正常运行状态
- **障碍物保护机制**：
  - 距离传感器检测到距离 < 40（约 40cm）时立即停止电机
  - 设置 `motor_stopped_by_obstacle = 1` 标志
  - 连续 3 次检测到距离 > 40 才清除障碍物标志并恢复电机运行
  - 单次距离 ≤ 40 即重置确认计数器，防止误恢复

### 4.2 传感器采集模块

#### 温度传感器
- 通过 ADC 外设采集模拟信号
- `get_tempture()` 返回当前 ADC 温度采样值
- **告警阈值**：温度值 > 500 触发温度告警
- 触发后 LED 切换到模式 2（慢闪 1Hz）
- 温度降至 ≤ 500 后自动恢复 LED 模式 1（常亮）

#### 距离传感器（超声波）
- GPIO 触发超声波模块发射 40kHz 脉冲
- 捕获回波信号，测量高电平脉冲宽度
- 根据声速换算为距离值（单位：cm 或 mm）
- `get_distance()` 返回当前距离值
- **障碍物阈值**：距离 < 40 触发障碍物保护

### 4.3 OLED 显示模块 (`oled.c` / `oled.h`)

- 通信接口：I2C
- 驱动芯片：SSD1306
- 分辨率：128 × 64 像素
- 显示布局：
  - **行 0**：目标到达时的 "stop" 提示（5 字符宽，显示后自动清除）
  - **行 2**：距离值实时显示（格式：`dis:XXXX`，16 像素字体）
  - **行 4**：温度值实时显示（格式：`temp:XXXX`，16 像素字体）
  - **行 6**：限位触发时的 "stop" 提示（4 字符宽，显示后自动清除）
- 字库：`oledfont.h` 提供标准 8×16 ASCII 字符点阵数据

### 4.4 串口通信模块 (`usart.c` / `usart.h`)

- **USART3** 作为主通信接口
- **printf 重定向**：重写 `fputc()` 将标准输出映射到 USART3，方便格式化调试输出
- **中断接收框架**：
  - 全局缓冲区：`USART3_RxBuffer[]`
  - 接收索引：`USART3_RxIndex`
  - 帧完成标志：`USART3_RxFinished`
  - 接收计数：`USART3_RxCount`
- **输出格式**：`temp:XXX\r\ndis:XXX\r\n`（每秒输出一次）

### 4.5 LED 状态指示模块 (`led.c` / `led.h`)

系统通过 4 种 LED 模式直观反映运行状态：

| 模式 | 宏值 | 行为 | 触发条件 | 含义 |
|------|------|------|----------|------|
| 熄灭 | 0 | LED 完全熄灭 | 初始化前 | 系统待机 |
| 常亮 | 1 | LED 持续点亮 | 正常运行 | 系统工作正常 |
| 慢闪 | 2 | ON 500ms / OFF 500ms（1Hz） | 温度 > 500 | 温度超限告警 |
| 快闪 | 3 | ON 100ms / OFF 100ms（5Hz） | 距离 < 40 | 障碍物检测停止 |

LED 闪烁通过 `sysTick_ms % 周期 < 占空比` 方式实现，其中 `sysTick_ms` 由 TIM1 中断每 1ms 递增。

### 4.6 定时器模块 (`timer.c` / `timer.h`)

- **TIM1 系统时基**：
  - 配置为向上计数模式
  - 每 1ms 产生一次更新中断
  - 中断服务函数：`TIM1_UP_IRQHandler()`
  - 维护全局变量 `sysTick_ms`（0 ~ 5000 循环）
  - 为 LED 闪烁、定时采集提供时间基准

### 4.7 RS485 模块 (`485.c` / `485.h`)

- 板载 RS485 收发器驱动
- 方向控制：通过 GPIO 控制 MAX485 等芯片的 RE/DE 引脚
- 为工业现场总线（Modbus RTU 等）通信提供硬件支持

## 五、主循环业务逻辑（详细状态机）

```
系统上电
   │
   ▼
NVIC 优先级分组 → bsp_init() → delay_ms(100)
   │
   ▼
motor_set_speed(1000) → motor_set_pulse_count(10000000) → motor_start()
   │
   ▼
┌──────────────── while(1) 主循环 ────────────────────┐
│                                                       │
│  ┌─ 时间触发 (sysTick_ms == 1000) ────────────────┐  │
│  │  采集温度 → 采集距离 → printf 输出 → OLED 更新 │  │
│  └────────────────────────────────────────────────┘  │
│                                                       │
│  ┌─ 限位触发 (motor_is_limit_reached()) ──────────┐  │
│  │  关限位中断 → 停电机 → 显示stop(行6)           │  │
│  │  → 延时300ms → 采数据 → 更新OLED(行2,行4)     │  │
│  │  → 清除行6 → 换向 → 回退3050脉冲               │  │
│  │  → 启电机 → 清脉冲计数 → 延时300ms             │  │
│  │  → 使能限位中断 → 恢复运行标志                  │  │
│  └────────────────────────────────────────────────┘  │
│                                                       │
│  ┌─ 目标到达 (motor_is_target_reached()) ─────────┐  │
│  │  清脉冲计数 → 重设目标10000000 → 关限位中断    │  │
│  │  → 停电机 → 显示stop(行0) → 延时300ms          │  │
│  │  → 采数据 → 更新OLED(行2,行4) → 延时300ms      │  │
│  │  → 清除行0 → 启电机 → 延时300ms                │  │
│  │  → 使能限位中断 → 恢复运行标志                  │  │
│  └────────────────────────────────────────────────┘  │
│                                                       │
│  ┌─ 温度超限 (temp > 500) ────────────────────────┐  │
│  │  LED_MODE = 2 (慢闪)                           │  │
│  │  temp ≤ 500 且 LED_MODE==2 时恢复 LED_MODE=1   │  │
│  └────────────────────────────────────────────────┘  │
│                                                       │
│  ┌─ 障碍物检测 (dis < 40 && !stopped) ────────────┐  │
│  │  停电机 → 重设目标 → LED_MODE=3 (快闪)         │  │
│  │  → motor_stopped_by_obstacle=1                 │  │
│  │  → obstacle_clear_count=0 → motor_is_runing=0  │  │
│  └────────────────────────────────────────────────┘  │
│                                                       │
│  ┌─ 障碍物恢复 (stopped && 连续3次dis>40) ────────┐  │
│  │  启电机 → LED_MODE=1 → 清除所有障碍物标志      │  │
│  │  → motor_is_runing=1                           │  │
│  └────────────────────────────────────────────────┘  │
│                                                       │
│  ┌─ LED 模式执行 (switch-case) ───────────────────┐  │
│  │  case 0: OFF    case 1: ON                     │  │
│  │  case 2: 1Hz闪  case 3: 5Hz闪                  │  │
│  └────────────────────────────────────────────────┘  │
│                                                       │
│  ┌─ 电机状态保持 ─────────────────────────────────┐  │
│  │  motor_is_runing==0 → motor_stop()             │  │
│  │  motor_is_runing==1 → GPIO_ResetBits + start   │  │
│  └────────────────────────────────────────────────┘  │
└───────────────────────────────────────────────────────┘
```

### 关键参数汇总

| 参数 | 本版本值 | 说明 |
|------|----------|------|
| 电机速度 | **1000 pps** | `motor_set_speed(1000)` 显式设置 |
| 目标脉冲数 | 10000000 | 极大值模拟无限运行 |
| 限位回退脉冲 | **3050** | 限位触发后反向回退的精确脉冲数 |
| 温度告警阈值 | 500 | ADC 原始值 |
| 障碍物距离阈值 | 40 | 单位 cm（或 mm，取决于传感器） |
| 障碍物恢复确认 | 3 次 | 连续 3 次采样距离 > 40 |
| 数据采集周期 | 1000 ms | 每秒一次 |
| 限位去抖延时 | 300 ms | 限位触发后稳定等待时间 |

## 六、中断向量分配

| 中断源 | 中断服务函数 | 优先级 | 功能描述 |
|--------|-------------|--------|----------|
| TIM1_UP | `TIM1_UP_IRQHandler()` | 默认 | 1ms 系统时基，驱动 sysTick_ms |
| EXTI (PC13) | EXTI15_10_IRQHandler | 配置 | 限位开关 1，触发电机换向 |
| EXTI (PC14) | EXTI15_10_IRQHandler | 配置 | 限位开关 2，触发电机换向 |
| USART3 | USART3_IRQHandler | 配置 | 串口中断接收指令 |
| DMA | DMA 相关中断 | 配置 | 电机脉冲 DMA 传输 |

## 七、版本对比与改进点

### 相较于 "比较输出版" 的关键改进

| 改进项 | 比较输出版 (未调试) | 本稳定版 | 改进效果 |
|--------|---------------------|----------|----------|
| 电机速度 | 未设置（默认值） | `motor_set_speed(1000)` | 速度可控，运行更平稳 |
| 限位回退脉冲 | 5900 | **3050** | 回退距离更精确，减少机械冲击 |
| `enable_limit_interrupt()` 位置 | 在 `motor_start()` 之前调用（含被注释掉的冗余代码） | 在 `delay_ms(300)` 之后统一调用 | 时序更合理，避免回退期间误触发 |
| 目标到达处理 | 限位中断使能逻辑混乱（有注释残留） | 清理了注释，逻辑清晰 | 代码可维护性提升 |
| 长时间运行稳定性 | 未验证 | **已通过 60min+ 持续运行测试** | 可靠性验证通过 |

### 与任务2（FreeRTOS版）的架构差异

| 维度 | 本版本（任务1 裸机） | 任务2（FreeRTOS） |
|------|----------------------|-------------------|
| 调度方式 | 前后台轮询 | FreeRTOS 抢占式调度 |
| 任务划分 | 单主循环全部处理 | 4 个独立任务（限位/控制/采集/通信） |
| 数据传递 | 全局变量直接读写 | FreeRTOS 队列（Queue） |
| 限位处理 | 轮询检查标志位 | 信号量（Semaphore）+ 独立任务 |
| 指令处理 | 仅框架（未实现） | 独立指令解析任务（STOP/START） |
| CPU 利用率 | 100% 忙等 | 任务阻塞时释放 CPU |

## 八、编译与烧录步骤

1. 使用 Keil MDK-ARM v5 打开工程文件 `F107VCT6.uvprojx`
2. 确认已安装设备包 `Keil::STM32F1xx_DFP.2.3.0`
3. 检查工程 Target 配置：`YS-F1PRO` → `Application/MDK-ARM`
4. 确认预定义宏：`USE_STDPERIPH_DRIVER` 和 `STM32F10X_CL` 已添加
5. 按 **F7** 编译工程，确保 0 Error / 0 Warning
6. 通过 ST-Link V2 连接 YS-F1PRO 开发板
7. 按 **F8** 下载固件到 Flash
8. 复位开发板，观察 LED 状态与 OLED 显示

## 九、模块依赖关系图

```
main.c (核心业务逻辑)
  │
  ├── stm32f10x.h              ← CMSIS 设备定义
  ├── stm32f10x_conf.h         ← 标准外设库配置
  ├── stm32f10x_it.h / .c      ← 中断服务函数
  │
  └── bsp.h (BSP 总入口)
      ├── gpio.h/c             ← GPIO 引脚配置与初始化
      ├── led.h/c              ← LED 状态控制（4种模式）
      ├── motor.h/c            ← 电机驱动核心
      │   ├── PWM 生成（TIM 通道）
      │   ├── 方向控制（GPIO）
      │   ├── 脉冲计数（TIM 编码器/中断）
      │   └── 限位中断管理（EXTI/NVIC）
      ├── timer.h/c            ← 系统时基定时器
      │   └── TIM1 1ms 中断
      ├── usart.h/c            ← USART3 通信
      │   ├── printf 重定向
      │   └── 中断接收缓冲区
      ├── oled.h/c             ← OLED 显示驱动
      │   ├── I2C 通信（GPIO 模拟或硬件 I2C）
      │   ├── SSD1306 命令控制
      │   └── oledfont.h 字库
      └── 485.h/c              ← RS485 驱动（备用）
```

---

> 本文件为项目工程说明文档。版本：任务1 稳定运行 60min 版。最后更新日期：2026-06-03

---

# STM32F107VCT6 Motor Control & Sensor Acquisition System — Task 1 (Stable 60min Version)

## 1. Overview

This project implements a **bare-metal (foreground/background) architecture** DC motor control system based on the STM32F107VCT6 MCU. It uses timer-generated PWM pulses to drive a stepper/DC motor, integrated with a temperature sensor, ultrasonic distance sensor, and OLED display. The system supports automatic motor reciprocation, limit switch detection, obstacle detection, and real-time sensor data display via serial output.

This is the **stable release of Task 1**, verified to run continuously for **60+ minutes** without crashes, unexpected resets, or state corruption. It uses the same bare-metal polling architecture where all business logic executes within `main()`'s main loop, but with several critical improvements over the comparison version.

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
| **Motor Driver** | GPIO + PWM: direction, start/stop, pulse counting |
| **Limit Switches** | PC13 / PC14, dual-channel, EXTI external interrupt |
| **Temperature Sensor** | ADC analog acquisition |
| **Ultrasonic Distance Sensor** | GPIO-triggered ranging, pulse-width to distance |
| **OLED Display** | I2C 0.96" OLED (SSD1306), 128×64 |
| **USART3** | Serial: printf output + interrupt-based command reception |
| **RS485** | Onboard 485 driver (reserved) |
| **LED** | Status indicator: on, off, slow flash, fast flash |

## 3. Software Architecture

### 3.1 Overview

Single `main()` with `while(1)` super-loop. TIM1 interrupt provides 1ms system tick. Key difference from comparison version: `motor_set_speed(1000)` is explicitly called, limit retract is 3050 pulses (more precise), and `enable_limit_interrupt()` timing is corrected.

### 3.2 Directory Structure

```
project-root/
├── Drivers/          # Startup files + STM32F10x Standard Peripheral Library
├── Inc/bsp/          # BSP headers: 485.h, bsp.h, gpio.h, led.h, motor.h, oled.h, oledfont.h, timer.h, usart.h
├── Src/bsp/          # BSP sources: 485.c, bsp.c, gpio.c, led.c, motor.c, oled.c, timer.c, usart.c
├── Src/main.c        # Core business logic
├── freertos/         # FreeRTOS source (not enabled)
├── docs/             # Documentation & datasheets
├── F107VCT6.uvprojx  # Keil MDK project
└── F107VCT6.uvoptx   # Keil MDK project options
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

- **Speed Control**: `motor_set_speed(1000)` — explicit 1000 pps setting
- **Direction Control**: `motor_change_dir()` — GPIO level toggle
- **Position Control**: `motor_set_pulse_count()` + ISR pulse accumulation + `motor_is_target_reached()`
- **Limit Protection** (auto-reciprocation):
  1. PC13/PC14 EXTI triggers → `motor_is_limit_reached()`
  2. `disable_limit_interrupt()` — prevent re-trigger during retract
  3. Stop motor, OLED "stop" on row 6
  4. Delay 300ms, acquire sensors, update OLED rows 2/4
  5. `motor_change_dir()`, set retract 3050 pulses
  6. Start motor, clear pulse count, delay 300ms
  7. `enable_limit_interrupt()`, resume normal operation
- **Obstacle Protection**: Distance < 40 → stop + flag; 3 consecutive > 40 readings to resume

### 4.2 Sensors

**Temperature**: ADC via `get_tempture()`. Threshold > 500 triggers LED mode 2 (1Hz slow flash). Auto-recovery when ≤ 500.

**Ultrasonic Distance**: GPIO trigger → echo pulse measurement → `get_distance()`. Threshold < 40 triggers obstacle protection.

### 4.3 OLED Display (`oled.c` / `oled.h`)

I2C SSD1306, 128×64. Row 0: target-reached "stop", Row 2: `dis:XXXX`, Row 4: `temp:XXXX`, Row 6: limit-triggered "stop". Font: 8×16 ASCII via `oledfont.h`.

### 4.4 Serial Communication (`usart.c` / `usart.h`)

USART3 with printf redirection. Interrupt reception: `USART3_RxBuffer[]`, `USART3_RxFinished`, `USART3_RxCount`. Outputs `temp:XXX\r\ndis:XXX\r\n` every second.

### 4.5 LED Status (`led.c` / `led.h`)

| Mode | Value | Behavior | Trigger |
|------|-------|----------|---------|
| Off | 0 | LED off | Standby |
| On | 1 | LED on | Normal operation |
| Slow Flash | 2 | 1Hz (500ms on/off) | Temp > 500 |
| Fast Flash | 3 | 5Hz (100ms on/off) | Distance < 40 |

### 4.6 Timer (`timer.c` / `timer.h`)

TIM1: 1ms interrupts, maintains `sysTick_ms` (0-5000 cycle). Drives LED timing and periodic sensor sampling.

## 5. Main Loop State Machine

1. **Time-triggered** (sysTick_ms == 1000): acquire temp + distance → printf → OLED update
2. **Limit triggered**: disable limit IRQ → stop → show stop (row 6) → delay 300ms → acquire data → OLED update → clear row 6 → change direction → retract 3050 → start → clear pulses → delay 300ms → enable limit IRQ
3. **Target reached**: clear pulses → reset target 10000000 → disable limit IRQ → stop → show stop (row 0) → delay 300ms → acquire data → OLED update → delay 300ms → clear row 0 → start → delay 300ms → enable limit IRQ
4. **Temp > 500**: LED_MODE = 2 (slow flash); recover when temp ≤ 500
5. **Distance < 40 && !stopped**: stop → LED_MODE = 3 → set obstacle flag → clear confirm counter
6. **Obstacle recovery**: 3 consecutive > 40 readings → restart → LED_MODE = 1 → clear flags
7. **LED mode execution** (switch-case)
8. **Motor state maintenance** (`motor_is_runing` flag)

### Key Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Motor Speed | **1000 pps** | Explicitly set |
| Target Pulses | 10000000 | Effectively infinite |
| Limit Retract | **3050** | Precise reverse pulse count |
| Temp Threshold | 500 | ADC raw value |
| Obstacle Threshold | 40 | Distance units |
| Recovery Confirm | 3 counts | Consecutive > 40 |
| Sampling Period | 1000 ms | 1 Hz |
| Debounce Delay | 300 ms | Post-limit stabilization |

## 6. Interrupt Vectors

| Source | ISR | Priority | Function |
|--------|-----|----------|----------|
| TIM1_UP | `TIM1_UP_IRQHandler()` | Default | 1ms system tick |
| EXTI (PC13) | EXTI15_10_IRQHandler | Configured | Limit switch 1 |
| EXTI (PC14) | EXTI15_10_IRQHandler | Configured | Limit switch 2 |
| USART3 | USART3_IRQHandler | Configured | Serial command reception |
| DMA | DMA IRQ | Configured | Motor pulse DMA |

## 7. Key Improvements over Comparison Version

| Improvement | Comparison Version | This Stable Version |
|-------------|-------------------|---------------------|
| Motor Speed | Not set (default) | `motor_set_speed(1000)` |
| Limit Retract | 5900 | **3050** (more precise) |
| `enable_limit_interrupt()` timing | Before `motor_start()` (with dead code) | After `delay_ms(300)` (clean) |
| Code quality | Commented-out remnants | Cleaned up |
| Long-run stability | Not verified | **Verified 60min+** |

### Architecture Difference vs Task 2 (FreeRTOS)

| Aspect | This Version (Bare Metal) | Task 2 (FreeRTOS) |
|--------|--------------------------|-------------------|
| Scheduling | Polling loop | Preemptive priority |
| Task division | Single loop | 4 independent tasks |
| Data passing | Global variables | Message queues |
| Limit handling | Poll flags | Semaphore + dedicated task |
| Command handling | Framework only | Dedicated parsing task |
| CPU utilization | 100% busy-wait | Releases CPU when blocked |

## 8. Build & Flash

1. Open `F107VCT6.uvprojx` in Keil MDK-ARM v5
2. Confirm `Keil::STM32F1xx_DFP.2.3.0` installed
3. Target: `YS-F1PRO` → `Application/MDK-ARM`
4. Confirm defines: `USE_STDPERIPH_DRIVER,STM32F10X_CL`
5. F7 build, ensure 0 errors
6. ST-Link V2 → YS-F1PRO
7. F8 flash, reset, observe LED + OLED

## 9. Module Dependencies

```
main.c
  ├── stm32f10x.h              ← CMSIS device header
  ├── stm32f10x_conf.h         ← Peripheral library config
  ├── stm32f10x_it.h/c         ← ISR implementations
  └── bsp.h (BSP entry)
      ├── gpio.h/c             ← GPIO config
      ├── led.h/c              ← LED control (4 modes)
      ├── motor.h/c            ← Motor driver (PWM/dir/pulse/limit)
      ├── timer.h/c            ← TIM1 1ms tick
      ├── usart.h/c            ← USART3 (printf + RX buffer)
      ├── oled.h/c             ← OLED (I2C/SSD1306)
      └── 485.h/c              ← RS485 driver
```

---

> Project engineering documentation. Version: Task 1 Stable 60min. Last updated: 2026-06-03
