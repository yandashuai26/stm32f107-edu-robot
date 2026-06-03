# STM32F107VCT6 电机控制与传感器采集系统 — 任务2（稳定运行版）

## 一、项目概述

本项目基于 STM32F107VCT6 微控制器和 **FreeRTOS 实时操作系统**，实现了一套多任务并行的直流电机控制系统。相比任务1的裸机前后台架构，本版本将系统功能拆分为四个独立 RTOS 任务并行运行：限位开关处理任务、串口指令控制任务、传感器数据采集任务和通信显示任务。任务间通过 FreeRTOS 消息队列和信号量进行高效的数据传递与事件同步。

本版本是任务2的**基础稳定运行版**，电机运行速度设置为 1000 pps，系统经过充分测试，各任务协同工作稳定可靠。

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
| **电机驱动** | TIM PWM + GPIO | 脉冲控制 + 方向切换 + 启停 |
| **限位开关 1** | PC13 (EXTI) | 电机行程限位检测 |
| **限位开关 2** | PC14 (EXTI) | 电机行程限位检测 |
| **温度传感器** | ADC 通道 | 模拟温度采集 |
| **超声波距离传感器** | GPIO Trig/Echo | 超声波测距 |
| **OLED 显示屏** | I2C | 0.96 寸 SSD1306，128×64 |
| **USART3** | TX/RX | 串口调试输出 + 上位机指令接收 |
| **RS485** | 板载芯片 | 工业总线接口（预留） |
| **LED** | GPIO | 系统状态指示 |

## 三、软件架构

### 3.1 整体架构 — FreeRTOS 多任务模型

```
┌────────────────────────────────────────────────────────────┐
│                       main()                                │
│  bsp_init() → motor_set_speed(1000) → FreeRTOS_Start()     │
│                       │                                     │
│              vTaskStartScheduler()                          │
│                       │                                     │
│         ┌─────────────┼─────────────┐                      │
│         ▼             ▼             ▼                      │
│  ┌────────────┐ ┌──────────┐ ┌────────────┐               │
│  │ 限位任务    │ │ 控制任务  │ │ 采集任务    │               │
│  │ (优先级4)  │ │ (优先级3) │ │ (优先级2)  │               │
│  │            │ │          │ │            │               │
│  │ 信号量等待 │ │ 指令解析  │ │ 温度/距离   │               │
│  │ 去抖+换向  │ │ STOP/STR │ │ 队列发送    │               │
│  │ 冷却期管理 │ │ 临界区    │ │ 100ms      │               │
│  └─────┬──────┘ └────┬─────┘ └─────┬──────┘               │
│        │              │             │                       │
│   LimitSemaphore  USART3_RxBuf  TempQueue/DisQueue         │
│        │              │             │                       │
│        │              │             ▼                       │
│        │              │  ┌────────────────────┐            │
│        │              │  │   通信任务 (优先级1) │            │
│        │              │  │  printf + OLED 显示 │            │
│        │              │  │  80ms 周期          │            │
│        │              │  └────────────────────┘            │
│        │              │                                     │
│   ┌────┴──────────────┴─────┐                              │
│   │      中断服务层          │                              │
│   │  TIM1 → 系统时基         │                              │
│   │  EXTI → 限位触发         │  ──→ xSemaphoreGiveFromISR  │
│   │  USART3 → 指令接收       │  ──→ 缓冲区写入              │
│   └──────────────────────────┘                              │
└────────────────────────────────────────────────────────────┘
```

### 3.2 任务设计方案

| 任务名称 | 优先级 | 栈大小 | 触发方式 | 核心职责 |
|----------|--------|--------|----------|----------|
| `vTaskLimitSwitch` | **4**（最高） | 256 words | 信号量阻塞等待 | 限位开关事件处理、电机换向、冷却期管理 |
| `vTaskControl` | **3** | 256 words | 10ms 周期轮询 | 上位机指令解析（STOP/START）、电机控制 |
| `vTaskCollect` | **2** | 256 words | 100ms 周期 | 温度 + 距离传感器采集、数据入队 |
| `vTaskCommunication` | **1**（最低） | 256 words | 80ms 周期 | 队列数据消费、串口输出、OLED 刷新 |

**优先级设计原则**：安全优先。限位开关涉及电机行程保护，优先级最高（4）；指令控制涉及人为干预，优先级次之（3）；数据采集与通信显示为后台任务，优先级较低（2 和 1）。

### 3.3 任务间通信机制

```
                    ┌──────────────────────────┐
                    │      中断层 (ISR)         │
                    │                          │
                    │  EXTI ISR                │
                    │    └→ xSemaphoreGiveFromISR() → LimitSemaphore → 限位任务
                    │                          │
                    │  USART3 ISR              │
                    │    └→ USART3_RxBuffer[] ───────────────────→ 控制任务
                    └──────────────────────────┘

                    ┌──────────────────────────┐
                    │      任务层               │
                    │                          │
                    │  采集任务                 │
                    │    └→ xQueueSend() ──→ TempQueue ──→ 通信任务
                    │    └→ xQueueSend() ──→ DisQueue  ──→ 通信任务
                    └──────────────────────────┘
```

| 通信对象 | 类型 | 方向 | 用途 |
|----------|------|------|------|
| `LimitSemaphore` | 二进制信号量 | ISR → 限位任务 | 限位事件通知 |
| `USART3_RxBuffer[]` | 全局缓冲区 | ISR → 控制任务 | 串口指令数据 |
| `TempQueue` | 消息队列 (20×uint32) | 采集任务 → 通信任务 | 温度数据传输 |
| `DisQueue` | 消息队列 (20×uint32) | 采集任务 → 通信任务 | 距离数据传输 |

### 3.4 目录结构

```
项目根目录/
├── Drivers/
│   ├── Start/          # 启动文件与 CMSIS 核心
│   └── Library/        # STM32F10x 标准外设库（SPL）
├── Inc/
│   ├── bsp/            # BSP 层头文件
│   │   ├── 485.h / bsp.h / gpio.h / led.h
│   │   ├── motor.h     # 电机驱动接口（含限位/信号量/冷却期）
│   │   ├── oled.h / oledfont.h / timer.h / usart.h
│   ├── stm32f10x_conf.h
│   └── stm32f10x_it.h
├── Src/
│   ├── bsp/            # BSP 层源文件
│   │   ├── motor.c     # 电机驱动核心实现
│   │   ├── 485.c / bsp.c / gpio.c / led.c / oled.c / timer.c / usart.c
│   ├── main.c          # 主函数入口
│   └── stm32f10x_it.c  # 中断服务函数实现
├── freertos/           # FreeRTOS v10.x 内核
│   ├── inc/            # 内核头文件 (FreeRTOS.h, FreeRTOSConfig.h, task.h, queue.h, semphr.h...)
│   ├── src/            # 内核源文件 (tasks.c, queue.c, heap_4.c, port.c...)
│   ├── FreeRTOS_demo.c # RTOS 应用层（任务创建、队列/信号量初始化）
│   └── FreeRTOS_demo.h
├── docs/               # 说明文档与数据手册
├── F107VCT6.uvprojx    # Keil MDK 工程文件
└── F107VCT6.uvoptx     # Keil MDK 工程选项文件
```

### 3.5 编译环境与配置

| 项目 | 说明 |
|------|------|
| **IDE** | Keil MDK-ARM v5 |
| **编译器** | ARM Compiler 5 (AC5) |
| **设备包** | Keil::STM32F1xx_DFP v2.3.0 |
| **RTOS** | FreeRTOS（源码集成在 `freertos/` 目录） |
| **预定义宏** | `USE_STDPERIPH_DRIVER`, `STM32F10X_CL` |
| **内存管理方案** | heap_4.c（首次适应 + 相邻空闲块合并） |
| **调试器** | ST-Link Debugger（SWD 接口） |

## 四、核心功能模块详解

### 4.1 FreeRTOS 应用层 (`FreeRTOS_demo.c`)

#### 系统初始化与任务创建

```
FreeRTOS_Start():
  ├── limit_semaphore_create()          创建限位二进制信号量
  ├── xQueueCreate(20, sizeof(uint32_t)) 创建温度队列 TempQueue
  ├── xQueueCreate(20, sizeof(uint32_t)) 创建距离队列 DisQueue
  ├── xTaskCreate(vTaskLimitSwitch,  256, prio=4)  限位任务
  ├── xTaskCreate(vTaskControl,      256, prio=3)  控制任务
  ├── xTaskCreate(vTaskCollect,      256, prio=2)  采集任务
  ├── xTaskCreate(vTaskCommunication,256, prio=1)  通信任务
  └── vTaskStartScheduler()              启动调度器（永不返回）
```

#### 任务 1：限位开关任务 (`vTaskLimitSwitch`)

```
while(1):
  ┌─ 阻塞等待限位信号量 (portMAX_DELAY)
  ├─ vTaskDelay(50ms)                    ← 机械去抖
  ├─ trigger_pin = get_limit_trigger_pin()
  ├─ if PC13 或 PC14:
  │    motor_reset_pulse_count()         ← 脉冲清零
  │    motor_change_dir()                ← 换向
  │    set_limit_reached_flag()          ← 设置标志
  │    limit_enter_cooldown()            ← 进入冷却期
  │    vTaskDelay(300ms)                 ← 冷却延时
  │    limit_exit_cooldown()             ← 退出冷却期
  └─ 循环
```

**关键技术细节**：
- 限位 ISR 中仅记录触发引脚号并通过 `xSemaphoreGiveFromISR()` 释放信号量，所有业务处理下沉到任务层
- 50ms 去抖延时消除机械开关触点弹跳
- 300ms 冷却期确保电机换向完成前不会再次响应同一限位开关

#### 任务 2：指令控制任务 (`vTaskControl`)

```
while(1):
  ┌─ taskENTER_CRITICAL()               ← 进入临界区
  ├─ 读取 USART3_RxFinished / USART3_RxCount
  └─ taskEXIT_CRITICAL()               ← 退出临界区
  if USART3_RxFinished == 1:
    ┌─ taskENTER_CRITICAL()
    ├─ memcpy 数据到 cmd_buffer
    ├─ 清零 USART3_RxFinished
    ├─ memset 清零接收缓冲区
    └─ taskEXIT_CRITICAL()
    ├─ 剥离 \r \n
    ├─ if "STOP":  motor_stop()  + printf("STOP\r\n")
    └─ if "START": motor_start() + printf("START\r\n")
  vTaskDelay(10ms)
```

**关键技术细节**：
- 临界区保护确保读取接收标志和拷贝缓冲区时不被 USART3 ISR 打断
- 支持指令：`STOP`（停止电机）、`START`（启动电机）
- 指令匹配使用标准 C 库 `strcmp()` 函数

#### 任务 3：传感器采集任务 (`vTaskCollect`)

```
while(1):
  temp = get_tempture()                     ← ADC 温度采集
  dis  = get_distance()                     ← 超声波测距
  xQueueSend(TempQueue, &temp, 0)           ← 非阻塞发送
  xQueueSend(DisQueue,  &dis,  0)
  vTaskDelay(100ms)                         ← 10Hz 采样率
```

**关键技术细节**：
- 采集周期 100ms（10Hz），兼顾实时性与 CPU 负载
- 非阻塞发送（超时参数 = 0）：队列满时直接返回 `errQUEUE_FULL` 并打印警告，不阻塞采集任务

#### 任务 4：通信显示任务 (`vTaskCommunication`)

```
while(1):
  if xQueueReceive(TempQueue, &temp, 0) == pdTRUE:
    printf("temp:%d\r\n", temp)                     ← 串口输出
    OLED_ShowString(行2, "temp:XXXX")               ← OLED 更新
  if xQueueReceive(DisQueue, &dis, 0) == pdTRUE:
    printf("dis:%d\r\n", dis)
    OLED_ShowString(行4, "dis:XXXX")
  vTaskDelay(80ms)
```

**关键技术细节**：
- 非阻塞接收（超时 = 0），有数据则显示，无数据则跳过
- 80ms 处理周期略快于采集周期（100ms），确保队列不积压
- 同时输出到两个通道：串口（远程监控）和 OLED（本地查看）

### 4.2 电机控制模块 (`motor.c` / `motor.h`)

电机驱动的 FreeRTOS 增强功能：

- **限位信号量管理**：
  - `limit_semaphore_create()`：调用 `xSemaphoreCreateBinary()` 创建信号量
  - EXTI ISR 中通过 `xSemaphoreGiveFromISR()` 释放信号量唤醒限位任务
- **冷却期机制**：
  - `limit_enter_cooldown()`：设置冷却标志，抑制期间到达的限位中断
  - `limit_exit_cooldown()`：清除冷却标志，恢复正常响应
  - 冷却期时长 300ms，覆盖电机换向 + 启动的过渡过程
- **触发引脚记录**：
  - ISR 中调用 `record_limit_trigger_pin()` 记录具体触发引脚（13 或 14）
  - 任务中通过 `get_limit_trigger_pin()` 获取，实现 PC13/PC14 的区分处理

### 4.3 传感器采集模块

#### 温度传感器
- **ADC 采集通道**：具体通道号见 `bsp.c` 中 ADC 初始化代码
- **采集函数**：`uint32_t get_tempture(void)` 返回 ADC 转换原始值
- **采集频率**：10Hz（由 `vTaskCollect` 任务 100ms 周期控制）

#### 距离传感器（超声波）
- **工作原理**：Trig 引脚发送 10μs 以上高电平脉冲 → 模块自动发送 8 个 40kHz 方波 → Echo 引脚输出高电平，持续时间与距离成正比
- **测量函数**：`uint32_t get_distance(void)` 返回距离值（单位取决于换算公式，通常为 mm）
- **采集频率**：10Hz（由 `vTaskCollect` 任务控制）

### 4.4 串口通信模块 (`usart.c` / `usart.h`)

- **硬件 USART3**：全双工异步串行通信
- **波特率**：115200 或 9600 bps
- **printf 重定向**：

```c
int fputc(int ch, FILE *f) {
    USART_SendData(USART3, (uint8_t)ch);
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
    return ch;
}
```

- **中断接收框架**：
  - 字节级中断接收，存入 `USART3_RxBuffer[]`
  - 检测到 `\r\n` 时置位 `USART3_RxFinished = 1`
  - 控制任务在临界区内安全读取并清除标志

### 4.5 OLED 显示模块 (`oled.c` / `oled.h`)

- **通信协议**：I2C（硬件 I2C 外设或 GPIO 模拟）
- **驱动芯片**：SSD1306
- **分辨率**：128 × 64 像素
- **显示字体**：8×16 像素 ASCII（`oledfont.h` 字库）
- **显示内容**：
  - 第 2 行：`temp:XXXX`（温度值）
  - 第 4 行：`dis:XXXX`（距离值）
- **更新频率**：由通信任务控制，受队列数据到达频率影响

### 4.6 定时器与中断管理

| 中断源 | 中断服务函数 | 抢占优先级 | 功能 |
|--------|-------------|-----------|------|
| TIM1_UP | `TIM1_UP_IRQHandler()` | 默认 | 1ms 系统时基计数，5s 循环复位 |
| EXTI15_10 | `EXTI15_10_IRQHandler()` | 高 | PC13/PC14 限位检测，记录引脚，释放信号量 |
| USART3 | `USART3_IRQHandler()` | 中 | 串口字节接收，缓冲区管理，帧检测 |

## 五、FreeRTOS 关键配置 (`FreeRTOSConfig.h`)

| 配置项 | 值 | 说明 |
|--------|-----|------|
| `configUSE_PREEMPTION` | 1 | 启用抢占式调度 |
| `configUSE_TIME_SLICING` | 1 | 同优先级时间片轮转 |
| `configTICK_RATE_HZ` | 1000 | 系统节拍 1kHz（1ms tick） |
| `configMAX_PRIORITIES` | 5 或更多 | 支持优先级 0~4 |
| `configMINIMAL_STACK_SIZE` | 128 | 最小任务栈 128 words（512 bytes） |
| `configUSE_MUTEXES` | 1 | 启用互斥量 |
| `configUSE_COUNTING_SEMAPHORES` | 1 | 启用计数信号量 |
| `configSUPPORT_DYNAMIC_ALLOCATION` | 1 | 动态内存分配 |

## 六、主函数流程 (`main.c`)

```
main():
  ├── NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2)  设置中断优先级分组 (2+2)
  ├── bsp_init()                初始化所有外设 (GPIO/TIM/USART/ADC/I2C/OLED/电机/LED)
  ├── delay_ms(100)             等待外设稳定
  ├── motor_set_speed(1000)     设置电机运行速度
  ├── FreeRTOS_Start()          启动 RTOS 调度器
  │   ├── 创建信号量 + 队列
  │   ├── 创建 4 个任务
  │   └── vTaskStartScheduler() ← 从此处进入 RTOS 世界，不再返回
  └── while(1) {}               永远不会执行到此处
```

## 七、版本特点与对比

### 本版本特点

1. **FreeRTOS 多任务架构**：功能拆分为 4 个独立任务，通过抢占式调度保证实时性
2. **电机速度 1000 pps**：标准运行速度，运行平稳
3. **信号量驱动限位处理**：ISR → Semaphore → Task 的事件传递链路，ISR 执行时间极短
4. **消息队列生产者-消费者模式**：采集任务生产数据，通信任务消费数据，解耦采集与显示
5. **串口指令控制**：支持上位机发送 `STOP` / `START` 实时控制电机启停
6. **临界区保护共享资源**：串口接收缓冲区的原子访问保护
7. **冷却期防重复触发机制**：300ms 冷却窗口抑制限位开关重复触发

### 与结构体保存状态版的差异

| 对比项 | 本版本（基础版） | 结构体保存状态版 |
|--------|-----------------|------------------|
| 电机速度 | **1000 pps** | 1500 pps |
| 状态管理 | 分散的全局变量 | **结构体封装** |
| 状态持久化 | 无 | **支持状态结构体保存/恢复** |

### 与任务1（裸机版）的核心差异

| 维度 | 任务1（裸机前后台） | 本版本（FreeRTOS 多任务） |
|------|---------------------|--------------------------|
| **软件架构** | 单 `main()` 循环轮询 | 4 个独立 RTOS 任务并行 |
| **调度方式** | 顺序执行，无优先级 | 抢占式优先级调度 |
| **限位处理** | 主循环中轮询标志位 | 信号量驱动 + 独立高优先级任务 |
| **指令控制** | 仅接收框架（未实现） | 独立任务实时解析 STOP/START |
| **数据流** | 全局变量直接读写 | 消息队列（Queue）FIFO 传递 |
| **CPU 利用率** | 100% 空转忙等 | 任务阻塞时进入空闲任务 |
| **实时性** | 依赖主循环执行速度 | 高优先级任务可抢占低优先级 |
| **可扩展性** | 添加功能需修改主循环 | 添加新任务即可，互不干扰 |

## 八、编译与烧录步骤

1. 使用 Keil MDK-ARM v5 打开工程文件 `F107VCT6.uvprojx`
2. 确认已安装设备包：`Keil::STM32F1xx_DFP` v2.3.0
3. 检查工程目标：`YS-F1PRO`
4. 检查预定义宏：`USE_STDPERIPH_DRIVER,STM32F10X_CL`
5. 检查包含路径：`freertos/inc`、`Inc/`、`Inc/bsp/`
6. 按 **F7** 编译，确保 0 Error / 0 Warning
7. 通过 **ST-Link V2** 连接 YS-F1PRO 开发板（SWD）
8. 按 **F8** 下载固件到 Flash
9. 复位开发板，打开串口助手观察输出
10. 发送 `STOP\r\n` 或 `START\r\n` 测试电机控制指令

## 九、模块依赖关系图

```
main.c
  ├── stm32f10x.h              ← CMSIS 设备定义（寄存器映射）
  ├── stm32f10x_conf.h         ← 标准外设库模块选择
  ├── bsp.h (BSP 统一入口)
  │   ├── gpio.h/c             ← GPIO 初始化与配置
  │   ├── led.h/c              ← LED 控制
  │   ├── motor.h/c            ← 电机驱动（脉冲/方向/限位/信号量/冷却期）
  │   ├── timer.h/c            ← 系统时基定时器
  │   ├── usart.h/c            ← USART3 通信（printf 重定向 + 中断接收）
  │   ├── oled.h/c             ← OLED 显示（I2C + SSD1306）
  │   └── 485.h/c              ← RS485 驱动
  ├── FreeRTOS_demo.h/c        ← RTOS 应用层
  │   └── FreeRTOS 内核 (task.c, queue.c, heap_4.c, port.c...)
  └── stm32f10x_it.h/c         ← 中断服务函数
```

---

> 本文件为项目工程说明文档。版本：任务2 稳定运行版。最后更新日期：2026-06-03

---

# STM32F107VCT6 Motor Control & Sensor Acquisition System — Task 2 (Stable Version)

## 1. Overview

This project implements a **multi-tasking parallel motor control system** based on the STM32F107VCT6 MCU and **FreeRTOS RTOS**. Unlike Task 1's bare-metal architecture, this version splits system functionality into four independent RTOS tasks: limit switch handling, serial command control, sensor data acquisition, and communication display. Tasks communicate via FreeRTOS message queues and semaphores.

This is the **basic stable release of Task 2**, with motor speed set to 1000 pps. All tasks have been thoroughly tested and work together reliably.

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
| Ultrasonic Sensor | GPIO Trig/Echo | Distance measurement |
| OLED | I2C | 0.96" SSD1306, 128×64 |
| USART3 | TX/RX | Debug output + host commands |
| RS485 | Onboard chip | Industrial bus (reserved) |
| LED | GPIO | Status indicator |

## 3. Software Architecture

### 3.1 FreeRTOS Multi-Task Model

Four tasks with preemptive priority scheduling. Priority principle: **safety first** (limit > control > acquisition > communication).

| Task | Priority | Stack | Trigger | Responsibility |
|------|----------|-------|---------|----------------|
| `vTaskLimitSwitch` | 4 (highest) | 256 words | Semaphore (EXTI) | Limit event, direction change, cooldown |
| `vTaskControl` | 3 | 256 words | 10ms polling | Serial command parsing (STOP/START) |
| `vTaskCollect` | 2 | 256 words | 100ms periodic | Sensor acquisition, queue send |
| `vTaskCommunication` | 1 (lowest) | 256 words | 80ms periodic | Queue consume, printf, OLED refresh |

### 3.2 Inter-Task Communication

| Object | Type | Direction | Purpose |
|--------|------|-----------|---------|
| `LimitSemaphore` | Binary semaphore | ISR → Limit Task | Limit event notification |
| `USART3_RxBuffer[]` | Global buffer | ISR → Control Task | Serial command data |
| `TempQueue` | Queue (20×uint32) | Collect → Communication | Temperature data |
| `DisQueue` | Queue (20×uint32) | Collect → Communication | Distance data |

### 3.3 Directory Structure

```
project-root/
├── Drivers/          # CMSIS startup + STM32F10x SPL
├── Inc/bsp/          # BSP headers (485.h, bsp.h, gpio.h, led.h, motor.h, oled.h, timer.h, usart.h)
├── Src/bsp/          # BSP sources + main.c + stm32f10x_it.c
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
| **Memory** | heap_4.c (first-fit + coalescing) |

## 4. Core Module Details

### 4.1 FreeRTOS Application Layer (`FreeRTOS_demo.c`)

**Initialization**: Create binary semaphore → create 2 queues (20×uint32 each) → create 4 tasks → `vTaskStartScheduler()`.

**Task 1 — Limit Switch** (prio 4): Blocks on `LimitSemaphore`. On trigger: 50ms debounce → read trigger pin → reset pulse count → change direction → set flag → enter cooldown (300ms) → exit cooldown. The ISR only records the pin and gives the semaphore; all processing is in the task.

**Task 2 — Control** (prio 3): Polls `USART3_RxFinished` every 10ms inside critical sections. On complete frame: copy buffer → clear flags → strip `\r\n` → `strcmp("STOP")`/`strcmp("START")` → motor action + printf response.

**Task 3 — Collect** (prio 2): Every 100ms: `get_tempture()` + `get_distance()` → non-blocking `xQueueSend()` to TempQueue and DisQueue.

**Task 4 — Communication** (prio 1): Every 80ms: non-blocking `xQueueReceive()` from both queues → `printf()` to serial → `OLED_ShowString()` to display.

### 4.2 Motor Control (`motor.c` / `motor.h`)

FreeRTOS enhancements: semaphore management (`limit_semaphore_create`), cooldown mechanism (300ms window), trigger pin recording (ISR → task). ISR is minimal (record pin + `xSemaphoreGiveFromISR`), all logic in the task.

### 4.3 Sensors

**Temperature**: ADC via `get_tempture()`, 10Hz acquisition rate.  
**Distance**: Ultrasonic Trig/Echo via `get_distance()`, 10Hz acquisition rate.

### 4.4 Serial (`usart.c` / `usart.h`)

USART3 full-duplex. printf via `fputc()` redirection. Interrupt-based reception: byte-level ISR → `USART3_RxBuffer[]` → `\r\n` detection → `USART3_RxFinished = 1`. Control task reads in critical section.

### 4.5 OLED (`oled.c` / `oled.h`)

I2C SSD1306, 128×64, 8×16 ASCII font. Row 2: `temp:XXXX`, Row 4: `dis:XXXX`.

### 4.6 Interrupts

| Source | ISR | Priority | Function |
|--------|-----|----------|----------|
| TIM1_UP | `TIM1_UP_IRQHandler()` | Default | 1ms tick, 5s cycle |
| EXTI15_10 | `EXTI15_10_IRQHandler()` | High | PC13/PC14 limit, give semaphore |
| USART3 | `USART3_IRQHandler()` | Medium | Byte reception, frame detect |

## 5. FreeRTOS Configuration

| Config | Value | Description |
|--------|-------|-------------|
| `configUSE_PREEMPTION` | 1 | Preemptive scheduling |
| `configTICK_RATE_HZ` | 1000 | 1ms tick |
| `configMAX_PRIORITIES` | 5+ | Priorities 0-4 |
| `configMINIMAL_STACK_SIZE` | 128 | 512 bytes |
| `configUSE_MUTEXES` | 1 | Mutex enabled |
| `configSUPPORT_DYNAMIC_ALLOCATION` | 1 | Dynamic allocation |

## 6. Version Characteristics

### Key Features
1. **4-task FreeRTOS architecture** with preemptive priority scheduling
2. **Motor speed 1000 pps** — standard, smooth operation
3. **Semaphore-driven limit handling** — minimal ISR, all logic in task
4. **Producer-consumer pattern** via message queues — decoupled acquisition & display
5. **Serial commands** — STOP/START for real-time motor control
6. **Critical section protection** for shared USART3 buffer
7. **Cooldown mechanism** — 300ms window prevents re-triggering

### vs. Struct-State Version

| Item | This (Basic) | Struct-State Version |
|------|-------------|---------------------|
| Motor Speed | 1000 pps | 1500 pps |
| State Management | Scattered globals | **Struct encapsulation** |
| State Persistence | None | **Struct save/restore** |

### vs. Task 1 (Bare Metal)

| Aspect | Task 1 | This Version |
|--------|--------|-------------|
| Architecture | Single polling loop | 4 RTOS tasks |
| Scheduling | Sequential | Preemptive priority |
| Limit Handling | Poll flags | Semaphore + dedicated task |
| Commands | Framework only | Real-time STOP/START |
| Data Flow | Global variables | Message queues |
| CPU Usage | 100% busy-wait | Blocks when idle |
| Extensibility | Modify main loop | Add new tasks |

## 7. Build & Flash

1. Open `F107VCT6.uvprojx` in Keil MDK-ARM v5
2. Install `Keil::STM32F1xx_DFP` v2.3.0
3. Target: `YS-F1PRO`, defines: `USE_STDPERIPH_DRIVER,STM32F10X_CL`
4. Include paths: `freertos/inc`, `Inc/`, `Inc/bsp/`
5. F7 build, F8 flash via ST-Link V2 (SWD)
6. Serial monitor at 115200/9600 baud
7. Send `STOP\r\n` / `START\r\n` to test

## 8. Module Dependencies

```
main.c
  ├── stm32f10x.h / stm32f10x_conf.h
  ├── bsp.h → gpio, led, motor, timer, usart, oled, 485
  ├── FreeRTOS_demo.h/c → FreeRTOS kernel (task, queue, heap_4, port)
  └── stm32f10x_it.h/c → ISR implementations
```

---

> Project engineering documentation. Version: Task 2 Stable. Last updated: 2026-06-03
