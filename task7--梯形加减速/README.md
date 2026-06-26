# 任务7 — 梯形加减速版

基于 STM32F107VCT6 的教育机器人控制系统最终版本，在任务6宏定义OC翻转基础上引入梯形加减速算法，并将整个电机控制 API 重构为 PascalCase 风格，对外采用**单位制距离控制**（0.01圈为单位），替代原始脉冲计数方式。

## 版本概述

| 项目 | 说明 |
|------|------|
| **架构** | FreeRTOS 实时多任务 (7 任务) |
| **电机速度** | 100 单位/秒 (1圈/秒 = 1600 Hz) |
| **加减速** | 梯形加减速 (可配置加速度/减速度) |
| **状态管理** | 结构体 (`motor_t`) + 状态机 (`MotorState`) |
| **IPC 机制** | 二进制信号量 + 消息队列 |
| **脉冲输出** | TIM5 OC 翻转模式 |
| **障碍物检测** | 超声波测距 < 50mm 触发停机，连续 3 次 > 50mm 自动恢复 |
| **LED 模式** | 4 种模式（关闭/常亮/慢闪/快闪），独立任务控制 |

## 与任务6的主要区别

| 对比项 | 任务6 (宏定义OC翻转) | 任务7 (梯形加减速) |
|--------|---------------------|-------------------|
| 加减速算法 | 线性步进 (1Hz/脉冲) | **梯形加减速** (ACCEL/RUN/DECEL 状态机) |
| 距离单位 | 脉冲数 | **0.01圈** (对外单位) |
| 速度单位 | Hz (脉冲频率) | **0.01圈/秒** (对外单位) |
| 函数命名 | `motor_set_speed()` 小写下划线 | **`MotorSetSpeed()` PascalCase** |
| 初始化函数 | `bsp_init()` / `FreeRTOS_Start()` | **`BspInit()` / `vFreeRtosStart()`** |
| 结构体字段 | `target_pulses`, `current_hz`, `running` | **`total_steps`, `PULSE`, `run_state` (MotorState 枚举)** |
| 加速度设置 | 无 | **`MotorSetAccel()` / `MotorSetDecel()`** |
| 信号量命名 | `MotorTargetReachedSemaphore` | **`motor_target_reached_semaphore`** (小写) |

## 梯形加减速原理

电机运行分为四个阶段，形成梯形速度曲线:

```
速度
  ↑
  │      ┌──────────────┐
  │     /│   匀速(RUN)   │\
  │    / │              │ \
  │   /加速(ACCEL)      \ 减速(DECEL)
  │  /   │              │   \
  │ /    │              │    \
  └──────┴──────────────┴──────→ 步数
  0    accel_limit    decel_start   total_steps
```

- **加速阶段 (ACCEL)**: 从 MOTOR_MIN_HZ 逐步加速至目标速度
- **匀速阶段 (RUN)**: 以目标速度匀速运行
- **减速阶段 (DECEL)**: 到达 decel_start 位置后开始减速
- **停止 (STOP)**: 减速至 MOTOR_MIN_HZ 后停止

### 状态机

```c
typedef enum {
    MOTOR_STOP = 0,    // 停止
    MOTOR_ACCEL,       // 加速
    MOTOR_RUN,         // 匀速
    MOTOR_DECEL,       // 减速
    MOTOR_WAIT,        // 等待(到位后暂停)
} MotorState;
```

## 梯形加减速实现详解

### 1. 基本物理量定义

设步进电机参数: `spr` = 每圈脉冲数, `ft` = 定时器频率 (9MHz), `α` = 步距角

| 物理量 | 符号 | 公式 | 单位 |
|--------|------|------|------|
| 步距角 | α | 2π / spr | rad/step |
| 位置 | θ | n × α = n | rad (或 step) |
| 速度 | ω | α / Δt = α × ft / ci | rad/sec (或 step/sec) |

其中 `ci` 为两个脉冲之间的定时器计数值, `Δt = ci / ft` 为脉冲间隔时间。

### 2. 恒定加速度下的脉冲间隔推导

设加速度为 `ω̇` (恒定), 从静止开始加速:

**运动学方程:**

```
速度:  ω(t) = ω̇ × t
位置:  θ(t) = ½ × ω̇ × t² = n × α
```

**第 n 个脉冲的总时间:**

由 `θ = nα = ½ω̇t²` 得:

```
tn = √(2nα / ω̇)
```

**第 n 个脉冲的定时器计数值 (脉冲间隔):**

```
cn = tn+1 - tn = (1/ft) × √(2α/ω̇) × (√(n+1) - √n)
```

**第 1 个脉冲的定时器计数值:**

```
c0 = (1/ft) × √(2α/ω̇)
```

**递推关系:**

```
cn = c0 × (√(n+1) - √n)
```

### 3. 麦克劳林展开简化 (避免开方运算)

直接计算 `√(n+1) - √n` 需要开方运算, 在嵌入式中太耗时。使用麦克劳林公式展开:

```
√(1 ± 1/n) = 1 ± 1/(2n) - 1/(8n²) + O(1/n³)
```

对 `cn/c(n-1)` 进行变换:

```
cn/c(n-1) = [√(n+1) - √n] / [√n - √(n-1)]
           = √n × [√(1+1/n) - 1] / [√n × (1 - √(1-1/n))]
           ≈ (4n - 1) / (4n + 1)
```

**加速递推公式:**

```
cn = c(n-1) × (4n - 1) / (4n + 1)
   = c(n-1) - 2 × c(n-1) / (4n + 1)
```

**减速递推公式 (对称推导):**

```
cn = c(n-1) × (4n + 1) / (4n - 1)
   = c(n-1) + 2 × c(n-1) / (4n - 1)
```

### 4. 误差修正系数 0.676

代入 `n=1` 验证: 理论值 `c1/c0 = √2 - 1 ≈ 0.4142`, 近似值 `(4×1-1)/(4×1+1) = 3/5 = 0.6`

存在 **0.4485 的偏差**。解决方案: 将 `c0` 乘以修正系数 **0.676** 来消除系统误差。

### 5. 整数化处理 (放大 100 倍)

由于计算中可能出现很多浮点数, 不利于效率提升, 将所有参数放大 100 倍:

```
由 ω = α × ft / c, 得 c = α × ft / ω

定义: A_T_x100 = α × ft × 100    (ft × α 的 100 倍)

当 ω = speed (最大速度) 时, 对应最小脉冲间隔:
min_delay = c = A_T_x100 / speed
```

### 6. 关键宏定义

| 宏 | 公式 | 说明 |
|----|------|------|
| `T1_FREQ` | 0.676 × ft / 100 | 初始频率常数 (含误差修正系数) |
| `A_SQ` | 2α × 10^10 | 加速度平方辅助常量 (放大 10^10 倍) |
| `ALPHA` | 2π / spr | 每步弧度 |

**初始脉冲间隔 c0:**

```
c0 = T1_FREQ × √(A_SQ / accel) / 100
```

### 7. 预计算梯形参数

已知: `step` (总步数), `accel` (加速度), `decel` (减速度), `speed` (目标速度)

待求: `max_s_lim`, `accel_lim`, `decel_val`

**加速到最大速度所需步数:**

```
max_s_lim = speed² / (2α × accel × 100)
```

推导: 由 `½ω̇t² = nα` 和 `ωn = ω̇ × tn` 得 `nω̇ = ωn² / (2α)`

分母乘 100 是为了平衡放大倍数。

**减速开始前的步数 (无最大速度限制时):**

```
accel_lim = step × decel / (accel + decel)
```

推导: 由 `n1 = (n2 + n1) × ω2 / (ω1 + ω2)` 得到

**实际减速步数:**

```
如果 max_s_lim < accel_lim (梯形, 可达最大速度):
    decel_val = max_s_lim × accel / decel

如果 max_s_lim > accel_lim (三角形, 加速受限于减速开始):
    decel_val = step - accel_lim
```

### 8. 临界点分析

根据 `v1² - v0² = 2as`, 总步数关系:

```
n_total = n_accel + n_decel + n_s

n_accel = speed² / (2 × accel)    (加速步数)
n_decel = speed² / (2 × decel)    (减速步数)
```

令 `n_s = 0` (无匀速段), 得最小步数:

```
n_min = speed² / (2 × accel) + speed² / (2 × decel)
```

当 `step < n_min` 时, 电机无法达到目标速度, 曲线退化为三角形。

### 9. ISR 中的实时脉冲间隔计算

每次新一步的时间间隔计算结果包括商数和余数, 为提高精度余数保留并包含在下一次计算中。

`rest` 为余数, 首次计算 `rest = 0`, `new_rest` 为保存除不尽的余数参与下一次计算。

**加速阶段 (nth step):**

```
new_step_delay = step_delay - (2 × step_delay + rest) / (4 × accel_count + 1)
new_rest = (2 × step_delay + rest) mod (4 × accel_count + 1)
```

**减速阶段 (nth step):**

```
new_step_delay = step_delay + (2 × step_delay + rest) / (4 × decel_val - 1)
new_rest = (2 × step_delay + rest) mod (4 × decel_val - 1)
```

**OC 翻转说明:**

TIM5 使用 OC 翻转模式, PA0 每翻转 2 次产生 1 个完整脉冲。因此 ISR 中的 `step_delay` 是**半周期** (即完整脉冲间隔的一半)。

## 单位制说明

对外接口统一使用 0.01圈 作为距离单位:

| 概念 | 公式 | 示例 |
|------|------|------|
| 脉冲 → 单位 | `MOTOR_PULSES_TO_UNIT(p)` = p / 16 | 1600脉冲 = 100单位 (1圈) |
| 单位 → 脉冲 | `MOTOR_UNIT_TO_PULSES(u)` = u × 16 | 175单位 = 2800脉冲 (1.75圈) |
| Hz → 速度 | `MOTOR_HZ_TO_SPEED(h)` = h / 16 | 1600Hz = 100单位 (1圈/秒) |
| 速度 → Hz | `MOTOR_SPEED_TO_HZ(s)` = s × 16 | 100单位 = 1600Hz |

## API 对照表

| 功能 | 任务6 (小写下划线) | 任务7 (PascalCase) |
|------|-------------------|-------------------|
| 初始化 | `bsp_init()` | `BspInit()` |
| 启动RTOS | `FreeRTOS_Start()` | `vFreeRtosStart()` |
| 延时 | `delay_ms()` | `DelayMs()` |
| 电机初始化 | `motor_init()` | `MotorInit()` |
| 设速度 | `motor_set_speed(hz)` | `MotorSetSpeed(speed)` — 参数: 0.01圈/秒 |
| 启动 | `motor_start()` | `MotorStart()` |
| 停止 | `motor_stop()` | `MotorStop()` |
| 换向 | `motor_change_dir()` | `MotorChangeDir()` |
| 设目标 | `motor_set_pulse_count(n)` | `MotorSetTargetUnit(unit)` — 参数: 0.01圈 |
| 重置计数 | `motor_reset_pulse_count()` | `MotorResetUnitCount()` |
| 目标到达 | `motor_is_target_reached()` | `MotorIsTargetReached()` |
| 设加速度 | 无 | `MotorSetAccel(a)` — 参数: 0.01圈/s² |
| 设减速度 | 无 | `MotorSetDecel(d)` — 参数: 0.01圈/s² |
| 温度采集 | `get_tempture()` | `GetTempture()` |
| 距离采集 | `get_distance()` | `GetDistance()` |
| LED开 | `LED_ON()` | `LedOn()` |
| LED关 | `LED_OFF()` | `LedOff()` |
| OLED显示 | `OLED_ShowString()` | `OledShowString()` |
| 限位创建 | `limit_semaphore_create()` | `LimitSemaphoreCreate()` |
| 串口创建 | `usart3_semaphore_create()` | `Usart3SemaphoreCreate()` |
| 电机信号量 | `motor_target_semaphore_create()` | `vMotorTargetSemaphoreCreate()` |

## 电机状态结构体

```c
typedef struct {
    /* 脉冲与位置 */
    uint32_t step_count;        // 当前已走步数(绝对值), ISR中递增
    uint32_t total_steps;       // 总目标步数(绝对值)

    /* 速度控制: 脉冲半周期的定时器计数值 */
    uint32_t PULSE;             // 当前脉冲半周期 (step_delay/2)
    uint32_t min_pulse;         // 最小脉冲周期(对应最大速度)
    uint32_t user_speed_hz;     // 用户设定速度 (内部: Hz)

    /* 梯形加减速参数 */
    uint32_t accel_count;       // 加速阶段公式计数器
    uint32_t accel_limit;       // 加速阶段总步数
    uint32_t decel_start;       // 开始减速的步进位置
    uint32_t decel_val;         // 剩余减速阶段步数
    uint32_t accel;             // 加速度 (脉冲/s²)
    uint32_t decel;             // 减速度 (脉冲/s²)

    /* 状态标志 */
    uint8_t  run_state;         // MotorState: STOP/ACCEL/RUN/DECEL/WAIT
    uint8_t  dir;               // 方向: 0=反向 1=正向
    uint8_t  target_reached;    // 目标到达标志
} motor_t;
```

## main.c 初始化流程

```c
int main(void)
{
    BspInit();
    DelayMs(100);

    MotorSetAccel(100);           // 加速度: 100 单位/s²
    MotorSetDecel(100);           // 减速度: 100 单位/s²
    MotorSetSpeed(100);           // 速度: 100 单位/秒 (1圈/秒)
    MotorSetTargetUnit(500);      // 目标: 500 单位 (5圈)
    MotorStart();

    vFreeRtosStart();

    while (1) { }
}
```

## 任务架构

```
优先级 4:  vTaskLimitSwitch       限位开关处理 (信号量触发)
优先级 4:  vTaskAbnormalSituation  异常情况处理 (障碍物检测与恢复)
优先级 3:  vTaskControl            串口指令解析 (信号量阻塞, STOP/START)
优先级 2:  vTaskCollect            传感器数据采集 (温度 + 距离)
优先级 2:  vTaskMotorReached       中点到达检测 (信号量触发)
优先级 1:  vTaskCommunication      数据显示 (OLED + 串口)
优先级 1:  vTaskLedMode            LED 模式控制
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

## 消息队列

| 队列名称 | 元素类型 | 容量 | 生产者 | 消费者 |
|----------|----------|------|--------|--------|
| `temp_queue` | uint32_t | 20 | vTaskCollect | vTaskCommunication |
| `dis_queue` | uint32_t | 20 | vTaskCollect | vTaskCommunication |
| `abnormal_situation_dis_queue` | uint32_t | 20 | vTaskCollect | vTaskAbnormalSituation |
| `led_mode_queue` | uint32_t | 20 | vTaskAbnormalSituation, vFreeRtosStart | vTaskLedMode |

## 信号量

| 信号量 | 类型 | 释放者 | 获取者 |
|--------|------|--------|--------|
| `limit_semaphore` | 二进制 | EXTI ISR | vTaskLimitSwitch |
| `abnormal_situation_semaphore` | 二进制 | vTaskCollect | vTaskAbnormalSituation |
| `usart3_rx_semaphore` | 二进制 | USART3 ISR | vTaskControl |
| `motor_target_reached_semaphore` | 二进制 | TIM5 CC1 ISR | vTaskMotorReached |

## 编译与烧录

### 环境要求

- **Keil MDK-ARM v5** + ARM Compiler 5 (AC5)
- **STM32F1xx_DFP v2.3.0** 设备包
- **ST-LINK V2** 调试器

### 步骤

1. 打开 `task7--梯形加减速/F107VCT6.uvprojx`
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
│   ├── led.h       → LedOn() / LedOff()
│   ├── gpio.h      → LimitSemaphoreCreate() / GetLimitTriggerPin()
│   ├── motor.h     → motor_t + MotorState + 梯形加减速 + 单位制API
│   ├── timer.h     → TIM1 1ms 时基
│   ├── usart.h     → USART3 printf + Usart3SemaphoreCreate()
│   ├── 485.h       → USART1 RS485 GetTempture() / GetDistance()
│   └── oled.h      → OledShowString() SSD1306 OLED
└── FreeRTOS_demo.h
    ├── FreeRTOS.h
    ├── task.h
    ├── queue.h
    └── semphr.h
```

## 版本演进

本版本 (任务7) 相对于任务6 的主要变更:

| 新增 | 说明 |
|------|------|
| 梯形加减速 | ACCEL → RUN → DECEL → STOP 状态机 |
| `MotorSetAccel()` / `MotorSetDecel()` | 可配置加速度/减速度 |
| `MotorState` 枚举 | STOP/ACCEL/RUN/DECEL/WAIT 五状态 |
| 单位制距离 | 对外使用 0.01圈 作为距离单位 |
| 单位制速度 | 对外使用 0.01圈/秒 作为速度单位 |
| PascalCase API | 所有函数和变量重命名为 PascalCase/snake_case |
| `usqrt()` | 整数平方根 (Newton-Raphson), 避免 math.h |
| 变量名英文化 | `USART3_RxBuffer` → `usart3_rx_buffer` 等 |

---

> **作者**: Tom
> **项目**: 毕业设计 — 教育机器人控制系统
> **最后更新**: 2026-06-23
