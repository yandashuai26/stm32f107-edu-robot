# 项目架构图集

> **STM32F107VCT6 教育机器人控制系统** — 架构可视化
> 生成日期: 2026-06-16
>
> 📁 Mermaid 实时渲染（下方） | 📁 [SVG 下载（Graphviz 渲染）](diagrams/)

---

## 1. 版本架构演进图

![版本演进](diagrams/01-version-evolution.svg)

```mermaid
graph TB
    subgraph v1["🔵 v1 裸机时代（任务1）"]
        direction TB
        V1A["v1.0-bare-metal-compare<br/>📁 task1-bare-metal-compare<br/>━━━━━━━━━━━━━━━━<br/>架构: 前后台轮询<br/>主循环: while(1)<br/>速度: 未设置<br/>回退: 5900 脉冲<br/>RTOS: 源码存在但未启用<br/>稳定性: 未验证"]
        V1B["v1.1-bare-metal-stable<br/>📁 task2-bare-metal-stable<br/>━━━━━━━━━━━━━━━━<br/>架构: 前后台轮询<br/>主循环: while(1)<br/>速度: 1000 pps<br/>回退: 3050 脉冲<br/>RTOS: 源码存在但未启用<br/>稳定性: ✅ 60min+ 验证"]
    end

    subgraph v2["🟢 v2 FreeRTOS 4任务（任务2）"]
        direction TB
        V2A["v2.0-freertos-4task<br/>📁 task3-freertos-4task<br/>━━━━━━━━━━━━━━━━<br/>架构: FreeRTOS 4任务<br/>任务: 限位/控制/采集/通信<br/>速度: 1000 pps<br/>IPC: 信号量 + 2队列<br/>状态: 全局变量"]
        V2B["v2.1-freertos-struct-state<br/>📁 task4-freertos-struct-state<br/>━━━━━━━━━━━━━━━━<br/>架构: FreeRTOS 4任务<br/>任务: 限位/控制/采集/通信<br/>速度: 1500 pps<br/>IPC: 信号量 + 2队列<br/>状态: motor_t 结构体"]
    end

    subgraph v3["🔴 v3 FreeRTOS 7任务（任务3）"]
        direction TB
        V3["v3.0-freertos-7task<br/>📁 task5-freertos-7task<br/>━━━━━━━━━━━━━━━━<br/>架构: FreeRTOS 7任务<br/>任务: 限位/异常/控制/采集/<br/>       中点到达/通信/LED<br/>速度: 1000 pps<br/>IPC: 4信号量 + 4队列<br/>状态: motor_t 结构体<br/>特色: 斜坡加减速 + 中点定位"]
    end

    V1A -->|"修复稳定性<br/>修正参数"| V1B
    V1B -->|"引入 FreeRTOS<br/>拆分为4任务"| V2A
    V2A -->|"引入结构体<br/>状态管理"| V2B
    V2B -->|"新增3任务<br/>全信号量驱动"| V3

    style V1A fill:#e3f2fd,stroke:#1565c0
    style V1B fill:#bbdefb,stroke:#1565c0
    style V2A fill:#c8e6c9,stroke:#2e7d32
    style V2B fill:#a5d6a7,stroke:#2e7d32
    style V3 fill:#ffcdd2,stroke:#c62828
```

---

## 2. 最终版（v3.0）详细任务架构图

```mermaid
graph TB
    subgraph INT["⚡ 中断层 (ISR)"]
        EXTI["EXTI15_10<br/>PC13/PC14 限位开关<br/>优先级: 0,0"]
        TIM5["TIM5<br/>PWM脉冲+计数<br/>优先级: 1,2"]
        USART3["USART3<br/>串口接收<br/>优先级: 1,2"]
        TIM1["TIM1_UP<br/>1ms 系统时基<br/>优先级: 3,2"]
    end

    subgraph IPC["📨 信号量（事件通知）"]
        direction LR
        LS["LimitSemaphore<br/>━━━━━━━━━━<br/>二进制信号量<br/>EXTI → 限位任务"]
        ASS["AbnormalSituation<br/>Semaphore<br/>━━━━━━━━━━<br/>二进制信号量<br/>采集任务 → 异常任务"]
        USARTS["USART3_RxSemaphore<br/>━━━━━━━━━━<br/>二进制信号量<br/>USART3 → 控制任务"]
        MTRS["MotorTargetReached<br/>Semaphore<br/>━━━━━━━━━━<br/>二进制信号量<br/>TIM5 → 中点任务"]
    end

    subgraph IPC_Q["📬 消息队列（数据传输）"]
        direction LR
        TQ["TempQueue<br/>uint32 × 20<br/>温度数据"]
        DQ["DisQueue<br/>uint32 × 20<br/>距离数据"]
        ASDQ["AbnormalSituation<br/>DisQueue<br/>uint32 × 20<br/>异常距离数据"]
        LMQ["LedModeQueue<br/>uint32 × 20<br/>LED模式指令"]
    end

    subgraph TASKS["🎯 FreeRTOS 任务层"]
        subgraph PRIO4["优先级 4（最高）"]
            TLS["vTaskLimitSwitch<br/>━━━━━━━━━━━━<br/>栈: 256 words<br/>触发: LimitSemaphore<br/>━━━━━━━━━━━━<br/>1. 等待信号量<br/>2. 50ms 去抖<br/>3. 读取触发引脚<br/>4. 清零脉冲+换向<br/>5. 300ms 冷却期"]
            TAS["vTaskAbnormalSituation<br/>━━━━━━━━━━━━<br/>栈: 256 words<br/>触发: AbnormalSituationSemaphore<br/>━━━━━━━━━━━━<br/>1. 等待信号量<br/>2. 读取距离<br/>3. dis < 50mm → 停机<br/>4. 连续3次 > 50mm → 恢复"]
        end

        subgraph PRIO3["优先级 3"]
            TCTL["vTaskControl<br/>━━━━━━━━━━━━<br/>栈: 256 words<br/>触发: USART3_RxSemaphore<br/>━━━━━━━━━━━━<br/>1. 等待信号量<br/>2. 临界区拷贝指令<br/>3. STOP → motor_stop()<br/>4. START → motor_start()"]
        end

        subgraph PRIO2["优先级 2"]
            TCOL["vTaskCollect<br/>━━━━━━━━━━━━<br/>栈: 256 words<br/>周期: 100ms<br/>━━━━━━━━━━━━<br/>1. RS485 读温度<br/>2. RS485 读距离<br/>3. 数据入3个队列<br/>4. 释放异常信号量"]
            TMR["vTaskMotorReached<br/>━━━━━━━━━━━━<br/>栈: 256 words<br/>触发: MotorTargetReached<br/>Semaphore<br/>━━━━━━━━━━━━<br/>1. 等待信号量<br/>2. 目标到达处理<br/>3. 显示 'Reached'<br/>4. 重设 10000000 脉冲"]
        end

        subgraph PRIO1["优先级 1（最低）"]
            TCOM["vTaskCommunication<br/>━━━━━━━━━━━━<br/>栈: 256 words<br/>周期: 200ms<br/>━━━━━━━━━━━━<br/>1. 读取 TempQueue<br/>2. 读取 DisQueue<br/>3. OLED 显示<br/>4. printf 串口输出"]
            TLED["vTaskLedMode<br/>━━━━━━━━━━━━<br/>栈: 256 words<br/>触发: LedModeQueue<br/>━━━━━━━━━━━━<br/>模式0: 关闭<br/>模式1: 常亮<br/>模式2: 700ms 慢闪<br/>模式3: 100ms 快闪"]
        end
    end

    subgraph HARDWARE["🔌 硬件外设"]
        MOTOR["🔄 电机 UIM240<br/>PWM+DIR+ENA"]
        OLED["📺 SSD1306 OLED<br/>128×64 I2C"]
        LED["💡 LED PE3/PE4<br/>状态指示"]
        DYP["📏 DYP-A02 超声波<br/>RS485 Modbus"]
        TEMP["🌡️ 温度传感器<br/>RS485 Modbus"]
        HOST["💻 上位机<br/>USART3 指令"]
    end

    %% 连接关系
    EXTI -->|"xSemaphoreGiveFromISR<br/>记录触发引脚"| LS
    TIM5 -->|"xSemaphoreGiveFromISR"| MTRS
    USART3 -->|"xSemaphoreGiveFromISR"| USARTS

    LS -->|"xSemaphoreTake"| TLS
    ASS -->|"xSemaphoreTake"| TAS
    USARTS -->|"xSemaphoreTake"| TCTL
    MTRS -->|"xSemaphoreTake"| TMR

    TCOL -->|"xQueueSend"| TQ
    TCOL -->|"xQueueSend"| DQ
    TCOL -->|"xQueueSend"| ASDQ
    TCOL -->|"xSemaphoreGive"| ASS
    TAS -->|"xQueueSend"| LMQ

    TQ -->|"xQueueReceive"| TCOM
    DQ -->|"xQueueReceive"| TCOM
    ASDQ -->|"xQueueReceive"| TAS
    LMQ -->|"xQueueReceive"| TLED

    TLS -->|"控制方向/脉冲"| MOTOR
    TCTL -->|"STOP/START"| MOTOR
    TCOM -->|"I2C 显示"| OLED
    TCOM -->|"printf"| HOST
    TLED -->|"GPIO 控制"| LED
    TCOL -->|"RS485 读取"| DYP
    TCOL -->|"RS485 读取"| TEMP
    HOST -->|"指令发送"| USART3

    style INT fill:#fff3e0,stroke:#e65100
    style IPC fill:#e8eaf6,stroke:#283593
    style IPC_Q fill:#e8f5e9,stroke:#1b5e20
    style PRIO4 fill:#ffebee,stroke:#b71c1c
    style PRIO3 fill:#fff8e1,stroke:#f57f17
    style PRIO2 fill:#e3f2fd,stroke:#0d47a1
    style PRIO1 fill:#f3e5f5,stroke:#4a148c
    style HARDWARE fill:#e0f2f1,stroke:#004d40
```

---

## 3. 三代架构对比图

```mermaid
graph LR
    subgraph GEN1["🏗️ v1 裸机前后台（任务1）"]
        direction TB
        G1_MAIN["main()"]
        G1_INIT["bsp_init()<br/>GPIO/TIM/USART<br/>ADC/I2C/OLED"]
        G1_LOOP["while(1) 超级循环<br/>━━━━━━━━━━━━━━<br/>① 1s定时采集温度/距离<br/>② 限位触发 → 停→换向→回退<br/>③ 目标到达 → 重置→重启<br/>④ 温度>500 → LED慢闪告警<br/>⑤ 距离<40 → 停机+快闪<br/>⑥ LED模式切换<br/>⑦ 电机状态保持"]
        G1_ISR["TIM1_UP_IRQHandler<br/>1ms 系统时基<br/>sysTick_ms 0~5000"]
        G1_EXEC["执行特征<br/>━━━━━━━━<br/>❌ 顺序执行<br/>❌ 无优先级<br/>❌ 100% CPU忙等<br/>❌ 实时性差"]

        G1_MAIN --> G1_INIT --> G1_LOOP
        G1_ISR -.->|"1ms中断"| G1_LOOP
        G1_LOOP --> G1_EXEC
    end

    subgraph GEN2["⚙️ v2 FreeRTOS 4任务（任务2）"]
        direction TB
        G2_MAIN["main() → FreeRTOS_Start()"]
        G2_TASKS["4个独立任务<br/>━━━━━━━━━━━━━━<br/>① vTaskLimitSwitch (prio4)<br/>    信号量触发 | 50ms去抖 | 300ms冷却<br/>② vTaskControl (prio3)<br/>    10ms轮询 | STOP/START指令<br/>③ vTaskCollect (prio2)<br/>    100ms周期 | 温度+距离采集<br/>④ vTaskCommunication (prio1)<br/>    80ms周期 | printf+OLED显示"]
        G2_IPC["IPC 机制<br/>━━━━━━━━<br/>LimitSemaphore × 1<br/>TempQueue × 1<br/>DisQueue × 1"]
        G2_EXEC["执行特征<br/>━━━━━━━━<br/>✅ 抢占式调度<br/>✅ 优先级保证<br/>✅ 阻塞时释放CPU<br/>✅ 实时性好"]

        G2_MAIN --> G2_TASKS
        G2_TASKS --> G2_IPC
        G2_IPC --> G2_EXEC
    end

    subgraph GEN3["🚀 v3 FreeRTOS 7任务（任务3）"]
        direction TB
        G3_MAIN["main() → FreeRTOS_Start()"]
        G3_TASKS["7个独立任务<br/>━━━━━━━━━━━━━━<br/>① vTaskLimitSwitch (prio4)<br/>② vTaskAbnormalSituation (prio4)<br/>③ vTaskControl (prio3)<br/>④ vTaskCollect (prio2)<br/>⑤ vTaskMotorReached (prio2)<br/>⑥ vTaskCommunication (prio1)<br/>⑦ vTaskLedMode (prio1)"]
        G3_IPC["IPC 机制<br/>━━━━━━━━<br/>4个信号量<br/>4个消息队列<br/>全事件驱动"]
        G3_FEAT["新增特性<br/>━━━━━━━━<br/>✅ 障碍物自动检测恢复<br/>✅ 中点定位(3050脉冲)<br/>✅ LED 4模式独立控制<br/>✅ 斜坡加减速(1Hz/脉冲)<br/>✅ motor_t 结构体管理<br/>✅ 🔥 零轮询 全信号量驱动"]
        G3_EXEC["执行特征<br/>━━━━━━━━<br/>✅ 抢占式调度<br/>✅ 安全优先(限位>异常>控制)<br/>✅ 扩展性强(新增任务即可)<br/>✅ 实时性最佳"]

        G3_MAIN --> G3_TASKS
        G3_TASKS --> G3_IPC
        G3_IPC --> G3_FEAT
        G3_FEAT --> G3_EXEC
    end

    GEN1 -->|"引入 FreeRTOS"| GEN2
    GEN2 -->|"扩展任务+全事件驱动"| GEN3

    style GEN1 fill:#e3f2fd,stroke:#1565c0
    style GEN2 fill:#c8e6c9,stroke:#2e7d32
    style GEN3 fill:#ffcdd2,stroke:#c62828
```

---

## 4. 模块依赖关系图（最终版）

```mermaid
graph TB
    subgraph APP["📱 应用层"]
        MAIN["main.c<br/>━━━━━━━━<br/>bsp_init()<br/>motor_set_speed(1000)<br/>FreeRTOS_Start()"]
        FREERTOS_DEMO["FreeRTOS_demo.c<br/>━━━━━━━━<br/>7任务创建<br/>信号量/队列初始化<br/>vTaskStartScheduler()"]
    end

    subgraph BSP["🔧 BSP 驱动层"]
        BSP_H["bsp.h<br/>━━━━━━<br/>总入口头文件<br/>汇总所有模块"]
        BSP_C["bsp.c<br/>━━━━━━<br/>统一外设初始化<br/>delay_ms()"]
    end

    subgraph MODULES["📦 功能模块"]
        GPIO["gpio.c/h<br/>━━━━━━━<br/>PC13/PC14 EXTI<br/>LimitSemaphore 创建"]
        MOTOR["motor.c/h<br/>━━━━━━━<br/>motor_t 结构体<br/>斜坡加减速<br/>信号量管理<br/>冷却期机制"]
        LED["led.c/h<br/>━━━━━━━<br/>PE3/PE4 控制<br/>4种闪烁模式"]
        TIMER["timer.c/h<br/>━━━━━━━<br/>TIM1 1ms时基<br/>TIM5 PWM配置"]
        USART["usart.c/h<br/>━━━━━━━<br/>USART3 printf<br/>中断接收<br/>RxSemaphore"]
        OLED["oled.c/h<br/>━━━━━━━<br/>SSD1306 I2C<br/>128×64 显示<br/>oledfont.h 字库"]
        RS485["485.c/h<br/>━━━━━━━<br/>RS485收发<br/>Modbus传感器通信"]
    end

    subgraph RTOS["⏱️ FreeRTOS 内核"]
        RTOS_INC["freertos/inc/<br/>━━━━━━━━<br/>FreeRTOS.h<br/>task.h<br/>queue.h<br/>semphr.h<br/>FreeRTOSConfig.h"]
        RTOS_SRC["freertos/src/<br/>━━━━━━━━<br/>tasks.c<br/>queue.c<br/>heap_4.c<br/>port.c<br/>timers.c"]
    end

    subgraph HAL["🏭 STM32 HAL/驱动层"]
        CMSIS["Drivers/Start/<br/>━━━━━━━━<br/>stm32f10x.h<br/>core_cm3.c<br/>system_stm32f10x.c<br/>startup_stm32f10x_cl.s"]
        SPL["Drivers/Library/<br/>━━━━━━━━<br/>stm32f10x_gpio.c<br/>stm32f10x_tim.c<br/>stm32f10x_usart.c<br/>stm32f10x_rcc.c<br/>stm32f10x_i2c.c<br/>stm32f10x_dma.c<br/>stm32f10x_exti.c<br/>stm32f10x_adc.c<br/>..."]
    end

    MAIN --> BSP_H
    MAIN --> FREERTOS_DEMO
    FREERTOS_DEMO --> BSP_H
    FREERTOS_DEMO --> RTOS_INC

    BSP_H --> GPIO
    BSP_H --> MOTOR
    BSP_H --> LED
    BSP_H --> TIMER
    BSP_H --> USART
    BSP_H --> OLED
    BSP_H --> RS485

    BSP_C --> BSP_H

    MODULES --> SPL
    MODULES --> CMSIS
    RTOS_SRC --> CMSIS

    style APP fill:#e8f5e9,stroke:#2e7d32
    style BSP fill:#fff3e0,stroke:#ef6c00
    style MODULES fill:#e3f2fd,stroke:#1565c0
    style RTOS fill:#f3e5f5,stroke:#6a1b9a
    style HAL fill:#eceff1,stroke:#546e7a
```

---

## 5. IPC 数据流图（最终版）

```mermaid
graph LR
    subgraph ISR["中断源"]
        I1["EXTI<br/>限位触发"]
        I2["USART3<br/>指令接收"]
        I3["TIM5<br/>脉冲到达"]
        I4["vTaskCollect<br/>100ms 软件触发"]
    end

    subgraph SEM["信号量（事件通知）"]
        S1["LimitSemaphore"]
        S2["USART3_RxSemaphore"]
        S3["MotorTargetReachedSemaphore"]
        S4["AbnormalSituationSemaphore"]
    end

    subgraph QUEUE["消息队列（数据传输）"]
        Q1["TempQueue<br/>温度"]
        Q2["DisQueue<br/>距离"]
        Q3["AbnormalSituationDisQueue<br/>异常距离"]
        Q4["LedModeQueue<br/>LED模式"]
    end

    subgraph TASK["消费任务"]
        T1["vTaskLimitSwitch<br/>限位处理"]
        T2["vTaskControl<br/>指令解析"]
        T3["vTaskMotorReached<br/>中点定位"]
        T4["vTaskAbnormalSituation<br/>异常检测"]
        T5["vTaskCommunication<br/>数据展示"]
        T6["vTaskLedMode<br/>LED控制"]
    end

    I1 -->|"xSemaphoreGiveFromISR"| S1
    I2 -->|"xSemaphoreGiveFromISR"| S2
    I3 -->|"xSemaphoreGiveFromISR"| S3
    I4 -->|"xSemaphoreGive"| S4

    S1 -->|"xSemaphoreTake"| T1
    S2 -->|"xSemaphoreTake"| T2
    S3 -->|"xSemaphoreTake"| T3
    S4 -->|"xSemaphoreTake"| T4

    I4 -->|"xQueueSend"| Q1
    I4 -->|"xQueueSend"| Q2
    I4 -->|"xQueueSend"| Q3
    T4 -->|"xQueueSend"| Q4

    Q1 -->|"xQueueReceive"| T5
    Q2 -->|"xQueueReceive"| T5
    Q3 -->|"xQueueReceive"| T4
    Q4 -->|"xQueueReceive"| T6

    style ISR fill:#ffebee,stroke:#c62828
    style SEM fill:#e8eaf6,stroke:#283593
    style QUEUE fill:#e8f5e9,stroke:#1b5e20
    style TASK fill:#fff8e1,stroke:#f57f17
```

---

## 6. 任务优先级与触发方式总览

| 任务 | 优先级 | 栈 | 触发方式 | 周期/超时 | 职责 |
|------|:---:|-----|----------|-----------|------|
| vTaskLimitSwitch | 4 | 256W | LimitSemaphore (阻塞) | portMAX_DELAY | 限位响应/换向/冷却 |
| vTaskAbnormalSituation | 4 | 256W | AbnormalSituationSemaphore (阻塞) | portMAX_DELAY | 障碍物检测/自动恢复 |
| vTaskControl | 3 | 256W | USART3_RxSemaphore (阻塞) | portMAX_DELAY | STOP/START 指令 |
| vTaskCollect | 2 | 256W | vTaskDelay 周期延时 | 100ms | 传感器采集/数据分发 |
| vTaskMotorReached | 2 | 256W | MotorTargetReachedSemaphore (阻塞) | portMAX_DELAY | 中点定位(3050脉冲) |
| vTaskCommunication | 1 | 256W | vTaskDelay 周期延时 | 200ms | OLED+printf 输出 |
| vTaskLedMode | 1 | 256W | LedModeQueue (非阻塞) | 0 (立即返回) | 4模式LED控制 |

---

> **作者**: Tom &nbsp;|&nbsp; **项目**: 毕业设计 — 教育机器人控制系统
