# FluidNC CNC 激光雕刻集成设计

## 概述

在小智 ESP32-S3 项目中集成激光雕刻运动控制功能。将现有文字→G-code→UART外发的流程，改为文字→G-code→本地运动控制器→GPIO步进脉冲+激光PWM，实现单芯片自主雕刻。

## 当前状态

- **项目**：xiaozhi-esp32，ESP-IDF 框架，ESP32-S3 芯片
- **已有代码**：[`main/boards/CNC/gcode_controller.h`](../../main/boards/CNC/gcode_controller.h) — `KanjiVGController` 类，将文字转为 G-code（G0/G1/M3/M5），通过 UART2 发往外部激光雕刻机
- **工作范围**：42×42mm 雕刻区域
- **硬件配置**：参考 FluidNC config.yaml（A4988 步进驱动，106.666 steps/mm，激光 PWM 5000Hz）

## 方案选择

选择**自行实现精简版运动控制器**，参考 FluidNC 核心算法但不移植整个项目。理由：
- 只需 G0/G1/M3/M5 四个指令
- 2轴 + 激光，无 Z 轴
- 42×42mm 小范围
- FluidNC 为通用 CNC 设计，核心代码数万行，引入成本远高于自研

## 架构

### 模块划分

```
KanjiVGController::GenerateGcode()
        │  std::string gcode_text
        ▼
GCodeParser::Parse()          → std::vector<GCodeCommand>
        │
        ▼
MotionController::Execute()   → 按序处理，G0/G1 入 Planner
        │
        ▼
Planner::PlanBufferLine()     → plan_block_t (步数/方向/速度曲线)
        │
        ▼
Stepper::PrepBuffer()         → segment_t (恒定速度微段 + AMASS)
        │
        ▼
Stepper::pulse_func() [ISR]   → GPIO step/dir + PWM
```

### 新增文件

全部位于 [`main/boards/CNC/`](../../main/boards/CNC/)：

| 文件 | 职责 |
|---|---|
| `gcode_controller.h` | 已有，文字→G-code 生成 |
| `font_data.h` | 已有，汉字/ASCII 字库数据 |
| `motion_controller.h` | 运动控制主入口，协调各模块 |
| `gcode_parser.h` | G-code 解析（G0/G1/M3/M5/G21/G90） |
| `planner.h` | 速度规划器，参考 FluidNC 双通算法 |
| `stepper.h` | 段生成器 + Bresenham ISR + AMASS |
| `stepping_engine.h` | GPTimer GPIO 脉冲硬件层 |

### 集成方式

- `gcode_controller.h` 中 `GenerateGcode()` 不再通过 UART 发送，改为调用 `MotionController::Execute(gcode)`
- MCP 工具 `self.engraving.engrave_text` 接口不变
- 引脚：释放 GPIO.19/20（原 UART2），改为步进控制

## 核心算法（参考 FluidNC 源码）

### 1. 运动规划（参考 Planner.cpp）

FluidNC 使用**反向+正向双通速度规划**：

**反向遍历**：从最后一个 block 向前，计算每个 block 的最大入口速度（受加速度和距离约束）：
```
entry_speed_sqr = MIN(max_entry_speed_sqr, 2 * accel * millimeters)
```

**正向遍历**：从第一个 block 向后，用加速度修正反向计算的结果，同时标记"最优停止计算点"以优化重复计算。

**连接点速度**：用向心加速度近似计算转角速度限制，使用 `sin(θ/2)` 半角恒等式避免三角函数运算。

### 2. 段生成器（参考 Stepper.cpp prep_buffer）

将 planner block 拆分为约 50ms 的恒定速度微段：

- **梯形加减速**：RAMP_ACCEL → RAMP_CRUISE → RAMP_DECEL
- 每个段计算：步数 `n_step`、定时器周期 `isrPeriod`、AMASS 级别
- 使用浮点步数舍入补偿，保证累积步数精确

### 3. Bresenham 直线插补（参考 Stepper.cpp pulse_func）

ISR 中执行的整数算法，无浮点累积误差：

```cpp
for (axis = X; axis < n_axis; axis++) {
    counter[axis] += steps[axis];
    if (counter[axis] > step_event_count) {
        set_bit(step_outbits, axis);  // 输出步进脉冲
        counter[axis] -= step_event_count;
    }
}
```

### 4. AMASS 自适应平滑（参考 Stepper.cpp）

在低速时通过位左移增加 Bresenham 分辨率，消除非主导轴的步进脉冲不均匀问题：
- Level 0：基准 Bresenham，每 ISR tick 1步
- Level 1：位左移1位，2个 ISR tick 完成1步，非主导轴可在中间 tick 步进
- Level N：位左移 N 位，2^N 个 tick 完成1步

### 5. 步进时序（参考 Stepping.cpp Timed 引擎）

```
方向引脚设置 → dir_delay_us (1µs) → 步进引脚拉高 → pulse_us (10µs) → 步进引脚拉低
```

使用 ESP32-S3 GPTimer 产生精确中断，1MHz 计数频率（1µs 精度）。

## 引脚规划

| 功能 | GPIO | 来源 |
|---|---|---|
| X 步进脉冲 | 21 | FluidNC 配置 |
| X 方向 | 20 | 原 UART TX |
| Y 步进脉冲 | 47 | FluidNC 配置 |
| Y 方向 | 48 | FluidNC 配置 |
| 激光 PWM | 17 | FluidNC 配置 |
| 电机使能(共用) | 45 | FluidNC 配置 |
| ~~UART TX~~ | ~~20~~ | 释放 |
| ~~UART RX~~ | ~~19~~ | 释放 |

## 配置参数

从 FluidNC config.yaml 提取：

| 参数 | 值 |
|---|---|
| steps_per_mm | 106.666 |
| max_rate_mm_per_min | 700 |
| acceleration_mm_per_sec2 | 800 |
| pulse_us | 10 |
| dir_delay_us | 1 |
| max_travel_mm | 42 |
| laser_pwm_hz | 5000 |
| laser_pin | gpio.17 |

## 测试策略

1. **单元测试**：G-code 解析器对已知输入输出验证
2. **运动测试**：发送单条 G1 指令，示波器检查步进脉冲时序
3. **雕刻测试**：用 GenerateGcode() 生成完整文字 G-code，执行雕刻并检查结果
4. **边界测试**：超范围移动应被软限位拦截

## 后续扩展

- G2/G3 圆弧插补（如果雕刻效果需要）
- 软限位开关支持
- 归零功能（当前配置 homing cycle 为 0）
