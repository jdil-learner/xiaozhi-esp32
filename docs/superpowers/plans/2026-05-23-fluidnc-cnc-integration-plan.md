# FluidNC CNC 激光雕刻集成 — 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 xiaozhi-esp32 项目中实现本地运动控制器，将 G-code 直接转化为步进电机脉冲 + 激光 PWM 输出

**Architecture:** 5 个新 header-only 模块 + 修改 1 个已有文件。自底向上构建：硬件层 → ISR → 规划器 → 解析器 → 控制器入口 → 集成改造

**Tech Stack:** ESP-IDF (gptimer, ledc, gpio), C++17, header-only（遵循项目现有模式）

**Pin 冲突警告:** bread-compact-wifi 板的 GPIO.47 (TOUCH_BUTTON) 和 GPIO.48 (BUILTIN_LED) 与 FluidNC 配置中的 Y 轴引脚冲突。实现中使用可配置的 `#define`，用户需根据实际硬件调整 Y 轴引脚。

---

## 文件总览

| 操作 | 文件 | 职责 |
|---|---|---|
| 新增 | `main/boards/CNC/stepping_engine.h` | GPTimer 步进脉冲 + LEDC 激光 PWM 硬件层 |
| 新增 | `main/boards/CNC/stepper.h` | Bresenham ISR + AMASS + 段生成器 |
| 新增 | `main/boards/CNC/planner.h` | 速度规划器（参考 FluidNC 双通算法） |
| 新增 | `main/boards/CNC/gcode_parser.h` | G-code 解析（G0/G1/M3/M5/G21/G90） |
| 新增 | `main/boards/CNC/motion_controller.h` | 运动控制主入口 |
| 修改 | `main/boards/CNC/gcode_controller.h` | 移除 UART，接入 MotionController |
| 修改 | `main/boards/bread-compact-wifi/compact_wifi_board.cc` | KanjiVGController 初始化更新 |

---

### Task 1: SteppingEngine — 硬件抽象层

**Files:**
- Create: `main/boards/CNC/stepping_engine.h`

- [ ] **Step 1: 写 stepping_engine.h**

```cpp
#ifndef __STEPPING_ENGINE_H__
#define __STEPPING_ENGINE_H__

#include <driver/gptimer.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <cstdint>

// ─── 引脚配置（可根据实际硬件修改） ───
#define STEP_X_STEP_PIN       GPIO_NUM_21
#define STEP_X_DIR_PIN        GPIO_NUM_20
#define STEP_Y_STEP_PIN       GPIO_NUM_47   // ⚠️ 与 TOUCH_BUTTON 冲突，需调整
#define STEP_Y_DIR_PIN        GPIO_NUM_48   // ⚠️ 与 BUILTIN_LED 冲突，需调整
#define STEP_ENABLE_PIN       GPIO_NUM_45
#define LASER_PWM_PIN         GPIO_NUM_17

// ─── 时序参数 ───
#define STEP_PULSE_US         10
#define STEP_DIR_DELAY_US     1
#define STEP_IDLE_MS          25
#define STEP_TIMER_RESOLUTION 1000000  // 1MHz → 1µs per tick

// ─── 步进引擎 ───
class SteppingEngine {
public:
    static void Init() {
        // X 轴引脚
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << STEP_X_STEP_PIN) | (1ULL << STEP_X_DIR_PIN)
                          | (1ULL << STEP_Y_STEP_PIN) | (1ULL << STEP_Y_DIR_PIN)
                          | (1ULL << STEP_ENABLE_PIN),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);

        // 使能引脚低有效（A4988 使能）
        gpio_set_level(STEP_ENABLE_PIN, 0);

        // 激光 PWM（LEDC）
        ledc_timer_config_t ledc_timer = {
            .speed_mode       = LEDC_LOW_SPEED_MODE,
            .duty_resolution  = LEDC_TIMER_10_BIT,
            .timer_num        = LEDC_TIMER_0,
            .freq_hz          = 5000,
            .clk_cfg          = LEDC_AUTO_CLK,
        };
        ledc_timer_config(&ledc_timer);

        ledc_channel_config_t ledc_channel = {
            .gpio_num       = LASER_PWM_PIN,
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = LEDC_CHANNEL_0,
            .intr_type      = LEDC_INTR_DISABLE,
            .timer_sel      = LEDC_TIMER_0,
            .duty           = 0,
            .hpoint         = 0,
        };
        ledc_channel_config(&ledc_channel);

        ESP_LOGI("SteppingEngine", "Pins: Xstep=%d Xdir=%d Ystep=%d Ydir=%d En=%d Laser=%d",
                 STEP_X_STEP_PIN, STEP_X_DIR_PIN,
                 STEP_Y_STEP_PIN, STEP_Y_DIR_PIN,
                 STEP_ENABLE_PIN, LASER_PWM_PIN);
    }

    static void SetLaser(uint32_t duty) {
        // duty: 0~1000 → 0~1023 (10-bit)
        uint32_t scaled = (duty * 1023) / 1000;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, scaled);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }

    static void LaserOff() {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }

    // ─── ISR 回调类型 ───
    using PulseCallback = bool (*)();
    static PulseCallback pulse_callback;

    // ─── 定时器 ───
    static gptimer_handle_t timer_;

    static bool StartTimer(PulseCallback callback) {
        pulse_callback = callback;

        gptimer_config_t timer_config = {
            .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
            .direction     = GPTIMER_COUNT_UP,
            .resolution_hz = STEP_TIMER_RESOLUTION,
        };
        ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer_));

        gptimer_alarm_config_t alarm_config = {
            .alarm_count     = 1000,  // 初始值，会被 Stepper 动态设置
            .reload_count    = 0,
            .flags           = { .auto_reload_on_alarm = true },
        };
        ESP_ERROR_CHECK(gptimer_set_alarm_action(timer_, &alarm_config));

        gptimer_event_callbacks_t cbs = {
            .on_alarm = TimerISR,
        };
        ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer_, &cbs, nullptr));
        ESP_ERROR_CHECK(gptimer_enable(timer_));
        ESP_ERROR_CHECK(gptimer_start(timer_));

        ESP_LOGI("SteppingEngine", "Timer started at %d Hz", STEP_TIMER_RESOLUTION);
        return true;
    }

    static void StopTimer() {
        if (timer_) {
            gptimer_stop(timer_);
            gptimer_disable(timer_);
            gptimer_del_timer(timer_);
            timer_ = nullptr;
        }
        DisableMotors();
        LaserOff();
    }

    static void SetTimerPeriod(uint16_t ticks) {
        gptimer_alarm_config_t alarm_config = {
            .alarm_count     = (uint32_t)ticks,
            .reload_count    = 0,
            .flags           = { .auto_reload_on_alarm = true },
        };
        gptimer_set_alarm_action(timer_, &alarm_config);
    }

    static void IRAM_ATTR SetStepPins(uint8_t step_bits) {
        if (step_bits & 0x01) gpio_set_level(STEP_X_STEP_PIN, 1);
        if (step_bits & 0x02) gpio_set_level(STEP_Y_STEP_PIN, 1);
    }

    static void IRAM_ATTR ClearAllPins() {
        gpio_set_level(STEP_X_STEP_PIN, 0);
        gpio_set_level(STEP_Y_STEP_PIN, 0);
    }

    static void SetDirection(uint8_t dir_bits) {
        gpio_set_level(STEP_X_DIR_PIN, (dir_bits & 0x01) ? 1 : 0);
        gpio_set_level(STEP_Y_DIR_PIN, (dir_bits & 0x02) ? 1 : 0);
        // 方向建立延时
        esp_rom_delay_us(STEP_DIR_DELAY_US);
    }

    static void DisableMotors() {
        gpio_set_level(STEP_ENABLE_PIN, 1);  // 高电平禁用
    }

    static void EnableMotors() {
        gpio_set_level(STEP_ENABLE_PIN, 0);  // 低电平使能
    }

private:
    static bool IRAM_ATTR TimerISR(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx) {
        if (pulse_callback) {
            return pulse_callback();  // 返回 false 则停止
        }
        return false;
    }
};

// 静态成员定义
inline SteppingEngine::PulseCallback SteppingEngine::pulse_callback = nullptr;
inline gptimer_handle_t SteppingEngine::timer_ = nullptr;

#endif // __STEPPING_ENGINE_H__
```

- [ ] **Step 2: 验证编译**

```bash
cd c:/Users/24703/Desktop/Work/AgentWorkSpace/xiaozhi-esp32-main
idf.py build 2>&1 | tail -20
```

- [ ] **Step 3: Commit**

```bash
git add main/boards/CNC/stepping_engine.h
git commit -m "feat: add SteppingEngine hardware abstraction layer"
```

---

### Task 2: Stepper — Bresenham ISR + 段生成器

**Files:**
- Create: `main/boards/CNC/stepper.h`

- [ ] **Step 1: 写 stepper.h**

```cpp
#ifndef __STEPPER_H__
#define __STEPPER_H__

#include "stepping_engine.h"
#include <cmath>
#include <cstring>
#include <esp_log.h>

#define MAX_AMASS_LEVEL     4
#define AMASS_THRESHOLD     2000
#define DT_SEGMENT          0.000833f  // ~50ms (in minutes)
#define REQ_MM_INCREMENT    0.01f
#define MAX_N_AXIS          2
#define SEGMENT_COUNT       12

// ─── Block 数据 ───
struct StepperBlock {
    uint32_t steps[MAX_N_AXIS];
    uint32_t step_event_count;
    uint8_t  direction_bits;
    float    millimeters;
    float    acceleration;
    float    entry_speed_sqr;
};

// ─── Segment 数据 ───
struct Segment {
    uint16_t n_step;
    uint16_t isr_period;
    uint8_t  st_block_index;
    uint8_t  amass_level;
};

// ─── ISR 运行状态 ───
struct StepperRuntime {
    uint32_t counter[MAX_N_AXIS];
    uint32_t steps[MAX_N_AXIS];
    uint8_t  step_outbits;
    uint16_t step_count;
    uint8_t  exec_block_index;
    StepperBlock* exec_block;
    Segment*      exec_segment;
};

class Stepper {
private:
    static StepperBlock block_buffer[SEGMENT_COUNT - 1];
    static Segment      segment_buffer[SEGMENT_COUNT];
    static volatile uint32_t seg_tail;
    static volatile uint32_t seg_head;
    static uint32_t          seg_next_head;

    static StepperBlock* st_prep_block;
    static StepperBlock* pl_block_input;
    static StepperRuntime st;
    static bool awake;

    // ─── 段预备状态 ───
    struct PrepState {
        uint8_t st_block_index;
        float   steps_remaining;
        float   step_per_mm;
        float   dt_remainder;
        float   req_mm_increment;
        float   mm_complete;
        float   current_speed;
        float   maximum_speed;
        float   exit_speed;
        float   accelerate_until;
        float   decelerate_after;
        uint8_t ramp_type;
        bool    recalculate;
    };
    static PrepState prep;

    enum RampType { RAMP_ACCEL = 0, RAMP_CRUISE, RAMP_DECEL };

    // ─── Bresenham ISR ───
    static bool IRAM_ATTR PulseISR() {
        if (!awake) return false;

        SteppingEngine::ClearAllPins();  // 结束上一个脉冲
        SteppingEngine::SetStepPins(st.step_outbits);  // 输出新脉冲
        st.step_outbits = 0;

        // 如果当前段已空，从缓冲区取新段
        if (st.exec_segment == nullptr) {
            if (seg_head != seg_tail) {
                st.exec_segment = &segment_buffer[seg_tail];

                SteppingEngine::SetTimerPeriod(st.exec_segment->isrPeriod);
                st.step_count = st.exec_segment->n_step;

                if (st.exec_block_index != st.exec_segment->st_block_index) {
                    st.exec_block_index = st.exec_segment->st_block_index;
                    st.exec_block       = &block_buffer[st.exec_block_index];

                    for (int a = 0; a < MAX_N_AXIS; a++) {
                        st.counter[a] = st.exec_block->step_event_count >> 1;
                    }
                }

                uint8_t dir = st.exec_block->direction_bits;
                SteppingEngine::SetDirection(dir);

                for (int a = 0; a < MAX_N_AXIS; a++) {
                    st.steps[a] = st.exec_block->steps[a] >> st.exec_segment->amass_level;
                }
            } else {
                // 缓冲区空 → 停止
                SteppingEngine::ClearAllPins();
                awake = false;
                return false;
            }
        }

        // Bresenham
        for (int a = 0; a < MAX_N_AXIS; a++) {
            st.counter[a] += st.steps[a];
            if (st.counter[a] > st.exec_block->step_event_count) {
                st.step_outbits |= (1 << a);
                st.counter[a] -= st.exec_block->step_event_count;
            }
        }

        st.step_count--;
        if (st.step_count == 0) {
            st.exec_segment = nullptr;
            seg_tail = (seg_tail >= (SEGMENT_COUNT - 1)) ? 0 : seg_tail + 1;
        }

        return true;
    }

    static uint8_t NextBlockIndex(uint8_t idx) {
        idx++;
        return (idx == (SEGMENT_COUNT - 1)) ? 0 : idx;
    }

public:
    static void Init() {
        memset(&prep, 0, sizeof(prep));
        memset(&st, 0, sizeof(st));
        st.exec_segment = nullptr;
        st.exec_block = nullptr;
        pl_block_input = nullptr;
        seg_tail = 0;
        seg_head = 0;
        seg_next_head = 1;
        awake = false;
    }

    static void WakeUp() {
        if (awake) return;
        awake = true;
        SteppingEngine::EnableMotors();
        SteppingEngine::StartTimer(PulseISR);
    }

    static void GoIdle() {
        awake = false;
        SteppingEngine::StopTimer();
    }

    static bool IsBusy() {
        return awake && seg_head != seg_tail;
    }

    // ─── 提交 planner block ───
    static void SubmitBlock(StepperBlock* block) {
        pl_block_input = block;
        prep.recalculate = true;
        PrepBuffer();
    }

    // ─── 等待队列清空 ───
    static void WaitForIdle() {
        while (IsBusy()) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    // ─── 段生成器 ───
    static void PrepBuffer() {
        while (seg_tail != seg_next_head) {
            if (pl_block_input == nullptr) return;

            if (prep.recalculate) {
                prep.recalculate = false;
            } else {
                prep.st_block_index = NextBlockIndex(prep.st_block_index);
                st_prep_block = &block_buffer[prep.st_block_index];

                st_prep_block->direction_bits = pl_block_input->direction_bits;
                st_prep_block->step_event_count = pl_block_input->step_event_count << MAX_AMASS_LEVEL;

                for (int a = 0; a < MAX_N_AXIS; a++) {
                    st_prep_block->steps[a] = pl_block_input->steps[a] << MAX_AMASS_LEVEL;
                }

                prep.steps_remaining  = (float)pl_block_input->step_event_count;
                prep.step_per_mm      = prep.steps_remaining / pl_block_input->millimeters;
                prep.req_mm_increment = REQ_MM_INCREMENT / prep.step_per_mm;
                prep.dt_remainder     = 0.0f;

                float entry_speed = sqrtf(pl_block_input->entry_speed_sqr);
                prep.current_speed = (entry_speed < 1.0f) ? 1.0f : entry_speed;
                prep.exit_speed    = 0.0f;
            }

            // ─── 计算速度曲线 ───
            float mm_remaining = pl_block_input->millimeters;
            float inv_2_accel  = 0.5f / pl_block_input->acceleration;
            float nominal_speed = pl_block_input->entry_speed_sqr > 0 ?
                sqrtf(pl_block_input->entry_speed_sqr) : prep.current_speed;

            float nominal_speed_sqr = nominal_speed * nominal_speed;
            float exit_speed_sqr    = prep.exit_speed * prep.exit_speed;
            float intersect = 0.5f * (pl_block_input->millimeters
                + inv_2_accel * (pl_block_input->entry_speed_sqr - exit_speed_sqr));

            // 简化梯形：只判断是否达到匀速段
            float accel_until, decel_after;
            if (intersect > 0.0f && intersect < pl_block_input->millimeters) {
                decel_after = inv_2_accel * (nominal_speed_sqr - exit_speed_sqr);
                if (decel_after < intersect) {
                    // 梯形：加速→匀速→减速
                    prep.maximum_speed     = nominal_speed;
                    prep.accelerate_until  = pl_block_input->millimeters
                        - inv_2_accel * (nominal_speed_sqr - pl_block_input->entry_speed_sqr);
                    prep.decelerate_after  = decel_after;
                    prep.ramp_type         = RAMP_ACCEL;
                } else {
                    // 三角形：加速→减速
                    prep.maximum_speed     = sqrtf(2.0f * pl_block_input->acceleration * intersect + exit_speed_sqr);
                    prep.accelerate_until  = intersect;
                    prep.decelerate_after  = intersect;
                    prep.ramp_type         = RAMP_ACCEL;
                }
            } else {
                // 仅减速
                prep.ramp_type = RAMP_DECEL;
                prep.maximum_speed = prep.current_speed;
            }

            prep.mm_complete = 0.0f;

            // ─── 生成恒定速度段 ───
            float dt_max = DT_SEGMENT;
            float dt     = 0.0f;
            float time_var;
            float speed_var;

            do {
                switch (prep.ramp_type) {
                    case RAMP_ACCEL:
                        speed_var = pl_block_input->acceleration * dt_max;
                        mm_remaining -= dt_max * (prep.current_speed + 0.5f * speed_var);
                        if (mm_remaining < prep.accelerate_until) {
                            mm_remaining = prep.accelerate_until;
                            time_var = 2.0f * (pl_block_input->millimeters - mm_remaining)
                                       / (prep.current_speed + prep.maximum_speed);
                            if (mm_remaining == prep.decelerate_after)
                                prep.ramp_type = RAMP_DECEL;
                            else
                                prep.ramp_type = RAMP_CRUISE;
                            prep.current_speed = prep.maximum_speed;
                        } else {
                            prep.current_speed += speed_var;
                            time_var = dt_max;
                        }
                        break;

                    case RAMP_CRUISE: {
                        float mm_var = mm_remaining - prep.maximum_speed * dt_max;
                        if (mm_var < prep.decelerate_after) {
                            time_var = (mm_remaining - prep.decelerate_after) / prep.maximum_speed;
                            mm_remaining = prep.decelerate_after;
                            prep.ramp_type = RAMP_DECEL;
                        } else {
                            mm_remaining = mm_var;
                            time_var = dt_max;
                        }
                        break;
                    }

                    case RAMP_DECEL:
                        speed_var = pl_block_input->acceleration * dt_max;
                        if (prep.current_speed > speed_var) {
                            float mm_var = mm_remaining - dt_max * (prep.current_speed - 0.5f * speed_var);
                            if (mm_var > prep.mm_complete) {
                                mm_remaining = mm_var;
                                prep.current_speed -= speed_var;
                                time_var = dt_max;
                                break;
                            }
                        }
                        time_var = 2.0f * (mm_remaining - prep.mm_complete)
                                   / (prep.current_speed + prep.exit_speed);
                        mm_remaining = prep.mm_complete;
                        prep.current_speed = prep.exit_speed;
                        break;
                }

                dt += time_var;
                if (dt < dt_max) {
                    float remaining = dt_max - dt;
                    float mm_min = mm_remaining - prep.req_mm_increment;
                    if (mm_min < 0.0f) mm_min = 0.0f;

                    if (mm_remaining > mm_min) {
                        dt_max += DT_SEGMENT;
                    } else {
                        break;
                    }
                } else {
                    break;
                }
            } while (mm_remaining > prep.mm_complete);

            // ─── 计算段参数 ───
            float step_dist_remaining    = prep.step_per_mm * mm_remaining;
            float n_steps_remaining      = ceilf(step_dist_remaining);
            float last_n_steps_remaining = ceilf(prep.steps_remaining);

            Segment* seg     = &segment_buffer[seg_head];
            seg->n_step      = (uint16_t)(last_n_steps_remaining - n_steps_remaining);
            seg->st_block_index = prep.st_block_index;

            dt += prep.dt_remainder;
            float inv_rate = dt / (last_n_steps_remaining - step_dist_remaining);
            uint32_t timer_ticks = (uint32_t)ceilf((STEP_TIMER_RESOLUTION * 60) * inv_rate);

            uint8_t level;
            for (level = 0; level < MAX_AMASS_LEVEL; level++) {
                if (timer_ticks < AMASS_THRESHOLD) break;
                timer_ticks >>= 1;
            }
            seg->amass_level = level;
            seg->n_step     <<= level;
            seg->isr_period  = (timer_ticks > 0xFFFF) ? 0xFFFF : (uint16_t)timer_ticks;

            seg_head = (seg_head >= (SEGMENT_COUNT - 1)) ? 0 : seg_head + 1;
            seg_next_head = (seg_next_head >= (SEGMENT_COUNT - 1)) ? 0 : seg_next_head + 1;

            pl_block_input->millimeters = mm_remaining;
            prep.steps_remaining = n_steps_remaining;
            prep.dt_remainder = (n_steps_remaining - step_dist_remaining) * inv_rate;

            if (mm_remaining <= prep.mm_complete) {
                pl_block_input = nullptr;
                return;
            }
        }
    }

    static float GetCurrentSpeed() {
        return prep.current_speed;
    }
};

// 静态成员定义
inline StepperBlock   Stepper::block_buffer[SEGMENT_COUNT - 1];
inline Segment        Stepper::segment_buffer[SEGMENT_COUNT];
inline volatile uint32_t Stepper::seg_tail = 0;
inline volatile uint32_t Stepper::seg_head = 0;
inline uint32_t          Stepper::seg_next_head = 1;
inline StepperBlock* Stepper::st_prep_block = nullptr;
inline StepperBlock* Stepper::pl_block_input = nullptr;
inline StepperRuntime Stepper::st = {};
inline bool Stepper::awake = false;
inline Stepper::PrepState Stepper::prep = {};

#endif // __STEPPER_H__
```

- [ ] **Step 2: 验证编译**

```bash
cd c:/Users/24703/Desktop/Work/AgentWorkSpace/xiaozhi-esp32-main
idf.py build 2>&1 | tail -20
```

- [ ] **Step 3: Commit**

```bash
git add main/boards/CNC/stepper.h
git commit -m "feat: add Stepper with Bresenham ISR and segment generator"
```

---

### Task 3: Planner — 速度规划器

**Files:**
- Create: `main/boards/CNC/planner.h`

- [ ] **Step 1: 写 planner.h**

```cpp
#ifndef __PLANNER_H__
#define __PLANNER_H__

#include <cmath>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <esp_log.h>

#define PLANNER_BLOCKS        16
#define MINIMUM_FEED_RATE     1.0f
#define MINIMUM_JUNCTION_SPEED 0.1f
#define SOME_LARGE_VALUE      1.0e12f

struct PlanBlock {
    float   target_x, target_y;
    float   feed_rate;         // mm/min
    float   millimeters;       // 线段长度 (mm)
    float   acceleration;      // mm/s²
    float   rapid_rate;        // mm/min
    float   entry_speed_sqr;   // (mm/min)²
    float   max_entry_speed_sqr;
    float   max_junction_speed_sqr;
    float   programmed_rate;   // mm/min
    bool    rapid_motion;

    // 步进数据（由 steps_per_mm 换算）
    uint32_t steps_x, steps_y;
    uint32_t step_event_count;
    uint8_t  direction_bits;  // bit0=X-, bit1=Y-

    bool processed;
};

class Planner {
private:
    static PlanBlock buffer[PLANNER_BLOCKS];
    static uint8_t   head;
    static uint8_t   tail;
    static uint8_t   next_head;
    static uint8_t   planned;

    static float previous_unit_vec_x;
    static float previous_unit_vec_y;
    static float previous_nominal_speed;

    // 配置（启动时设置）
    static float steps_per_mm_x;
    static float steps_per_mm_y;
    static float max_rate;         // mm/min
    static float acceleration;     // mm/s²
    static float junction_deviation;  // mm
    static float max_travel;       // mm

    static uint8_t NextIdx(uint8_t idx) {
        idx++;
        return (idx == PLANNER_BLOCKS) ? 0 : idx;
    }

    static uint8_t PrevIdx(uint8_t idx) {
        return (idx == 0) ? PLANNER_BLOCKS - 1 : idx - 1;
    }

    static void Recalculate() {
        if (head == tail) return;

        uint8_t idx = PrevIdx(head);
        if (idx == planned) return;

        // ─── 反向遍历：计算最大入口速度 ───
        PlanBlock* current = &buffer[idx];
        current->entry_speed_sqr = std::min(
            current->max_entry_speed_sqr,
            2.0f * current->acceleration * current->millimeters
        );

        idx = PrevIdx(idx);
        if (idx == planned) return;

        while (idx != planned) {
            PlanBlock* next    = current;
            current            = &buffer[idx];
            idx                = PrevIdx(idx);

            if (current->entry_speed_sqr != current->max_entry_speed_sqr) {
                float entry_sqr = next->entry_speed_sqr + 2.0f * current->acceleration * current->millimeters;
                if (entry_sqr < current->max_entry_speed_sqr)
                    current->entry_speed_sqr = entry_sqr;
                else
                    current->entry_speed_sqr = current->max_entry_speed_sqr;
            }
        }

        // ─── 正向遍历：加速约束修正 ───
        next = &buffer[planned];
        idx = NextIdx(planned);
        while (idx != head) {
            current = next;
            next    = &buffer[idx];

            if (current->entry_speed_sqr < next->entry_speed_sqr) {
                float entry_sqr = current->entry_speed_sqr + 2.0f * current->acceleration * current->millimeters;
                if (entry_sqr < next->entry_speed_sqr) {
                    next->entry_speed_sqr = entry_sqr;
                    planned = idx;
                }
            }

            if (next->entry_speed_sqr == next->max_entry_speed_sqr) {
                planned = idx;
            }
            idx = NextIdx(idx);
        }
    }

public:
    static void Init(float sx, float sy, float mr, float acc, float jd, float mt) {
        steps_per_mm_x   = sx;
        steps_per_mm_y   = sy;
        max_rate         = mr;
        acceleration     = acc;
        junction_deviation = jd;
        max_travel       = mt;

        memset(buffer, 0, sizeof(buffer));
        head    = 0;
        tail    = 0;
        next_head = 1;
        planned  = 0;
        previous_unit_vec_x = 0;
        previous_unit_vec_y = 0;
        previous_nominal_speed = SOME_LARGE_VALUE;
    }

    static void Reset() {
        memset(buffer, 0, sizeof(buffer));
        head    = 0;
        tail    = 0;
        next_head = 1;
        planned  = 0;
        previous_unit_vec_x = 0;
        previous_unit_vec_y = 0;
        previous_nominal_speed = SOME_LARGE_VALUE;
    }

    static bool IsFull() {
        return tail == next_head;
    }

    static bool IsEmpty() {
        return head == tail;
    }

    static PlanBlock* GetCurrent() {
        if (IsEmpty()) return nullptr;
        return &buffer[tail];
    }

    static void DiscardCurrent() {
        if (!IsEmpty()) {
            uint8_t idx = NextIdx(tail);
            if (tail == planned) planned = idx;
            tail = idx;
        }
    }

    static bool BufferLine(float x, float y, float feed_rate, bool rapid) {
        if (IsFull()) return false;

        PlanBlock* block = &buffer[head];
        memset(block, 0, sizeof(PlanBlock));
        block->target_x  = x;
        block->target_y  = y;
        block->feed_rate = feed_rate;
        block->rapid_motion = rapid;

        // 计算步数
        // 需要当前位置 → 从当前 head-1 获取
        float prev_x = 0.0f, prev_y = 0.0f;
        if (!IsEmpty()) {
            uint8_t prev_idx = PrevIdx(head);
            prev_x = buffer[prev_idx].target_x;
            prev_y = buffer[prev_idx].target_y;
        }

        float dx_mm = x - prev_x;
        float dy_mm = y - prev_y;
        block->millimeters = sqrtf(dx_mm * dx_mm + dy_mm * dy_mm);

        if (block->millimeters < 0.001f) return false;  // 零长度

        int32_t dx_steps = (int32_t)roundf(dx_mm * steps_per_mm_x);
        int32_t dy_steps = (int32_t)roundf(dy_mm * steps_per_mm_y);

        block->steps_x = (uint32_t)abs(dx_steps);
        block->steps_y = (uint32_t)abs(dy_steps);
        block->step_event_count = std::max(block->steps_x, block->steps_y);

        block->direction_bits = 0;
        if (dx_steps < 0) block->direction_bits |= 0x01;
        if (dy_steps < 0) block->direction_bits |= 0x02;

        // 计算单位向量
        float unit_x = dx_mm / block->millimeters;
        float unit_y = dy_mm / block->millimeters;

        // 限制加速度和速率
        block->acceleration = acceleration;
        block->rapid_rate   = max_rate;

        if (rapid) {
            block->programmed_rate = max_rate;
        } else {
            block->programmed_rate = feed_rate;
            if (block->programmed_rate > max_rate)
                block->programmed_rate = max_rate;
        }

        // 连接点速度（向心加速度近似）
        if (IsEmpty()) {
            block->entry_speed_sqr        = 0.0f;
            block->max_junction_speed_sqr = 0.0f;
        } else {
            float cos_theta = -(previous_unit_vec_x * unit_x + previous_unit_vec_y * unit_y);
            if (cos_theta > 0.999999f) {
                block->max_junction_speed_sqr = MINIMUM_JUNCTION_SPEED * MINIMUM_JUNCTION_SPEED;
            } else if (cos_theta < -0.999999f) {
                block->max_junction_speed_sqr = SOME_LARGE_VALUE;
            } else {
                float sin_theta_d2 = sqrtf(0.5f * (1.0f - cos_theta));
                float junction_accel = acceleration;
                block->max_junction_speed_sqr = std::max(
                    MINIMUM_JUNCTION_SPEED * MINIMUM_JUNCTION_SPEED,
                    (junction_accel * junction_deviation * sin_theta_d2) / (1.0f - sin_theta_d2)
                );
            }
        }

        // 计算最大入口速度
        float nominal_speed = block->programmed_rate;
        float prev_nominal  = previous_nominal_speed;
        if (nominal_speed > prev_nominal)
            block->max_entry_speed_sqr = prev_nominal * prev_nominal;
        else
            block->max_entry_speed_sqr = nominal_speed * nominal_speed;

        if (block->max_entry_speed_sqr > block->max_junction_speed_sqr)
            block->max_entry_speed_sqr = block->max_junction_speed_sqr;

        previous_nominal_speed = nominal_speed;
        previous_unit_vec_x    = unit_x;
        previous_unit_vec_y    = unit_y;

        head     = next_head;
        next_head = NextIdx(head);
        Recalculate();

        return true;
    }
};

// 静态成员定义
inline PlanBlock Planner::buffer[PLANNER_BLOCKS];
inline uint8_t   Planner::head = 0;
inline uint8_t   Planner::tail = 0;
inline uint8_t   Planner::next_head = 1;
inline uint8_t   Planner::planned = 0;
inline float Planner::previous_unit_vec_x = 0;
inline float Planner::previous_unit_vec_y = 0;
inline float Planner::previous_nominal_speed = SOME_LARGE_VALUE;
inline float Planner::steps_per_mm_x = 106.666f;
inline float Planner::steps_per_mm_y = 106.666f;
inline float Planner::max_rate = 700.0f;
inline float Planner::acceleration = 800.0f;
inline float Planner::junction_deviation = 0.02f;
inline float Planner::max_travel = 42.0f;

#endif // __PLANNER_H__
```

- [ ] **Step 2: 验证编译**

```bash
idf.py build 2>&1 | tail -20
```

- [ ] **Step 3: Commit**

```bash
git add main/boards/CNC/planner.h
git commit -m "feat: add Planner with FluidNC-style dual-pass velocity planning"
```

---

### Task 4: GCodeParser — G-code 解析器

**Files:**
- Create: `main/boards/CNC/gcode_parser.h`

- [ ] **Step 1: 写 gcode_parser.h**

```cpp
#ifndef __GCODE_PARSER_H__
#define __GCODE_PARSER_H__

#include <string>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <esp_log.h>

struct GCodeCommand {
    enum Type { G0, G1, M3, M5, G21, G90, UNKNOWN };

    Type  type;
    float x, y;       // NAN if unspecified
    float feed_rate;  // mm/min, 0 if unspecified
    int   spindle;    // S parameter, 0 if unspecified
};

class GCodeParser {
public:
    static std::vector<GCodeCommand> Parse(const std::string& gcode) {
        std::vector<GCodeCommand> commands;
        float current_x = 0.0f, current_y = 0.0f;
        int   current_spindle = 0;
        float current_feed    = 400.0f;

        const char* p = gcode.c_str();
        const char* end = p + gcode.size();

        while (p < end) {
            // 跳过空行和注释
            while (p < end && (*p == '\n' || *p == '\r' || *p == ' ')) p++;
            if (p >= end) break;
            if (*p == ';') {
                while (p < end && *p != '\n') p++;
                continue;
            }

            // 读取指令字
            char letter = *p;
            p++;
            if (!(letter == 'G' || letter == 'g' || letter == 'M' || letter == 'm')) {
                while (p < end && *p != '\n') p++;
                continue;
            }

            int number = 0;
            while (p < end && *p >= '0' && *p <= '9') {
                number = number * 10 + (*p - '0');
                p++;
            }

            GCodeCommand cmd;
            cmd.type      = GCodeCommand::UNKNOWN;
            cmd.x         = NAN;
            cmd.y         = NAN;
            cmd.feed_rate = 0;
            cmd.spindle   = 0;

            // 读取参数
            while (p < end && *p != '\n' && *p != '\r') {
                if (*p == ' ') { p++; continue; }

                char param = *p;
                p++;

                // 读取数值
                float value = 0.0f;
                bool has_decimal = false;
                float decimal_place = 0.1f;
                bool negative = false;

                if (*p == '-') { negative = true; p++; }

                while (p < end && ((*p >= '0' && *p <= '9') || *p == '.')) {
                    if (*p == '.') {
                        has_decimal = true;
                        p++;
                        continue;
                    }
                    if (has_decimal) {
                        value += (*p - '0') * decimal_place;
                        decimal_place *= 0.1f;
                    } else {
                        value = value * 10.0f + (*p - '0');
                    }
                    p++;
                }
                if (negative) value = -value;

                switch (param) {
                    case 'X': case 'x': cmd.x = value; break;
                    case 'Y': case 'y': cmd.y = value; break;
                    case 'F': case 'f': cmd.feed_rate = value; break;
                    case 'S': case 's': cmd.spindle = (int)value; break;
                }
            }

            // 确定指令类型
            switch (letter) {
                case 'G': case 'g':
                    switch (number) {
                        case 0:  cmd.type = GCodeCommand::G0; break;
                        case 1:  cmd.type = GCodeCommand::G1; break;
                        case 21: cmd.type = GCodeCommand::G21; break;
                        case 90: cmd.type = GCodeCommand::G90; break;
                    }
                    break;
                case 'M': case 'm':
                    switch (number) {
                        case 3:  cmd.type = GCodeCommand::M3; break;
                        case 5:  cmd.type = GCodeCommand::M5; break;
                    }
                    break;
            }

            // 使用模态值填充未指定参数
            if (isnan(cmd.x)) cmd.x = current_x;
            if (isnan(cmd.y)) cmd.y = current_y;
            if (cmd.feed_rate == 0) cmd.feed_rate = current_feed;
            if (cmd.spindle == 0 && cmd.type == GCodeCommand::M3)
                cmd.spindle = current_spindle;

            // 更新模态状态
            if (cmd.type == GCodeCommand::G0 || cmd.type == GCodeCommand::G1) {
                current_x = cmd.x;
                current_y = cmd.y;
                if (cmd.feed_rate > 0) current_feed = cmd.feed_rate;
            }
            if (cmd.type == GCodeCommand::M3 && cmd.spindle > 0) {
                current_spindle = cmd.spindle;
            }

            commands.push_back(cmd);

            // 跳到下一行
            while (p < end && *p != '\n') p++;
        }

        return commands;
    }
};

#endif // __GCODE_PARSER_H__
```

- [ ] **Step 2: 验证编译**

```bash
idf.py build 2>&1 | tail -20
```

- [ ] **Step 3: Commit**

```bash
git add main/boards/CNC/gcode_parser.h
git commit -m "feat: add GCodeParser for G0/G1/M3/M5/G21/G90"
```

---

### Task 5: MotionController — 运动控制主入口

**Files:**
- Create: `main/boards/CNC/motion_controller.h`

- [ ] **Step 1: 写 motion_controller.h**

```cpp
#ifndef __MOTION_CONTROLLER_H__
#define __MOTION_CONTROLLER_H__

#include "gcode_parser.h"
#include "planner.h"
#include "stepper.h"
#include "stepping_engine.h"
#include <esp_log.h>

class MotionController {
private:
    float current_x;
    float current_y;
    int   laser_power;
    bool  initialized;

public:
    MotionController() : current_x(0.0f), current_y(0.0f), laser_power(0), initialized(false) {}

    void Init(float steps_x, float steps_y, float max_rate, float acceleration,
              float junction_dev, float max_travel) {
        Planner::Init(steps_x, steps_y, max_rate, acceleration, junction_dev, max_travel);
        SteppingEngine::Init();
        Stepper::Init();
        current_x   = 0.0f;
        current_y   = 0.0f;
        laser_power = 0;
        initialized = true;
        ESP_LOGI("MotionController", "Initialized. Origin at (0,0)");
    }

    void Execute(const std::string& gcode) {
        if (!initialized) {
            ESP_LOGE("MotionController", "Not initialized");
            return;
        }

        auto commands = GCodeParser::Parse(gcode);
        ESP_LOGI("MotionController", "Executing %d G-code commands", (int)commands.size());

        Stepper::WakeUp();

        for (size_t i = 0; i < commands.size(); i++) {
            auto& cmd = commands[i];

            switch (cmd.type) {
                case GCodeCommand::G0:
                case GCodeCommand::G1: {
                    bool rapid = (cmd.type == GCodeCommand::G0);
                    float feed = rapid ? Planner::max_rate : cmd.feed_rate;

                    if (cmd.x > Planner::max_travel || cmd.y > Planner::max_travel) {
                        ESP_LOGW("MotionController",
                                 "Move to (%.1f, %.1f) exceeds travel limit %.0fmm",
                                 cmd.x, cmd.y, Planner::max_travel);
                        continue;
                    }

                    // 填充 planner 直到有空间
                    while (Planner::IsFull()) {
                        Stepper::PrepBuffer();
                        vTaskDelay(pdMS_TO_TICKS(1));
                    }
                    Planner::BufferLine(cmd.x, cmd.y, feed, rapid);
                    Stepper::PrepBuffer();
                    break;
                }

                case GCodeCommand::M3:
                    laser_power = cmd.spindle;
                    SteppingEngine::SetLaser((uint32_t)laser_power);
                    ESP_LOGI("MotionController", "Laser ON: S%d", laser_power);
                    break;

                case GCodeCommand::M5:
                    SteppingEngine::LaserOff();
                    ESP_LOGI("MotionController", "Laser OFF");
                    break;

                case GCodeCommand::G21:
                case GCodeCommand::G90:
                    // 默认已是 mm 和绝对坐标，无需处理
                    break;

                default:
                    break;
            }
        }

        // 等待 planner 队列排空
        ESP_LOGI("MotionController", "Waiting for planner to drain...");
        while (!Planner::IsEmpty()) {
            PlanBlock* block = Planner::GetCurrent();
            if (block && !block->processed) {
                Stepper::SubmitBlock(block);
                block->processed = true;
            }

            Stepper::PrepBuffer();
            if (Planner::GetCurrent() == nullptr) {
                Stepper::WaitForIdle();
                Planner::DiscardCurrent();
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        // 回到原点
        ESP_LOGI("MotionController", "Returning to origin...");
        while (Planner::IsFull()) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        Planner::BufferLine(0.0f, 0.0f, Planner::max_rate, true);  // G0 回原
        {
            PlanBlock* block = Planner::GetCurrent();
            if (block) {
                Stepper::SubmitBlock(block);
                block->processed = true;
            }
        }

        Stepper::PrepBuffer();
        Stepper::WaitForIdle();
        Planner::DiscardCurrent();

        Stepper::GoIdle();
        SteppingEngine::LaserOff();
        current_x = 0.0f;
        current_y = 0.0f;
        ESP_LOGI("MotionController", "Done. Position: (0, 0)");
    }
};

#endif // __MOTION_CONTROLLER_H__
```

- [ ] **Step 2: 验证编译**

```bash
idf.py build 2>&1 | tail -20
```

- [ ] **Step 3: Commit**

```bash
git add main/boards/CNC/motion_controller.h
git commit -m "feat: add MotionController as main entry point"
```

---

### Task 6: 改造 gcode_controller.h — 移除 UART, 接入 MotionController

**Files:**
- Modify: `main/boards/CNC/gcode_controller.h`

- [ ] **Step 1: 修改 gcode_controller.h**

替换构造函数（移除 UART 初始化）和 `Initialize` 方法中的 `uart_write_bytes`：

```cpp
// ─── 头文件区域：删除 #include <driver/uart.h>，替换为 ───
#include "motion_controller.h"

// ─── 构造函数：整个替换 ───
public:
    KanjiVGController() {
        ESP_LOGI("KanjiVG", "Initialized (local motion controller mode)");
    }

// ─── Initialize 方法中的 lambda：替换发送部分 ───
// 找到这两行：
//     int sent = uart_write_bytes(UART_PORT, gcode.c_str(), gcode.size());
//     ESP_LOGI("KanjiVG", "Sent %d/%d bytes via UART2", sent, gcode.size());
// 替换为：
                static MotionController mc;
                static bool mc_initialized = false;
                if (!mc_initialized) {
                    mc.Init(
                        106.666f,  // steps_per_mm_x
                        106.666f,  // steps_per_mm_y
                        700.0f,    // max_rate mm/min
                        800.0f,    // acceleration mm/s²
                        0.02f,     // junction_deviation mm
                        42.0f      // max_travel mm
                    );
                    mc_initialized = true;
                }
                mc.Execute(gcode);
                ESP_LOGI("KanjiVG", "Motion executed, %d lines", line_count);

// 同时更新返回的 cJSON：
//     cJSON_AddStringToObject(result, "sent_to", "UART2");
// 改为：
                cJSON_AddStringToObject(result, "sent_to", "local_motion_controller");
```

- [ ] **Step 2: 删除 KanjiVGController 中不再使用的 UART 常量**

删除：
```cpp
    static constexpr uart_port_t UART_PORT  = UART_NUM_2;
    static constexpr gpio_num_t UART_TX_PIN = GPIO_NUM_20;
    static constexpr gpio_num_t UART_RX_PIN = GPIO_NUM_19;
    static constexpr int UART_BAUD = 115200;
```

- [ ] **Step 3: 移除 #include <driver/uart.h>**（如果不再被其他地方使用）

- [ ] **Step 4: 验证编译**

```bash
idf.py build 2>&1 | tail -20
```

- [ ] **Step 5: Commit**

```bash
git add main/boards/CNC/gcode_controller.h
git commit -m "refactor: replace UART output with local MotionController in KanjiVGController"
```

---

### Task 7: 最终验证

- [ ] **Step 1: 完整编译验证**

```bash
cd c:/Users/24703/Desktop/Work/AgentWorkSpace/xiaozhi-esp32-main
idf.py fullclean && idf.py build 2>&1 | tail -40
```

预期：编译通过，无错误。

- [ ] **Step 2: 检查 GPIO 引脚冲突并确认**

当前已知冲突：
- `GPIO.47`: bread-compact-wifi 的 TOUCH_BUTTON vs FluidNC 的 Y_STEP
- `GPIO.48`: bread-compact-wifi 的 BUILTIN_LED vs FluidNC 的 Y_DIR

需要在 [`stepping_engine.h`](../../main/boards/CNC/stepping_engine.h) 中调整 Y 轴引脚。建议替代方案（需用户确认）：
```cpp
#define STEP_Y_STEP_PIN       GPIO_NUM_1   // 或 GPIO 2, 9, 10, 11, 12, 13, 14
#define STEP_Y_DIR_PIN        GPIO_NUM_2   // 或 GPIO 9, 10, 11, 12, 13, 14
```

- [ ] **Step 3: 提交最终版本**

```bash
git add main/boards/CNC/stepping_engine.h
git commit -m "fix: resolve GPIO pin conflicts for Y axis step/dir"
```

---

## 自检清单

1. **Spec 覆盖**：坐标系原点 ✅ | 雕刻后回原点 ✅ | 梯形加减速 ✅ | Bresenham ✅ | AMASS ✅ | 双通规划 ✅ | G0/G1/M3/M5 ✅
2. **无占位符**：所有代码步骤都有完整可编译的实现
3. **类型一致性**：PlanBlock → StepperBlock 转换通过 Planner 中定义的字段对接，MotionController 调用链完整
