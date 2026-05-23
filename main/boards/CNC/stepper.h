#ifndef __STEPPER_H__
#define __STEPPER_H__

// 参考 FluidNC src/Stepper.cpp pulse_func + prep_buffer

#include "stepping_engine.h"
#include "planner.h"
#include <cmath>
#include <cstring>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class Stepper {
private:
    static PlanBlock* _current_block;

    // ─── ISR 运行状态 ───
    struct StepperState {
        uint32_t counter[2];
        uint32_t steps[2];
        uint32_t step_event_count;
        uint8_t  step_outbits;
        uint8_t  dir_outbits;
        uint32_t step_count;
        bool     active;
    };
    static StepperState st;

    static uint32_t isr_period;

public:
    static void Init() {
        memset(&st, 0, sizeof(st));
        _current_block = nullptr;
        isr_period = 1000;
    }

    // ─── ISR 回调（由 SteppingEngine 的定时器 ISR 调用） ───
    // 返回 true = 继续定时器，false = 停止
    static bool PulseISR() {
        if (!st.active) return false;

        // 1. 输出脉冲（上周期 Bresenham 的计算结果）
        //    波形: HIGH(10µs) → LOW(ISR周期−10µs)
        if (st.step_outbits) {
            SteppingEngine::StartStep(st.step_outbits);  // pin HIGH + 10µs delay
        }
        SteppingEngine::EndStep();  // pin LOW（之后保持到下次 ISR）
        st.step_outbits = 0;

        // 2. Bresenham 计算下周期的 step_outbits
        for (int a = 0; a < 2; a++) {
            if (st.steps[a] == 0) continue;
            st.counter[a] += st.steps[a];
            if (st.counter[a] > st.step_event_count) {
                st.step_outbits |= (1 << a);
                st.counter[a] -= st.step_event_count;
            }
        }

        // 3. 步数递减
        st.step_count--;
        if (st.step_count == 0) {
            st.active = false;
            _current_block = nullptr;
            return false;
        }

        return true;
    }

    // ─── 提交 block（将一个 PlanBlock 转换为步进运动） ───
    static void SubmitBlock(PlanBlock* block) {
        SteppingEngine::EnableMotors();  // 开始运动前使能
        _current_block = block;

        // 计算定时器周期
        // 速度 mm/min → mm/s → steps/s → µs/step → timer ticks
        float feed_mm_per_sec = block->programmed_rate / 60.0f;
        float steps_per_sec = feed_mm_per_sec * Planner::steps_per_mm_x;
        if (steps_per_sec < 10.0f) steps_per_sec = 10.0f;
        if (steps_per_sec > 10000.0f) steps_per_sec = 10000.0f;

        // fStepperTimer = 20_000_000 Hz → 1 tick = 0.05µs
        // period = fStepperTimer / steps_per_sec
        isr_period = (uint32_t)(fStepperTimer / steps_per_sec);
        if (isr_period < 200) isr_period = 200;      // 最快 100kHz
        if (isr_period > 2000000) isr_period = 2000000; // 最慢 10Hz

        // 设置 ISR 状态
        st.step_event_count = block->step_event_count;
        st.steps[0]         = block->steps[0];
        st.steps[1]         = block->steps[1];
        st.counter[0]       = block->step_event_count >> 1;
        st.counter[1]       = block->step_event_count >> 1;
        st.step_count       = (uint32_t)block->step_event_count + 1;  // +1 补偿首次 ISR 预热
        st.step_outbits     = 0;
        st.dir_outbits      = block->direction_bits;
        st.active           = true;

        // 设置方向
        SteppingEngine::SetDirection(st.dir_outbits);

        // 设置定时器周期
        SteppingEngine::SetTimerPeriod(isr_period);

        ESP_LOGI("Stepper", "Block: dir=0x%02X %lu steps X=%lu Y=%lu F=%.0f",
                 st.dir_outbits, st.step_count, st.steps[0], st.steps[1],
                 block->programmed_rate);

        // 启动定时器
        SteppingEngine::StartTimer();
    }

    static bool IsBlockDone() {
        return _current_block == nullptr && !st.active;
    }

    static bool IsBusy() {
        return st.active;
    }

    static void PrepBuffer() {
        // 简化版：不需要段生成，block 直接由 ISR 消费
    }

    static void WaitForIdle() {
        while (st.active) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    static void GoIdle() {
        SteppingEngine::StopTimer();
        SteppingEngine::DisableMotors();
        SteppingEngine::LaserOff();
        st.active = false;
    }

    static float GetCurrentSpeed() {
        if (!st.active) return 0;
        float steps_per_sec = (float)fStepperTimer / isr_period;
        return steps_per_sec / Planner::steps_per_mm_x * 60.0f;
    }
};

inline PlanBlock*           Stepper::_current_block = nullptr;
inline uint32_t             Stepper::isr_period = 1000;
inline Stepper::StepperState Stepper::st = {};

#endif // __STEPPER_H__
