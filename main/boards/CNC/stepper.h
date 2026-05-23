#ifndef __STEPPER_H__
#define __STEPPER_H__

#include "stepping_engine.h"
#include "planner.h"
#include <cmath>
#include <cstring>
#include <esp_log.h>

#define MAX_AMASS_LEVEL     4
#define AMASS_THRESHOLD     2000
#define DT_SEGMENT          0.000833f  // ~50ms (in minutes)
#define REQ_MM_INCREMENT    0.01f
#define SEGMENT_COUNT       12

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
    PlanBlock* exec_block;
    Segment*      exec_segment;
};

class Stepper {
private:
    static PlanBlock block_buffer[SEGMENT_COUNT - 1];
    static Segment      segment_buffer[SEGMENT_COUNT];
    static volatile uint32_t seg_tail;
    static volatile uint32_t seg_head;
    static uint32_t          seg_next_head;

    static PlanBlock* st_prep_block;
    static PlanBlock* pl_block_input;
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
    static bool PulseISR() {
        if (!awake) return false;

        SteppingEngine::ClearAllPins();
        SteppingEngine::SetStepPins(st.step_outbits);
        st.step_outbits = 0;

        if (st.exec_segment == nullptr) {
            if (seg_head != seg_tail) {
                st.exec_segment = &segment_buffer[seg_tail];

                SteppingEngine::SetTimerPeriod(st.exec_segment->isr_period);
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
    static void SubmitBlock(PlanBlock* block) {
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

            float decel_after;
            if (intersect > 0.0f && intersect < pl_block_input->millimeters) {
                decel_after = inv_2_accel * (nominal_speed_sqr - exit_speed_sqr);
                if (decel_after < intersect) {
                    prep.maximum_speed     = nominal_speed;
                    prep.accelerate_until  = pl_block_input->millimeters
                        - inv_2_accel * (nominal_speed_sqr - pl_block_input->entry_speed_sqr);
                    prep.decelerate_after  = decel_after;
                    prep.ramp_type         = RAMP_ACCEL;
                } else {
                    prep.maximum_speed     = sqrtf(2.0f * pl_block_input->acceleration * intersect + exit_speed_sqr);
                    prep.accelerate_until  = intersect;
                    prep.decelerate_after  = intersect;
                    prep.ramp_type         = RAMP_ACCEL;
                }
            } else {
                prep.ramp_type = RAMP_DECEL;
                prep.maximum_speed = prep.current_speed;
            }

            prep.mm_complete = 0.0f;

            // ─── 生成恒定速度段 ───
            float dt_max = DT_SEGMENT;
            float dt     = 0.0f;
            float time_var = 0.0f;

            do {
                switch (prep.ramp_type) {
                    case RAMP_ACCEL: {
                        float speed_var = pl_block_input->acceleration * dt_max;
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
                    }

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

                    case RAMP_DECEL: {
                        float speed_var = pl_block_input->acceleration * dt_max;
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
                }

                dt += time_var;
                if (dt < dt_max) {
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
inline PlanBlock       Stepper::block_buffer[SEGMENT_COUNT - 1];
inline Segment            Stepper::segment_buffer[SEGMENT_COUNT];
inline volatile uint32_t  Stepper::seg_tail = 0;
inline volatile uint32_t  Stepper::seg_head = 0;
inline uint32_t           Stepper::seg_next_head = 1;
inline PlanBlock*      Stepper::st_prep_block = nullptr;
inline PlanBlock*      Stepper::pl_block_input = nullptr;
inline StepperRuntime     Stepper::st = {};
inline bool               Stepper::awake = false;
inline Stepper::PrepState Stepper::prep = {};

#endif // __STEPPER_H__
