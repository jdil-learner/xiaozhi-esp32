#ifndef __PLANNER_H__
#define __PLANNER_H__

#include <cmath>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <esp_log.h>

#define PLANNER_BLOCKS        16
#define MAX_N_AXIS            2
#define MINIMUM_FEED_RATE     1.0f
#define MINIMUM_JUNCTION_SPEED 0.1f
#define SOME_LARGE_VALUE      1.0e12f

struct PlanBlock {
    float   target_x, target_y;
    float   feed_rate;
    float   millimeters;
    float   acceleration;
    float   rapid_rate;
    float   entry_speed_sqr;
    float   max_entry_speed_sqr;
    float   max_junction_speed_sqr;
    float   programmed_rate;
    bool    rapid_motion;

    uint32_t steps[MAX_N_AXIS];
    uint32_t step_event_count;
    uint8_t  direction_bits;

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

    static float steps_per_mm_x;
    static float steps_per_mm_y;

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
                float entry_sqr = next->entry_speed_sqr
                    + 2.0f * current->acceleration * current->millimeters;
                if (entry_sqr < current->max_entry_speed_sqr)
                    current->entry_speed_sqr = entry_sqr;
                else
                    current->entry_speed_sqr = current->max_entry_speed_sqr;
            }
        }

        // ─── 正向遍历：加速度约束修正 ───
        PlanBlock* next = &buffer[planned];
        idx = NextIdx(planned);
        while (idx != head) {
            current = next;
            next    = &buffer[idx];

            if (current->entry_speed_sqr < next->entry_speed_sqr) {
                float entry_sqr = current->entry_speed_sqr
                    + 2.0f * current->acceleration * current->millimeters;
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
    static float max_rate;
    static float acceleration;
    static float junction_deviation;
    static float max_travel;

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

        float prev_x = 0.0f, prev_y = 0.0f;
        if (!IsEmpty()) {
            uint8_t prev_idx = PrevIdx(head);
            prev_x = buffer[prev_idx].target_x;
            prev_y = buffer[prev_idx].target_y;
        }

        float dx_mm = x - prev_x;
        float dy_mm = y - prev_y;
        block->millimeters = sqrtf(dx_mm * dx_mm + dy_mm * dy_mm);

        if (block->millimeters < 0.001f) return false;

        int32_t dx_steps = (int32_t)roundf(dx_mm * steps_per_mm_x);
        int32_t dy_steps = (int32_t)roundf(dy_mm * steps_per_mm_y);

        block->steps[0] = (uint32_t)abs(dx_steps);
        block->steps[1] = (uint32_t)abs(dy_steps);
        block->step_event_count = std::max(block->steps[0], block->steps[1]);

        block->direction_bits = 0;
        if (dx_steps < 0) block->direction_bits |= 0x01;
        if (dy_steps < 0) block->direction_bits |= 0x02;

        float unit_x = dx_mm / block->millimeters;
        float unit_y = dy_mm / block->millimeters;

        block->acceleration = acceleration;
        block->rapid_rate   = max_rate;

        if (rapid) {
            block->programmed_rate = max_rate;
        } else {
            block->programmed_rate = feed_rate;
            if (block->programmed_rate > max_rate)
                block->programmed_rate = max_rate;
        }

        // ─── 连接点速度 ───
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

        // ─── 最大入口速度 ───
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
