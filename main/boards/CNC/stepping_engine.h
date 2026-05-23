#ifndef __STEPPING_ENGINE_H__
#define __STEPPING_ENGINE_H__

// 参考 FluidNC:
//   esp32/timed_engine.c (GPIO 直驱步进引擎)
//   esp32/StepTimer.cpp  (timer_ll 硬件定时器)
//   src/Stepper.cpp      (Bresenham ISR)
//   src/Stepping.cpp     (step/unstep 时序)

#include "hal/timer_ll.h"
#include "esp_intr_alloc.h"
#include "soc/timer_periph.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_attr.h"
#include "esp_log.h"
#include <cstdint>
#include <rom/ets_sys.h>

// ─── 引脚配置 ───
#define STEP_X_STEP_PIN       GPIO_NUM_21
#define STEP_X_DIR_PIN        GPIO_NUM_20
#define STEP_Y_STEP_PIN       GPIO_NUM_47
#define STEP_Y_DIR_PIN        GPIO_NUM_48
#define STEP_ENABLE_PIN       GPIO_NUM_45
#define LASER_PWM_PIN         GPIO_NUM_17

// ─── 时序参数 ───
#define STEP_PULSE_US         10
#define STEP_DIR_DELAY_US     1
#define STEP_IDLE_MS          25

// FluidNC: fTimers=80MHz, fStepperTimer=20MHz → prescaler=4
static const uint32_t fTimers       = 80000000;
static const uint32_t fStepperTimer = 20000000;

class SteppingEngine {
private:
    static bool _timer_running;

    // ─── 定时器 ISR ───
    static bool (*timer_callback)(void);

    static void timer_isr_wrapper(void* arg) {
        timer_ll_clear_intr_status(&TIMERG0, TIMER_LL_EVENT_ALARM(0));

        if (timer_callback()) {
            timer_ll_enable_alarm(&TIMERG0, 0, true);
        }
    }

public:
    // ─── 初始化 ───
    static void Init() {
        // GPIO 初始化
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
        EnableMotors();

        // 激光 PWM
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

    // ─── 激光 ───
    static void SetLaser(uint32_t duty) {
        uint32_t scaled = (duty * 1023) / 1000;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, scaled);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }

    static void LaserOff() {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }

    // ─── 电机使能 ───
    static void EnableMotors() {
        gpio_set_level(STEP_ENABLE_PIN, 0);
    }
    static void DisableMotors() {
        gpio_set_level(STEP_ENABLE_PIN, 1);
    }

    // ─── 定时器（参考 FluidNC StepTimer.cpp） ───
    static void InitTimer(bool (*callback)(void)) {
        timer_callback = callback;

        // 禁用并配置 TG0 TIMER_0
        timer_ll_enable_intr(&TIMERG0, TIMER_LL_EVENT_ALARM(0), false);
        timer_ll_enable_counter(&TIMERG0, 0, false);
        timer_ll_set_reload_value(&TIMERG0, 0, 0ULL);
        timer_ll_trigger_soft_reload(&TIMERG0, 0);

        // 预分频: 80MHz / prescaler = 20MHz → prescaler = 4
        timer_ll_set_clock_prescale(&TIMERG0, 0, fTimers / fStepperTimer);
        timer_ll_set_count_direction(&TIMERG0, 0, GPTIMER_COUNT_UP);
        timer_ll_enable_intr(&TIMERG0, TIMER_LL_EVENT_ALARM(0), false);
        timer_ll_clear_intr_status(&TIMERG0, TIMER_LL_EVENT_ALARM(0));
        timer_ll_enable_alarm(&TIMERG0, 0, false);
        timer_ll_enable_auto_reload(&TIMERG0, 0, true);
        timer_ll_enable_counter(&TIMERG0, 0, false);
        timer_ll_set_reload_value(&TIMERG0, 0, 0);
        timer_ll_trigger_soft_reload(&TIMERG0, 0);

        // 注册中断（LEVEL3 高优先级，不使用 IRAM flag 避免 l32r 字面值池问题）
        esp_intr_alloc_intrstatus(timer_group_periph_signals.groups[0].timer_irq_id[0],
                                  ESP_INTR_FLAG_LEVEL3,
                                  (uint32_t)timer_ll_get_intr_status_reg(&TIMERG0),
                                  TIMER_LL_EVENT_ALARM(0),
                                  timer_isr_wrapper,
                                  nullptr,
                                  nullptr);

        timer_ll_enable_intr(&TIMERG0, TIMER_LL_EVENT_ALARM(0), true);
        _timer_running = false;

        ESP_LOGI("SteppingEngine", "Timer init: fTimer=%d Hz (prescaler=%d)",
                 fStepperTimer, (int)(fTimers / fStepperTimer));
    }

    static void StartTimer() {
        // 先完整复位定时器状态，再启动
        timer_ll_enable_counter(&TIMERG0, 0, false);
        timer_ll_enable_alarm(&TIMERG0, 0, false);
        timer_ll_clear_intr_status(&TIMERG0, TIMER_LL_EVENT_ALARM(0));
        timer_ll_set_reload_value(&TIMERG0, 0, 0ULL);
        timer_ll_trigger_soft_reload(&TIMERG0, 0);
        // alarm value 已由 SetTimerPeriod 预设
        timer_ll_enable_alarm(&TIMERG0, 0, true);
        timer_ll_enable_counter(&TIMERG0, 0, true);
        _timer_running = true;
    }

    static void StopTimer() {
        timer_ll_enable_counter(&TIMERG0, 0, false);
        timer_ll_enable_alarm(&TIMERG0, 0, false);
        _timer_running = false;
    }

    static void SetTimerPeriod(uint32_t ticks) {
        timer_ll_set_alarm_value(&TIMERG0, 0, (uint64_t)ticks);
    }

    // ─── GPIO 步进控制（参考 FluidNC Stepping.cpp step/unstep） ───

    // 设置方向引脚
    static void SetDirection(uint8_t dir_bits) {
        gpio_set_level(STEP_X_DIR_PIN, (dir_bits & 0x01) ? 1 : 0);
        gpio_set_level(STEP_Y_DIR_PIN, (dir_bits & 0x02) ? 1 : 0);
        // 方向建立延时
        esp_rom_delay_us(STEP_DIR_DELAY_US);
    }

    // 开始步进脉冲（拉高 step 引脚 + 等待脉冲宽度）
    static void StartStep(uint8_t step_bits) {
        if (step_bits & 0x01) gpio_set_level(STEP_X_STEP_PIN, 1);
        if (step_bits & 0x02) gpio_set_level(STEP_Y_STEP_PIN, 1);
        esp_rom_delay_us(STEP_PULSE_US);
    }

    // 结束步进脉冲（拉低 step 引脚）
    static void EndStep() {
        gpio_set_level(STEP_X_STEP_PIN, 0);
        gpio_set_level(STEP_Y_STEP_PIN, 0);
    }
};

inline bool     SteppingEngine::_timer_running = false;
inline bool (*SteppingEngine::timer_callback)(void) = nullptr;

#endif // __STEPPING_ENGINE_H__
