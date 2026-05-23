#ifndef __STEPPING_ENGINE_H__
#define __STEPPING_ENGINE_H__

#include <driver/gptimer.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <esp_log.h>
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
#define STEP_TIMER_RESOLUTION 1000000  // 1MHz → 1µs per tick

// ─── 步进引擎 ───
class SteppingEngine {
public:
    static void Init() {
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

        // 使能引脚低有效（A4988）
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

    // ─── ISR 回调 ───
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
            .alarm_count     = 1000,
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

    static void SetStepPins(uint8_t step_bits) {
        if (step_bits & 0x01) gpio_set_level(STEP_X_STEP_PIN, 1);
        if (step_bits & 0x02) gpio_set_level(STEP_Y_STEP_PIN, 1);
    }

    static void ClearAllPins() {
        gpio_set_level(STEP_X_STEP_PIN, 0);
        gpio_set_level(STEP_Y_STEP_PIN, 0);
    }

    static void SetDirection(uint8_t dir_bits) {
        gpio_set_level(STEP_X_DIR_PIN, (dir_bits & 0x01) ? 1 : 0);
        gpio_set_level(STEP_Y_DIR_PIN, (dir_bits & 0x02) ? 1 : 0);
        esp_rom_delay_us(STEP_DIR_DELAY_US);
    }

    static void DisableMotors() {
        gpio_set_level(STEP_ENABLE_PIN, 1);
    }

    static void EnableMotors() {
        gpio_set_level(STEP_ENABLE_PIN, 0);
    }

private:
    static bool TimerISR(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx) {
        if (pulse_callback) {
            return pulse_callback();
        }
        return false;
    }
};

// 静态成员定义
inline SteppingEngine::PulseCallback SteppingEngine::pulse_callback = nullptr;
inline gptimer_handle_t SteppingEngine::timer_ = nullptr;

#endif // __STEPPING_ENGINE_H__
