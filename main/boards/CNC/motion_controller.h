#ifndef __MOTION_CONTROLLER_H__
#define __MOTION_CONTROLLER_H__

#include "gcode_parser.h"
#include "planner.h"
#include "stepper.h"
#include "stepping_engine.h"
#include <cstring>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class MotionController {
private:
    float current_x;
    float current_y;
    int   laser_power;
    bool  initialized;

public:
    MotionController() : current_x(0.0f), current_y(0.0f), laser_power(0), initialized(false) {}

    static MotionController& Get() {
        static MotionController instance;
        return instance;
    }

    static void GlobalInit(float steps_x, float steps_y, float max_rate, float acceleration,
                           float junction_dev, float max_travel) {
        auto& mc = Get();
        if (mc.initialized) return;
        Planner::Init(steps_x, steps_y, max_rate, acceleration, junction_dev, max_travel);
        SteppingEngine::Init();
        SteppingEngine::InitTimer(Stepper::PulseISR);
        Stepper::Init();
        mc.current_x   = 0.0f;
        mc.current_y   = 0.0f;
        mc.laser_power = 0;
        mc.initialized = true;
        ESP_LOGI("MotionController", "Initialized. Origin at (0,0)");
    }

    void Execute(const std::string& gcode) {
        if (!initialized) {
            ESP_LOGE("MotionController", "Not initialized");
            return;
        }

        // 仅显示 G-code 注释行（含雕刻参数）
        const char* gp = gcode.c_str();
        while (*gp) {
            if (*gp == ';') {
                const char* nl = strchr(gp, '\n');
                if (nl) {
                    ESP_LOGI("GCode", "%.*s", (int)(nl - gp), gp);
                    gp = nl + 1;
                } else {
                    ESP_LOGI("GCode", "%s", gp);
                    break;
                }
            } else {
                const char* nl = strchr(gp, '\n');
                gp = nl ? nl + 1 : gp + strlen(gp);
            }
        }

        auto commands = GCodeParser::Parse(gcode);
        ESP_LOGI("MotionController", "Parsed %d G-code commands", (int)commands.size());

        float pos_x = 0.0f, pos_y = 0.0f;  // 追踪当前位置
        float last_x = 0.0f, last_y = 0.0f;

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

                    last_x = cmd.x;
                    last_y = cmd.y;

                    // 直接从当前位置计算增量，绕过 Planner
                    float dx = cmd.x - pos_x;
                    float dy = cmd.y - pos_y;
                    float mm = sqrtf(dx*dx + dy*dy);

                    if (mm > 0.001f) {
                        PlanBlock block;
                        memset(&block, 0, sizeof(block));
                        block.millimeters = mm;
                        int32_t sx = (int32_t)roundf(dx * Planner::steps_per_mm_x);
                        int32_t sy = (int32_t)roundf(dy * Planner::steps_per_mm_y);
                        block.steps[0] = (uint32_t)abs(sx);
                        block.steps[1] = (uint32_t)abs(sy);
                        block.step_event_count = std::max(block.steps[0], block.steps[1]);
                        block.direction_bits = 0;
                        if (sx < 0) block.direction_bits |= 0x01;
                        if (sy < 0) block.direction_bits |= 0x02;
                        block.programmed_rate = feed;

                        Stepper::SubmitBlock(&block);
                        Stepper::WaitForIdle();
                    }

                    pos_x = cmd.x;
                    pos_y = cmd.y;
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
                    break;

                default:
                    break;
            }
        }

        // 第二步：回到原点（直接构造 block，不走 Planner）
        ESP_LOGI("MotionController", "Returning to origin from (%.2f, %.2f)...", last_x, last_y);
        {
            float dx = 0.0f - last_x;
            float dy = 0.0f - last_y;
            float mm = sqrtf(dx*dx + dy*dy);

            if (mm > 0.001f) {
                PlanBlock home;
                memset(&home, 0, sizeof(home));
                home.millimeters     = mm;
                int32_t sx = (int32_t)roundf(dx * Planner::steps_per_mm_x);
                int32_t sy = (int32_t)roundf(dy * Planner::steps_per_mm_y);
                home.steps[0]        = (uint32_t)abs(sx);
                home.steps[1]        = (uint32_t)abs(sy);
                home.step_event_count = std::max(home.steps[0], home.steps[1]);
                home.direction_bits   = 0;
                if (sx < 0) home.direction_bits |= 0x01;
                if (sy < 0) home.direction_bits |= 0x02;
                home.programmed_rate  = Planner::max_rate;

                Stepper::SubmitBlock(&home);
                Stepper::WaitForIdle();
            }
        }

        Stepper::GoIdle();
        current_x = 0.0f;
        current_y = 0.0f;
        ESP_LOGI("MotionController", "Done. Position: (0, 0)");
    }
};

#endif // __MOTION_CONTROLLER_H__
