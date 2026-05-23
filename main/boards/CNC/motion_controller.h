#ifndef __MOTION_CONTROLLER_H__
#define __MOTION_CONTROLLER_H__

#include "gcode_parser.h"
#include "planner.h"
#include "stepper.h"
#include "stepping_engine.h"
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
                    break;

                default:
                    break;
            }
        }

        // 等待 Planner 队列排空
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
        Planner::BufferLine(0.0f, 0.0f, Planner::max_rate, true);
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
