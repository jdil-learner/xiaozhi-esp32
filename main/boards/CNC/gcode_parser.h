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
    float x, y;
    float feed_rate;
    int   spindle;
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
            while (p < end && (*p == '\n' || *p == '\r' || *p == ' ')) p++;
            if (p >= end) break;
            if (*p == ';') {
                while (p < end && *p != '\n') p++;
                continue;
            }

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

            while (p < end && *p != '\n' && *p != '\r') {
                if (*p == ' ') { p++; continue; }

                char param = *p;
                p++;

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

            if (std::isnan(cmd.x)) cmd.x = current_x;
            if (std::isnan(cmd.y)) cmd.y = current_y;
            if (cmd.feed_rate == 0) cmd.feed_rate = current_feed;
            if (cmd.spindle == 0 && cmd.type == GCodeCommand::M3)
                cmd.spindle = current_spindle;

            if (cmd.type == GCodeCommand::G0 || cmd.type == GCodeCommand::G1) {
                current_x = cmd.x;
                current_y = cmd.y;
                if (cmd.feed_rate > 0) current_feed = cmd.feed_rate;
            }
            if (cmd.type == GCodeCommand::M3 && cmd.spindle > 0) {
                current_spindle = cmd.spindle;
            }

            commands.push_back(cmd);

            while (p < end && *p != '\n') p++;
        }

        return commands;
    }
};

#endif // __GCODE_PARSER_H__
