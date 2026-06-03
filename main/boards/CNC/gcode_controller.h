#ifndef __GCODE_CONTROLLER_H__
#define __GCODE_CONTROLLER_H__

#include "mcp_server.h"
#include "application.h"
#include "font_data.h"
#include "motion_controller.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <cstring>
#include <string>
#include <vector>
#include <cmath>

#define ENGRAVING_AREA_WIDTH  42.0f
#define ENGRAVING_AREA_HEIGHT 42.0f

class KanjiVGController {
private:
    static constexpr float ASCII_FONT_H = 21.0f;  // Hershey 基准高度

    // ─── UTF-8 解码 ───
    static uint16_t DecodeUtf8(const char*& p, const char* end) {
        uint8_t c = static_cast<uint8_t>(*p);
        if (c < 0x80) { p++; return c; }
        uint16_t cp; int len;
        if ((c & 0xE0) == 0xC0)      { cp = c & 0x1F; len = 1; }
        else if ((c & 0xF0) == 0xE0)  { cp = c & 0x0F; len = 2; }
        else if ((c & 0xF8) == 0xF0)  { cp = c & 0x07; len = 3; }
        else { p++; return 0; }
        p++;
        for (int i = 0; i < len && p < end; i++, p++)
            cp = (cp << 6) | (static_cast<uint8_t>(*p) & 0x3F);
        return cp;
    }

    // ─── 字体查找 ───
    static const int8_t* LookupChar(uint16_t unicode, uint8_t& out_width) {
        int count;
        const FontChar* table = font_lookup_table(count);
        for (int i = 0; i < count; i++) {
            if (table[i].code == unicode) {
                out_width = table[i].width;
                return table[i].strokes;
            }
        }
        out_width = 0;
        return nullptr;
    }

    // ─── ASCII 单字 (Hershey: Y+ 向下 → 翻转, 基线由调用者传入) ───
    static void GenAsciiChar(std::string& gcode, const int8_t* data,
                             float& x_cursor, float y_off,
                             float scale, int power, int feed_rate) {
        // Hershey 坐标 Y+ 向下, 扫描 min_x
        int8_t min_x = 0;
        {
            const int8_t* scan = data;
            while (*scan != FONT_STROKE_END) {
                int8_t n = *scan++;
                for (int i = 0; i < n; i++) {
                    int8_t sx = scan[i * 2];
                    if (sx < min_x) min_x = sx;
                }
                scan += n * 2;
            }
        }

        char buf[128];
        float x_off = x_cursor - min_x * scale;

        while (*data != FONT_STROKE_END) {
            int8_t n = *data++;
            for (int i = 0; i < n; i++) {
                float gx = x_off + static_cast<float>(data[i * 2]) * scale;
                float gy = y_off - static_cast<float>(data[i * 2 + 1]) * scale;  // Y 翻转
                if (i == 0) {
                    int len = snprintf(buf, sizeof(buf), "G0 X%.3f Y%.3f\n", gx, gy);
                    gcode.append(buf, len);
                    len = snprintf(buf, sizeof(buf), "M3 S%d\n", power);
                    gcode.append(buf, len);
                } else {
                    int len = snprintf(buf, sizeof(buf), "G1 X%.3f Y%.3f F%d\n", gx, gy, feed_rate);
                    gcode.append(buf, len);
                }
            }
            gcode.append("M5\n");
            data += n * 2;
        }
    }

    // ─── 汉字单字 (0~127 归一化, Y+ 向上) ───
    static void GenHanziChar(std::string& gcode, const int8_t* data,
                             float& x_cursor, float y_base,
                             float scale, int power, int feed_rate) {
        int8_t min_x = 127, min_y = 127, max_y = -128;
        {
            const int8_t* scan = data;
            while (*scan != FONT_STROKE_END) {
                int8_t n = *scan++;
                for (int i = 0; i < n; i++) {
                    int8_t sx = scan[i * 2], sy = scan[i * 2 + 1];
                    if (sx < min_x) min_x = sx;
                    if (sy < min_y) min_y = sy;
                    if (sy > max_y) max_y = sy;
                }
                scan += n * 2;
            }
        }

        char buf[128];
        float x_off = x_cursor - min_x * scale;
        float y_off = y_base - min_y * scale;

        // 宽字(如"一""心")按宽度缩放后高度不足，垂直居中使其与其他字对齐
        float ch_h = static_cast<float>(max_y - min_y) * scale;
        float nominal_h = FONT_GRID_SIZE * scale;
        if (ch_h < nominal_h * 0.75f)
            y_off += (nominal_h - ch_h) * 0.5f;

        while (*data != FONT_STROKE_END) {
            int8_t n = *data++;
            for (int i = 0; i < n; i++) {
                float gx = x_off + static_cast<float>(data[i * 2]) * scale;
                float gy = y_off + static_cast<float>(data[i * 2 + 1]) * scale;  // 无需翻转
                if (i == 0) {
                    int len = snprintf(buf, sizeof(buf), "G0 X%.3f Y%.3f\n", gx, gy);
                    gcode.append(buf, len);
                    len = snprintf(buf, sizeof(buf), "M3 S%d\n", power);
                    gcode.append(buf, len);
                } else {
                    int len = snprintf(buf, sizeof(buf), "G1 X%.3f Y%.3f F%d\n", gx, gy, feed_rate);
                    gcode.append(buf, len);
                }
            }
            gcode.append("M5\n");
            data += n * 2;
        }
    }

    // ─── 排版单元 ───
    struct LayoutUnit {
        const char* start;       // UTF-8 起始指针
        uint16_t    byte_len;    // 字节长度
        float       width_mm;    // 渲染宽度
        bool        is_space;    // 是否是空格（允许断行位置）
        uint16_t    sample_cp;   // 第一个字符的 Unicode（用于错误提示）
    };

    // ─── 完整 G-code 生成（支持自动换行） ───
    static std::string GenerateGcode(const std::string& text,
                                     float size_mm, float x_start, float y_start,
                                     int power, int feed_rate,
                                     float line_spacing = 2.0f) {
        std::string gcode;
        char buf[128];
        int len;

        int slen = snprintf(buf, sizeof(buf),
            "; Text: \"%s\"  Size: %.1fmm  Power: %d  LineSpacing: %.1f\n",
            text.c_str(), size_mm, power, line_spacing);
        gcode.append(buf, slen);
        gcode.append("G21 ; mm\n");
        gcode.append("G90 ; absolute\n");

        float ascii_scale = size_mm / ASCII_FONT_H;
        float hanzi_scale = size_mm / FONT_GRID_SIZE;
        const float MAX_LINE_W = ENGRAVING_AREA_WIDTH;

        // ─── 扫描 ASCII 高度范围（用于基线计算） ───
        int8_t ascii_max_y = -128, ascii_min_y = 127;
        bool has_ascii = false;
        {
            const char* p = text.c_str();
            const char* end = p + text.size();
            while (p < end) {
                uint16_t cp = DecodeUtf8(p, end);
                if (cp < 128) {
                    uint8_t w;
                    const int8_t* d = LookupChar(cp, w);
                    if (d) {
                        has_ascii = true;
                        const int8_t* scan = d;
                        while (*scan != FONT_STROKE_END) {
                            int8_t n = *scan++;
                            for (int i = 0; i < n; i++) {
                                int8_t sy = scan[i * 2 + 1];
                                if (sy > ascii_max_y) ascii_max_y = sy;
                                if (sy < ascii_min_y) ascii_min_y = sy;
                            }
                            scan += n * 2;
                        }
                    }
                }
            }
        }

        // ─── 第一步: 拆分为排版单元 ───
        std::vector<LayoutUnit> units;
        {
            const char* p = text.c_str();
            const char* end = p + text.size();
            while (p < end) {
                const char* scan = p;
                uint16_t cp = DecodeUtf8(scan, end);
                int byte_len = (int)(scan - p);

                if (cp < 128) {
                    // ASCII 字符：累积连续非空格字符为单词
                    if (cp == ' ') {
                        // 空格：独立单元，断行点
                        LayoutUnit u;
                        u.start      = p;
                        u.byte_len   = byte_len;
                        u.width_mm   = 4.0f * ascii_scale;  // 空格宽度约为 1/5 汉字
                        u.is_space   = true;
                        u.sample_cp  = ' ';
                        units.push_back(u);
                        p = scan;
                    } else {
                        // 非空格 ASCII：累积为单词
                        const char* word_start = p;
                        float word_w = 0.0f;
                        uint16_t first_cp = cp;
                        while (p < end && cp < 128 && cp != ' ') {
                            uint8_t cw;
                            LookupChar(cp, cw);
                            word_w += (cw > 0) ? cw * ascii_scale : 8.0f * ascii_scale;
                            p = scan;
                            if (p >= end) break;
                            scan = p;
                            cp = DecodeUtf8(scan, end);
                        }
                        LayoutUnit u;
                        u.start      = word_start;
                        u.byte_len   = (uint16_t)(p - word_start);
                        u.width_mm   = word_w;
                        u.is_space   = false;
                        u.sample_cp  = first_cp;
                        units.push_back(u);
                    }
                } else {
                    // 汉字：每个字独立一个单元
                    uint8_t cw;
                    LookupChar(cp, cw);
                    float ch_w = (cw > 0) ? cw * hanzi_scale : 8.0f * hanzi_scale;
                    LayoutUnit u;
                    u.start      = p;
                    u.byte_len   = (uint16_t)(scan - p);
                    u.width_mm   = ch_w;
                    u.is_space   = false;
                    u.sample_cp  = cp;
                    units.push_back(u);
                    p = scan;
                }
            }
        }

        // ─── 第二步: 换行排版 ───
        struct Line {
            int unit_start;   // 起始 unit 索引
            int unit_count;   // unit 数量
            float line_w;     // 总宽度
        };
        std::vector<Line> lines;

        int line_start = 0;
        float cur_w = 0.0f;
        for (int i = 0; i < (int)units.size(); i++) {
            auto& u = units[i];
            if (u.is_space && (i + 1 < (int)units.size())) {
                // 空格 + 下一个单词：检查下一个单词加入后是否超出
                float next_w = units[i + 1].width_mm;
                if (cur_w + u.width_mm + next_w > MAX_LINE_W && cur_w > 0) {
                    // 换行：空格及后面的单词移到下一行
                    lines.push_back({line_start, i - line_start, cur_w});
                    line_start = i;
                    cur_w = u.width_mm + next_w;
                    i++;  // 跳过下一个单词（已计入）
                } else {
                    cur_w += u.width_mm + next_w;
                    i++;
                }
            } else if (cur_w + u.width_mm > MAX_LINE_W && cur_w > 0) {
                // 当前单元导致溢出 → 换行
                lines.push_back({line_start, i - line_start, cur_w});
                line_start = i;
                cur_w = u.width_mm;
            } else {
                cur_w += u.width_mm;
            }
        }
        // 最后一行
        if (line_start < (int)units.size())
            lines.push_back({line_start, (int)units.size() - line_start, cur_w});

        // ─── 第三步: 高度校验 ───
        float row_h = size_mm + line_spacing;
        float total_h = lines.size() * row_h - line_spacing;  // 最后一行无行距

        // 根据锚点自动调整 Y 方向
        float render_y = y_start;
        if (y_start + total_h > ENGRAVING_AREA_HEIGHT)
            render_y = y_start - total_h;

        if (render_y < 0 || render_y + total_h > ENGRAVING_AREA_HEIGHT) {
            int overflow_line = 0;
            float acc_h = 0;
            for (int li = 0; li < (int)lines.size(); li++) {
                acc_h += row_h;
                if (render_y + acc_h > ENGRAVING_AREA_HEIGHT) {
                    overflow_line = li + 1;
                    break;
                }
            }
            // 提取溢出行的第一个字
            const auto& ol = lines[overflow_line > 0 ? overflow_line - 1 : 0];
            uint16_t overflow_cp = units[ol.unit_start].sample_cp;
            char ch_buf[8] = {};
            if (overflow_cp < 128) {
                ch_buf[0] = (char)overflow_cp; ch_buf[1] = 0;
            } else if (overflow_cp < 0x800) {
                ch_buf[0] = 0xC0 | (overflow_cp >> 6);
                ch_buf[1] = 0x80 | (overflow_cp & 0x3F);
            } else {
                ch_buf[0] = 0xE0 | (overflow_cp >> 12);
                ch_buf[1] = 0x80 | ((overflow_cp >> 6) & 0x3F);
                ch_buf[2] = 0x80 | (overflow_cp & 0x3F);
            }
            len = snprintf(buf, sizeof(buf),
                "第%d行'%s'超出42mm范围。共%d行高%.0fmm。请缩短文字或减小字号。",
                overflow_line, ch_buf, (int)lines.size(), total_h);
            return std::string(buf, len);
        }

        // ─── 第四步: 逐行生成 G-code ───
        float ascii_y_off_ref = y_start + ascii_max_y * ascii_scale;
        float ascii_y_off = ascii_y_off_ref + (render_y - y_start);

        for (int li = 0; li < (int)lines.size(); li++) {
            auto& line = lines[li];
            float line_y_base = render_y + (lines.size() - 1 - li) * row_h;
            float line_ascii_y = ascii_y_off + (line_y_base - render_y);

            len = snprintf(buf, sizeof(buf),
                "; Line %d: Y=%.1f\n", li + 1, line_y_base);
            gcode.append(buf, len);

            float x_cursor = 0.0f;  // 方案B: 每行从 x=0 开始

            for (int ui = line.unit_start; ui < line.unit_start + line.unit_count; ui++) {
                auto& u = units[ui];
                if (u.is_space) {
                    x_cursor += u.width_mm;
                    continue;
                }

                // 按字节遍历该单元内的每个字符
                const char* cp = u.start;
                const char* cp_end = u.start + u.byte_len;
                while (cp < cp_end) {
                    uint16_t ch_cp = DecodeUtf8(cp, cp_end);
                    len = snprintf(buf, sizeof(buf), "; Char: U+%04X\n", (unsigned int)ch_cp);
                    gcode.append(buf, len);

                    uint8_t cw;
                    const int8_t* d = LookupChar(ch_cp, cw);
                    if (d == nullptr || cw == 0) {
                        x_cursor += 8.0f * ((ch_cp < 128) ? ascii_scale : hanzi_scale);
                        continue;
                    }

                    if (ch_cp < 128) {
                        GenAsciiChar(gcode, d, x_cursor, line_ascii_y, ascii_scale, power, feed_rate);
                        x_cursor += cw * ascii_scale;
                    } else {
                        GenHanziChar(gcode, d, x_cursor, line_y_base, hanzi_scale, power, feed_rate);
                        x_cursor += cw * hanzi_scale;
                    }
                }
            }
        }

        gcode.append("G0 X0 Y0\n");
        len = snprintf(buf, sizeof(buf),
            "; Total: %d lines, %.1fmm x %.1fmm\n",
            (int)lines.size(), MAX_LINE_W, total_h);
        gcode.append(buf, len);
        return gcode;
    }

public:
    KanjiVGController() {
        ESP_LOGI("KanjiVG", "Initialized (local motion controller mode)");
    }

    void Initialize(McpServer& mcp_server) {
        mcp_server.AddTool("self.engraving.engrave_text",
            "生成激光雕刻 G-code 并发送给激光雕刻机。当用户要求雕刻/刻字时调用此工具。\n"
            "支持 3500 常用汉字、ASCII 英文字母、数字和符号。\n"
            "用户语音指令中必须在雕刻内容前加类型标记词, 你提取 `text` 时必须去掉该标记:\n"
            "  \"汉字XXX\" → text=\"XXX\" (如\"雕刻汉字你好\"→text=\"你好\")\n"
            "  \"英语XXX\" → text=\"XXX\" (如\"雕刻英语HELLO\"→text=\"HELLO\")\n"
            "  \"数字XXX\" → text=\"XXX\" (如\"雕刻数字123\"→text=\"123\")\n"
            "  \"符号XXX\" → text=对应符号 (如\"雕刻符号at\"→text=\"@\")\n"
            "  \"雕刻一个5mm的汉字我爱你\" → size=5, text=\"我爱你\"\n"
            "用户指定雕刻位置时, 提取 x/y 坐标作为锚点:\n"
            "  锚点是文字的定位参考点, 系统会自动选择文字从锚点延伸的方向, 确保不超出 42x42mm 范围。\n"
            "  四角坐标(可以使用近似值):\n"
            "    左下角(原点) → x=0, y=0      右下角 → x=42, y=0\n"
            "    左上角 → x=0, y=42            右上角 → x=42, y=42\n"
            "    中间 → x=21, y=21(文字居中于锚点附近)\n"
            "  如\"在左上角雕刻汉字一骑绝尘\" → x=0, y=42\n"
            "  如\"在右下角雕刻汉字大吉大利\" → x=42, y=0\n"
            "  如\"在中间雕刻汉字福\" → x=21, y=21\n"
            "文字超出一行时自动换行, 行间距默认2mm。\n"
            "参数:\n"
            "  `text` 要雕刻的文字   `size` 字高mm(1-42)默认5   `power` 激光功率0-1000默认1000\n"
            "  `x` `y` 锚点坐标mm默认(0,0)  雕刻速度固定400mm/min\n"
            "  `line_spacing` 行间距mm(0-20)默认2\n"
            "雕刻范围 42x42mm, 超出则返回错误。原点的位置是左下角，(0,42)是左上角，(42,0)是右下角，(42,42)是右上角。没有特定要求，不要擅自重复雕刻。结束后等待雕刻完成,不要回复其他无关内容",
            PropertyList({
                Property("text", kPropertyTypeString),
                Property("size", kPropertyTypeInteger, 5, 1, 42),
                Property("power", kPropertyTypeInteger, 1000, 0, 1000),
                Property("x", kPropertyTypeInteger, 0, 0, 42),
                Property("y", kPropertyTypeInteger, 0, 0, 42),
                Property("line_spacing", kPropertyTypeInteger, 2, 0, 20),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string text = properties["text"].value<std::string>();
                float size_mm       = static_cast<float>(properties["size"].value<int>());
                int power           = properties["power"].value<int>();
                int feed_rate       = 400;
                float x_start       = static_cast<float>(properties["x"].value<int>());
                float y_start       = static_cast<float>(properties["y"].value<int>());
                float line_spacing  = static_cast<float>(properties["line_spacing"].value<int>());

                if (text.empty()) throw std::runtime_error("text 不能为空");
                if (x_start > ENGRAVING_AREA_WIDTH)
                    throw std::runtime_error("起始 X 坐标超出雕刻范围(0~42mm)");
                if (y_start > ENGRAVING_AREA_HEIGHT)
                    throw std::runtime_error("起始 Y 坐标超出雕刻范围(0~42mm)");

                std::string gcode = GenerateGcode(text, size_mm, x_start, y_start,
                                                  power, feed_rate, line_spacing);
                if (!gcode.empty() && gcode[0] != ';')
                    throw std::runtime_error(gcode);

                int line_count = 0;
                for (char c : gcode) { if (c == '\n') line_count++; }

                // 启动后台雕刻任务
                struct Ctx {
                    std::string gcode;
                    std::string text;
                    int line_count;
                };
                auto* ctx = new Ctx{std::move(gcode), std::move(text), line_count};

                xTaskCreatePinnedToCore([](void* arg) {
                    auto* ctx = static_cast<Ctx*>(arg);
                    try {
                        MotionController::GlobalInit(106.666f, 106.666f, 700.0f, 800.0f, 0.02f, 42.0f);
                        vTaskDelay(pdMS_TO_TICKS(100));  // 等待步进驱动稳定，避免首次移动丢步
                        MotionController::Get().Execute(ctx->gcode);
                        Stepper::GoIdle();
                        ESP_LOGI("KanjiVG", "Motion done, %d lines", ctx->line_count);

                        // 通过 listen/detect 消息通知服务器雕刻完成
                        // 服务器收到后会将文字导入对话, 触发 LLM 生成 TTS 播报
                        Application::GetInstance().SendListenDetect("雕刻已完成：" + ctx->text);
                    } catch (const std::exception& e) {
                        ESP_LOGE("KanjiVG", "Engrave failed: %s", e.what());
                        Application::GetInstance().SendListenDetect("雕刻失败：" + std::string(e.what()));
                    }
                    delete ctx;
                    vTaskDelete(nullptr);
                }, "engrave", 8192, ctx, 5, nullptr, 1);

                ESP_LOGI("KanjiVG", "Engrave task started, %d lines", line_count);

                // 立即返回 "started", 服务器不会超时
                cJSON* ack = cJSON_CreateObject();
                cJSON_AddStringToObject(ack, "status", "started");
                cJSON_AddStringToObject(ack, "text", text.c_str());
                cJSON_AddNumberToObject(ack, "size_mm", size_mm);
                cJSON_AddNumberToObject(ack, "lines", line_count);
                return ack;
            });
    }
};

#endif // __GCODE_CONTROLLER_H__
