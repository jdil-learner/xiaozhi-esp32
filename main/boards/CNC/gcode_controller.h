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

    // ─── 完整 G-code 生成 ───
    static std::string GenerateGcode(const std::string& text,
                                     float size_mm, float x_start, float y_start,
                                     int power, int feed_rate) {
        std::string gcode;
        char buf[128];

        int len = snprintf(buf, sizeof(buf),
            "; Text: \"%s\"  Size: %.1fmm  Power: %d\n",
            text.c_str(), size_mm, power);
        gcode.append(buf, len);
        gcode.append("G21 ; mm\n");
        gcode.append("G90 ; absolute\n");

        float ascii_scale = size_mm / ASCII_FONT_H;
        float hanzi_scale = size_mm / FONT_GRID_SIZE;

        // 预估总宽度 + ASCII 高度范围
        float total_width = 0.0f;
        int8_t ascii_max_y = -128, ascii_min_y = 127;
        bool has_ascii = false;
        {
            const char* p = text.c_str();
            const char* end = p + text.size();
            while (p < end) {
                uint16_t cp = DecodeUtf8(p, end);
                uint8_t w;
                const int8_t* d = LookupChar(cp, w);
                float s = (cp < 128) ? ascii_scale : hanzi_scale;
                total_width += (d && w > 0) ? w * s : 8.0f * s;
                if (cp < 128 && d) {
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
        float ascii_y_off_ref = y_start + ascii_max_y * ascii_scale;
        float ascii_height = has_ascii ? (ascii_max_y - ascii_min_y) * ascii_scale : 0;
        float total_height = (ascii_height > size_mm) ? ascii_height : size_mm;

        // ─── 根据锚点位置自动调整文字延伸方向 ───
        // 默认: 锚点为左下角, 文字向右(X+)上(Y+)延伸
        float render_x = x_start;
        float render_y = y_start;

        // 右侧超出 → 锚点变为右下角, 文字向左延伸
        if (x_start + total_width > ENGRAVING_AREA_WIDTH)
            render_x = x_start - total_width;
        // 上侧超出 → 锚点变为左上角, 文字向下延伸
        if (y_start + total_height > ENGRAVING_AREA_HEIGHT)
            render_y = y_start - total_height;

        // 边界校验
        if (render_x < 0 || render_y < 0 ||
            render_x + total_width > ENGRAVING_AREA_WIDTH ||
            render_y + total_height > ENGRAVING_AREA_HEIGHT) {
            int len2 = snprintf(buf, sizeof(buf),
                "文字\"%s\"大小%.0fmm超出范围。宽%.0f高%.0f。请缩短文字或减小字号。",
                text.c_str(), size_mm, total_width, total_height);
            return std::string(buf, len2);
        }

        // 根据偏移量调整 ASCII 和汉字的 Y 基准
        float ascii_y_off = ascii_y_off_ref + (render_y - y_start);
        float hanzi_y_base = render_y;

        int len3 = snprintf(buf, sizeof(buf),
            "; Anchor (%.0f,%.0f) → Render (%.0f,%.0f)\n",
            x_start, y_start, render_x, render_y);
        gcode.append(buf, len3);

        // 逐字生成
        float x_cursor = render_x;
        const char* p = text.c_str();
        const char* end = p + text.size();
        while (p < end) {
            uint16_t cp = DecodeUtf8(p, end);
            len = snprintf(buf, sizeof(buf), "; Char: U+%04X\n", (unsigned int)cp);
            gcode.append(buf, len);

            uint8_t cw;
            const int8_t* d = LookupChar(cp, cw);
            if (d == nullptr || cw == 0) {
                x_cursor += 8.0f * ((cp < 128) ? ascii_scale : hanzi_scale);
                continue;
            }

            if (cp < 128) {
                GenAsciiChar(gcode, d, x_cursor, ascii_y_off, ascii_scale, power, feed_rate);
                x_cursor += cw * ascii_scale;
            } else {
                GenHanziChar(gcode, d, x_cursor, hanzi_y_base, hanzi_scale, power, feed_rate);
                x_cursor += cw * hanzi_scale;
            }
        }

        gcode.append("G0 X0 Y0\n");
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
            "参数:\n"
            "  `text` 要雕刻的文字   `size` 字高mm(1-42)默认5   `power` 激光功率0-1000默认1000\n"
            "  `x` `y` 锚点坐标mm默认(0,0)  雕刻速度固定400mm/min\n"
            "雕刻范围 42x42mm, 超出则返回错误。原点的位置是左下角，(0,42)是左上角，(42,0)是右下角，(42,42)是右上角。没有特定要求，不要擅自重复雕刻。结束后等待雕刻完成,不要回复其他无关内容",
            PropertyList({
                Property("text", kPropertyTypeString),
                Property("size", kPropertyTypeInteger, 5, 1, 42),
                Property("power", kPropertyTypeInteger, 1000, 0, 1000),
                Property("x", kPropertyTypeInteger, 0, 0, 42),
                Property("y", kPropertyTypeInteger, 0, 0, 42),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string text = properties["text"].value<std::string>();
                float size_mm     = static_cast<float>(properties["size"].value<int>());
                int power         = properties["power"].value<int>();
                int feed_rate     = 400;
                float x_start     = static_cast<float>(properties["x"].value<int>());
                float y_start     = static_cast<float>(properties["y"].value<int>());

                if (text.empty()) throw std::runtime_error("text 不能为空");
                if (x_start > ENGRAVING_AREA_WIDTH)
                    throw std::runtime_error("起始 X 坐标超出雕刻范围(0~42mm)");
                if (y_start > ENGRAVING_AREA_HEIGHT)
                    throw std::runtime_error("起始 Y 坐标超出雕刻范围(0~42mm)");

                std::string gcode = GenerateGcode(text, size_mm, x_start, y_start, power, feed_rate);
                if (!gcode.empty() && gcode.compare(0, 6, "雕刻") == 0)
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
