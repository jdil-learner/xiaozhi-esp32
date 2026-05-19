#ifndef __GCODE_CONTROLLER_H__
#define __GCODE_CONTROLLER_H__

#include "mcp_server.h"
#include "font_data.h"

#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <cstring>
#include <string>
#include <cmath>

#define ENGRAVING_AREA_WIDTH  42.0f
#define ENGRAVING_AREA_HEIGHT 42.0f

class KanjiVGController {
private:
    static constexpr uart_port_t UART_PORT  = UART_NUM_2;
    static constexpr gpio_num_t UART_TX_PIN = GPIO_NUM_20;
    static constexpr gpio_num_t UART_RX_PIN = GPIO_NUM_19;
    static constexpr int UART_BAUD = 115200;

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

    // ─── ASCII 单字 (Hershey 原始逻辑) ───
    static void GenAsciiChar(std::string& gcode, const int8_t* data,
                             float& x_cursor, float y_base,
                             float scale, int power, int feed_rate) {
        // Hershey 坐标 Y+ 向下, 扫描 min_x 和 max_y
        int8_t min_x = 0, max_y = -128;
        {
            const int8_t* scan = data;
            while (*scan != FONT_STROKE_END) {
                int8_t n = *scan++;
                for (int i = 0; i < n; i++) {
                    int8_t sx = scan[i * 2], sy = scan[i * 2 + 1];
                    if (sx < min_x) min_x = sx;
                    if (sy > max_y) max_y = sy;
                }
                scan += n * 2;
            }
        }

        char buf[128];
        float x_off = x_cursor - min_x * scale;
        float y_off = y_base + max_y * scale;  // 翻转后 max_y 对应底部

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
        int8_t min_x = 127, min_y = 127;
        {
            const int8_t* scan = data;
            while (*scan != FONT_STROKE_END) {
                int8_t n = *scan++;
                for (int i = 0; i < n; i++) {
                    int8_t sx = scan[i * 2], sy = scan[i * 2 + 1];
                    if (sx < min_x) min_x = sx;
                    if (sy < min_y) min_y = sy;
                }
                scan += n * 2;
            }
        }

        char buf[128];
        float x_off = x_cursor - min_x * scale;
        float y_off = y_base - min_y * scale;

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

        // 预估总宽度
        float total_width = 0.0f;
        {
            const char* p = text.c_str();
            const char* end = p + text.size();
            while (p < end) {
                uint16_t cp = DecodeUtf8(p, end);
                uint8_t w;
                const int8_t* d = LookupChar(cp, w);
                float s = (cp < 128) ? ascii_scale : hanzi_scale;
                total_width += (d && w > 0) ? w * s : 8.0f * s;
            }
        }
        float total_height = size_mm;

        if (x_start + total_width > ENGRAVING_AREA_WIDTH ||
            y_start + total_height > ENGRAVING_AREA_HEIGHT) {
            int len2 = snprintf(buf, sizeof(buf),
                "{\"error\":\"超出雕刻范围(42x42mm): 文字宽%.1fmm,高%.1fmm,起点(%.0f,%.0f)\"}",
                total_width, total_height, x_start, y_start);
            return std::string(buf, len2);
        }

        // 逐字生成
        float x_cursor = x_start;
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
                GenAsciiChar(gcode, d, x_cursor, y_start, ascii_scale, power, feed_rate);
                x_cursor += cw * ascii_scale;
            } else {
                GenHanziChar(gcode, d, x_cursor, y_start, hanzi_scale, power, feed_rate);
                x_cursor += cw * hanzi_scale;
            }
        }

        gcode.append("G0 X0 Y0\n");
        return gcode;
    }

public:
    KanjiVGController() {
        uart_config_t uart_config = {
            .baud_rate = UART_BAUD,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        };
        ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 2048, 0, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                                      UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        ESP_LOGI("KanjiVG", "UART2 initialized: TX=IO%d, RX=IO%d, baud=%d",
                 UART_TX_PIN, UART_RX_PIN, UART_BAUD);
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
            "符号口述→实际字符: \"at\"\"艾特\"→@ \"美元\"→$ \"井号\"→# \"百分号\"→% \"和\"\"and\"→&\n"
            "  \"星号\"→* \"加号\"→+ \"减号\"\"横线\"→- \"点\"\"句号\"→. \"斜杠\"→/ \"冒号\"→:\n"
            "  \"问号\"→? \"感叹号\"→! \"括号\"→() \"小于\"→< \"大于\"→> \"等于\"→=\n"
            "  \"引号\"→\" \"逗号\"→, \"分号\"→; \"下划线\"→_ \"竖线\"→|\n"
            "  \"波浪号\"→~ \"方括号\"→[] \"花括号\"→{} \"反斜杠\"→\\\n"
            "参数:\n"
            "  `text` 要雕刻的文字   `size` 字高mm(1-42)默认5   `power` 激光功率0-1000默认800\n"
            "  `feed_rate` 速度mm/min默认800   `x` `y` 起始位置mm默认(0,0)\n"
            "雕刻范围 42x42mm, 超出则返回错误。",
            PropertyList({
                Property("text", kPropertyTypeString),
                Property("size", kPropertyTypeInteger, 5, 1, 42),
                Property("power", kPropertyTypeInteger, 800, 0, 1000),
                Property("feed_rate", kPropertyTypeInteger, 800, 100, 5000),
                Property("x", kPropertyTypeInteger, 0, 0, 40),
                Property("y", kPropertyTypeInteger, 0, 0, 40),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string text = properties["text"].value<std::string>();
                float size_mm     = static_cast<float>(properties["size"].value<int>());
                int power         = properties["power"].value<int>();
                int feed_rate     = properties["feed_rate"].value<int>();
                float x_start     = static_cast<float>(properties["x"].value<int>());
                float y_start     = static_cast<float>(properties["y"].value<int>());

                if (text.empty()) throw std::runtime_error("text 不能为空");
                if (x_start >= ENGRAVING_AREA_WIDTH)
                    throw std::runtime_error("起始 X 坐标超出雕刻范围(0~42mm)");
                if (y_start >= ENGRAVING_AREA_HEIGHT)
                    throw std::runtime_error("起始 Y 坐标超出雕刻范围(0~42mm)");

                std::string gcode = GenerateGcode(text, size_mm, x_start, y_start, power, feed_rate);
                if (!gcode.empty() && gcode[0] == '{')
                    throw std::runtime_error(gcode);

                int sent = uart_write_bytes(UART_PORT, gcode.c_str(), gcode.size());
                ESP_LOGI("KanjiVG", "Sent %d/%d bytes via UART2", sent, gcode.size());

                int line_count = 0;
                for (char c : gcode) { if (c == '\n') line_count++; }

                cJSON* result = cJSON_CreateObject();
                cJSON_AddStringToObject(result, "text", text.c_str());
                cJSON_AddNumberToObject(result, "size_mm", size_mm);
                cJSON_AddNumberToObject(result, "power", power);
                cJSON_AddNumberToObject(result, "feed_rate", feed_rate);
                cJSON_AddNumberToObject(result, "x", x_start);
                cJSON_AddNumberToObject(result, "y", y_start);
                cJSON_AddNumberToObject(result, "lines", line_count);
                cJSON_AddStringToObject(result, "gcode", gcode.c_str());
                cJSON_AddStringToObject(result, "sent_to", "UART2");

                return result;
            });
    }
};

#endif // __GCODE_CONTROLLER_H__
