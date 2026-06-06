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
        float       width_mm;    // 渲染宽度（横向模式）
        bool        is_space;    // 是否是空格（允许断行位置）
        bool        is_punct;    // 是否是中文标点（断行/断列，横向雕刻，竖向跳过）
        uint16_t    sample_cp;   // 第一个字符的 Unicode（用于错误提示）
    };

    // ─── 标点判断 ───
    static bool IsCjkPunctuation(uint16_t cp) {
        return cp == 0x3002 || cp == 0xFF0C || cp == 0x3001 ||
               cp == 0xFF01 || cp == 0xFF1F || cp == 0xFF1B || cp == 0xFF1A;
               // 。      ，       、       ！       ？       ；       ：
    }

    // ─── 完整 G-code 生成（支持横/竖向 + 自动换行/换列） ───
    static std::string GenerateGcode(const std::string& text,
                                     float size_mm, float x_start, float y_start,
                                     int power, int feed_rate,
                                     float line_spacing = 2.0f,
                                     bool vertical = false,
                                     float col_spacing = 2.0f) {
        std::string gcode;
        char buf[128];
        int len;

        const char* layout_name = vertical ? "Vertical" : "Horizontal";
        int slen = snprintf(buf, sizeof(buf),
            "; Text: \"%s\"  Size: %.1fmm  Layout: %s  Spacing: %.1f\n",
            text.c_str(), size_mm, layout_name,
            vertical ? col_spacing : line_spacing);
        gcode.append(buf, slen);
        gcode.append("G21 ; mm\n");
        gcode.append("G90 ; absolute\n");

        float ascii_scale = size_mm / ASCII_FONT_H;
        float hanzi_scale = size_mm / FONT_GRID_SIZE;
        const float MAX_W = ENGRAVING_AREA_WIDTH;
        const float MAX_H = ENGRAVING_AREA_HEIGHT;

        // ─── 扫描 ASCII 高度范围 ───
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

                if (IsCjkPunctuation(cp)) {
                    // 中文标点：断行/断列标记
                    LayoutUnit u;
                    u.start      = p;
                    u.byte_len   = (uint16_t)(scan - p);
                    u.width_mm   = hanzi_scale * 8.0f;   // 标点宽度约为半字宽
                    u.is_space   = false;
                    u.is_punct   = true;
                    u.sample_cp  = cp;
                    units.push_back(u);
                    p = scan;
                } else if (cp < 128) {
                    if (cp == ' ') {
                        LayoutUnit u;
                        u.start     = p;
                        u.byte_len  = byte_len;
                        u.width_mm  = 4.0f * ascii_scale;
                        u.is_space  = true;
                        u.is_punct  = false;
                        u.sample_cp = ' ';
                        units.push_back(u);
                        p = scan;
                    } else {
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
                        u.start     = word_start;
                        u.byte_len  = (uint16_t)(p - word_start);
                        u.width_mm  = word_w;
                        u.is_space  = false;
                        u.is_punct  = false;
                        u.sample_cp = first_cp;
                        units.push_back(u);
                    }
                } else {
                    uint8_t cw;
                    LookupChar(cp, cw);
                    float ch_w = (cw > 0) ? cw * hanzi_scale : 8.0f * hanzi_scale;
                    LayoutUnit u;
                    u.start     = p;
                    u.byte_len  = (uint16_t)(scan - p);
                    u.width_mm  = ch_w;
                    u.is_space  = false;
                    u.is_punct  = false;
                    u.sample_cp = cp;
                    units.push_back(u);
                    p = scan;
                }
            }
        }

        if (vertical) {
            // ═══════════════════ 竖向排版 ═══════════════════
            // 从上到下，从右到左。标点触发断列但不雕刻；空格跳过不雕刻。
            // 每列仅含汉字/数字，以标点或高度超限为断列条件。

            const float char_h = size_mm;  // 每个字占的高度
            const float col_dx = char_h + col_spacing;

            struct Col {
                int unit_start;
                int unit_count;
                int char_count;   // 实际要雕刻的字符数(不含标点)
                float col_h;
            };
            std::vector<Col> cols;

            int col_start = 0;
            int cur_chars = 0;
            for (int i = 0; i < (int)units.size(); i++) {
                auto& u = units[i];
                if (u.is_space) continue;  // 跳过空格

                if (u.is_punct) {
                    // 标点触发断列（不雕刻，不留任何痕迹）
                    if (cur_chars > 0) {
                        cols.push_back({col_start, i - col_start, cur_chars,
                                        cur_chars * char_h});
                    }
                    col_start = i + 1;
                    cur_chars = 0;
                    continue;
                }

                // 竖向只支持汉字和数字（跳过 ASCII 单词）
                if (u.sample_cp < 128) continue;

                // 先确保单字是一个 unit（已是）
                if ((cur_chars + 1) * char_h > MAX_H && cur_chars > 0) {
                    cols.push_back({col_start, i - col_start, cur_chars,
                                    cur_chars * char_h});
                    col_start = i;
                    cur_chars = 1;
                } else {
                    cur_chars++;
                }
            }
            if (cur_chars > 0)
                cols.push_back({col_start, (int)units.size() - col_start, cur_chars,
                                cur_chars * char_h});

            // ─── 宽度校验 ───
            // 每列佔 size_mm 宽，列间间距 col_spacing
            float total_w = cols.size() * size_mm + (cols.size() - 1) * col_spacing;
            if (total_w < 0) total_w = 0;
            float render_x = x_start;
            if (x_start + total_w > MAX_W)
                render_x = x_start - total_w;

            if (render_x < 0 || render_x + total_w > MAX_W) {
                int overflow_col = 0;
                float acc_w = 0;
                for (int ci = 0; ci < (int)cols.size(); ci++) {
                    acc_w += col_dx;
                    if (render_x + acc_w > MAX_W) {
                        overflow_col = ci + 1; break;
                    }
                }
                auto& oc = cols[overflow_col > 0 ? overflow_col - 1 : 0];
                uint16_t ocp = units[oc.unit_start].sample_cp;
                char cb[8] = {};
                if (ocp < 0x800) { cb[0]=0xC0|(ocp>>6); cb[1]=0x80|(ocp&0x3F); }
                else { cb[0]=0xE0|(ocp>>12); cb[1]=0x80|((ocp>>6)&0x3F); cb[2]=0x80|(ocp&0x3F); }
                len = snprintf(buf, sizeof(buf),
                    "第%d列'%s'超出%.0fx%.0fmm范围。共%d列宽%.0fmm。请缩短文字或减小字号。",
                    overflow_col, cb, MAX_W, MAX_H, (int)cols.size(), total_w);
                return std::string(buf, len);
            }

            // ─── 逐列渲染（从右到左: 第0列最右） ───
            for (int ci = 0; ci < (int)cols.size(); ci++) {
                auto& col = cols[ci];
                float col_x = render_x + (cols.size() - 1 - ci) * col_dx;

                len = snprintf(buf, sizeof(buf),
                    "; Col %d: X=%.1f\n", ci + 1, col_x);
                gcode.append(buf, len);

                int rendered = 0;
                for (int ui = col.unit_start; ui < col.unit_start + col.unit_count; ui++) {
                    auto& u = units[ui];
                    if (u.is_punct || u.is_space) continue;
                    if (u.sample_cp < 128) continue;

                    float ch_y = y_start - size_mm - rendered * char_h;
                    const char* cp = u.start;
                    const char* cp_end = u.start + u.byte_len;
                    while (cp < cp_end) {
                        uint16_t ch_cp = DecodeUtf8(cp, cp_end);
                        len = snprintf(buf, sizeof(buf), "; Char: U+%04X\n", (unsigned int)ch_cp);
                        gcode.append(buf, len);
                        uint8_t cw;
                        const int8_t* d = LookupChar(ch_cp, cw);
                        if (!d || cw == 0) continue;
                        GenHanziChar(gcode, d, col_x, ch_y, hanzi_scale, power, feed_rate);
                    }
                    rendered++;
                }
            }

            gcode.append("G0 X0 Y0\n");
            len = snprintf(buf, sizeof(buf),
                "; Total: %d cols, %.1fmm x %.1fmm\n",
                (int)cols.size(), total_w, MAX_H);
            gcode.append(buf, len);
            return gcode;
        }

        // ═══════════════════ 横向排版（原有逻辑） ═══════════════════

        // ─── 第二步: 换行排版 ───
        struct Line {
            int unit_start;
            int unit_count;
            float line_w;
        };
        std::vector<Line> lines;

        int line_start = 0;
        float cur_w = 0.0f;
        for (int i = 0; i < (int)units.size(); i++) {
            auto& u = units[i];

            // 标点触发换行（标点留在当前行尾）
            if (u.is_punct && cur_w > 0) {
                cur_w += u.width_mm;
                lines.push_back({line_start, i - line_start + 1, cur_w});
                line_start = i + 1;
                cur_w = 0.0f;
                continue;
            }

            if (u.is_space && (i + 1 < (int)units.size())) {
                float next_w = units[i + 1].width_mm;
                if (cur_w + u.width_mm + next_w > MAX_W && cur_w > 0) {
                    lines.push_back({line_start, i - line_start, cur_w});
                    line_start = i;
                    cur_w = u.width_mm + next_w;
                    i++;
                } else {
                    cur_w += u.width_mm + next_w;
                    i++;
                }
            } else if (cur_w + u.width_mm > MAX_W && cur_w > 0) {
                lines.push_back({line_start, i - line_start, cur_w});
                line_start = i;
                cur_w = u.width_mm;
            } else {
                cur_w += u.width_mm;
            }
        }
        if (line_start < (int)units.size())
            lines.push_back({line_start, (int)units.size() - line_start, cur_w});

        // ─── 第三步: 高度校验 ───
        float row_h = size_mm + line_spacing;
        float total_h = lines.size() * row_h - line_spacing;

        float render_y = y_start;
        if (y_start + total_h > MAX_H)
            render_y = y_start - total_h;

        if (render_y < 0 || render_y + total_h > MAX_H) {
            int overflow_line = 0;
            float acc_h = 0;
            for (int li = 0; li < (int)lines.size(); li++) {
                acc_h += row_h;
                if (render_y + acc_h > MAX_H) { overflow_line = li + 1; break; }
            }
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
                "第%d行'%s'超出%.0fx%.0fmm范围。共%d行高%.0fmm。请缩短文字或减小字号。",
                overflow_line, ch_buf, MAX_W, MAX_H,
                (int)lines.size(), total_h);
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

            float x_cursor = 0.0f;

            for (int ui = line.unit_start; ui < line.unit_start + line.unit_count; ui++) {
                auto& u = units[ui];
                if (u.is_space) {
                    x_cursor += u.width_mm;
                    continue;
                }

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
            (int)lines.size(), MAX_W, total_h);
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
            "支持横竖两种排版方向:\n"
            "  \"横着雕刻\" → layout=\"horizontal\", 文字从左到右, 行从上到下\n"
            "  \"竖着雕刻\" → layout=\"vertical\", 文字从上到下, 列从右到左 (竹简风格)\n"
            "  默认横向。标点符号 (。，、！？；：) 横向触发换行并雕刻, 竖向触发换列且不雕刻。\n"
            "文字超出一行时自动换行, 行间距默认2mm。\n"
            "参数:\n"
            "  `text` 要雕刻的文字   `size` 字高mm(1-42)默认5   `power` 激光功率0-1000默认1000\n"
            "  `x` `y` 锚点坐标mm默认(0,0)  雕刻速度固定400mm/min\n"
            "  `line_spacing` 横向行间距mm(0-20)默认2\n"
            "  `layout` \"horizontal\"或\"vertical\"，默认\"horizontal\"\n"
            "  `col_spacing` 竖向列间距mm(0-20)默认2\n"
            "雕刻范围 42x42mm, 超出则返回错误。原点的位置是左下角，(0,42)是左上角，(42,0)是右下角，(42,42)是右上角。没有特定要求，不要擅自重复雕刻。结束后等待雕刻完成,不要回复其他无关内容",
            PropertyList({
                Property("text", kPropertyTypeString),
                Property("size", kPropertyTypeInteger, 5, 1, 42),
                Property("power", kPropertyTypeInteger, 1000, 0, 1000),
                Property("x", kPropertyTypeInteger, 0, 0, 42),
                Property("y", kPropertyTypeInteger, 0, 0, 42),
                Property("line_spacing", kPropertyTypeInteger, 2, 0, 20),
                Property("layout", kPropertyTypeString, std::string("horizontal")),
                Property("col_spacing", kPropertyTypeInteger, 2, 0, 20),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string text = properties["text"].value<std::string>();
                float size_mm       = static_cast<float>(properties["size"].value<int>());
                int power           = properties["power"].value<int>();
                int feed_rate       = 400;
                float x_start       = static_cast<float>(properties["x"].value<int>());
                float y_start       = static_cast<float>(properties["y"].value<int>());
                float line_spacing  = static_cast<float>(properties["line_spacing"].value<int>());
                std::string layout  = properties["layout"].value<std::string>();
                float col_spacing   = static_cast<float>(properties["col_spacing"].value<int>());

                bool vertical = (layout == "vertical");

                if (text.empty()) throw std::runtime_error("text 不能为空");
                if (x_start > ENGRAVING_AREA_WIDTH)
                    throw std::runtime_error("起始 X 坐标超出雕刻范围(0~42mm)");
                if (y_start > ENGRAVING_AREA_HEIGHT)
                    throw std::runtime_error("起始 Y 坐标超出雕刻范围(0~42mm)");

                std::string gcode = GenerateGcode(text, size_mm, x_start, y_start,
                                                  power, feed_rate, line_spacing,
                                                  vertical, col_spacing);
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
