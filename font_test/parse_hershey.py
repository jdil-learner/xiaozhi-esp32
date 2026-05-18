#!/usr/bin/env python3
"""从标准 rowmans.jhf 解析 Hershey Simplex Roman 字体数据"""

import os

with open('hershey-source/fonts/rowmans.jhf', 'r') as f:
    lines = f.readlines()

glyphs = {}
for line in lines:
    line = line.rstrip('\n')
    if len(line) < 10:
        continue
    try:
        gn = int(line[0:5])
    except:
        continue

    nv = int(line[5:8])
    left  = ord(line[8]) - ord('R')
    right = ord(line[9]) - ord('R')

    verts = []
    i = 10
    while i < len(line) - 1:
        x = ord(line[i]) - ord('R')
        y = ord(line[i+1]) - ord('R')
        verts.append((x, y))
        i += 2

    glyphs[gn] = {'nv': nv, 'left': left, 'right': right, 'verts': verts}

print(f"Parsed {len(glyphs)} glyphs")

def to_strokes(g):
    verts = g['verts']
    strokes = []
    cur = []
    for x, y in verts:
        if x == -50 and y == 0:
            if cur:
                strokes.append(cur)
                cur = []
        else:
            cur.append((x, y))
    if cur:
        strokes.append(cur)
    return strokes

def strokes_to_data(strokes):
    result = []
    for s in strokes:
        result.append(len(s))
        for x, y in s:
            result.append(x)
            result.append(y)
    result.append(0)
    return result

SYMBOL_NAMES = {
    32: 'SPC', 33: 'EXCL', 34: 'DQUOTE', 35: 'HASH', 36: 'DOLLAR',
    37: 'PERCENT', 38: 'AMPERSAND', 39: 'APOS', 40: 'LPAREN', 41: 'RPAREN',
    42: 'STAR', 43: 'PLUS', 44: 'COMMA', 45: 'MINUS', 46: 'DOT',
    47: 'SLASH', 58: 'COLON', 59: 'SEMICOLON', 60: 'LESS', 61: 'EQUAL',
    62: 'GREATER', 63: 'QUESTION', 64: 'AT',
    91: 'LBRACKET', 92: 'BACKSLASH', 93: 'RBRACKET',
    95: 'UNDERSCORE', 123: 'LBRACE', 124: 'PIPE', 125: 'RBRACE', 126: 'TILDE',
}

FONT = {}
LOOKUP = []

for ascii_code in range(32, 127):
    if ascii_code in glyphs:
        g = glyphs[ascii_code]
        strokes = to_strokes(g)
        data = strokes_to_data(strokes)
        width = g['right'] - g['left']
        ch = chr(ascii_code)

        if ascii_code in SYMBOL_NAMES:
            key = SYMBOL_NAMES[ascii_code]
        else:
            key = ch

        FONT[key] = data
        LOOKUP.append((ch, key, width))
        print(f"  '{ch}' ({key}): {len(data)}b, {len(strokes)} strokes, w={width}")
    else:
        print(f"  MISSING: ascii={ascii_code}")

print(f"\nTotal: {len(FONT)} characters")

# Generate Python script
py_out = '''#!/usr/bin/env python3
"""使用标准 Hershey Simplex Roman 字体数据生成每个字符的 G-code 文件"""

import os

HERSHEY_FONT_HEIGHT = 21.0

FONT = {
'''

for key in sorted(FONT.keys()):
    data = FONT[key]
    ekey = key.replace('\\', '\\\\').replace("'", "\\'")
    py_out += f"    '{ekey}': {data},\n"

py_out += '''}

LOOKUP = [
'''

def escape_ch(ch):
    if ch == '\\': return '\\\\'
    if ch == '\'': return '\\\''
    return ch

for ch, key, width in LOOKUP:
    ech = escape_ch(ch)
    ekey = key.replace('\\', '\\\\').replace("'", "\\'")
    py_out += f"    ('{ech}', '{ekey}', {width}),\n"

py_out += ''']

SYMBOL_NAMES = {
'''

for ascii_code, name in sorted(SYMBOL_NAMES.items()):
    ch = chr(ascii_code)
    ech = escape_ch(ch)
    py_out += f"    '{ech}': '{name}',\n"

py_out += '''}

SIZE_MM = 5.0
POWER = 800
FEED_RATE = 800


def gen_gcode(char, font_key, width):
    data = FONT[font_key]
    scale = SIZE_MM / HERSHEY_FONT_HEIGHT

    lines = []
    lines.append(f"; Laser Engraving G-code")
    lines.append(f"; Character: '{char}'  Size: {SIZE_MM}mm  Power: {POWER}")
    lines.append("G21 ; mm")
    lines.append("G90 ; absolute")

    if font_key == 'SPC':
        lines.append("; (space)")
        lines.append("G0 X0 Y0")
        lines.append("; End")
        return "\\n".join(lines) + "\\n"

    idx = 0
    while data[idx] != 0:
        n = data[idx]
        idx += 1

        for i in range(n):
            hx = data[idx + i * 2] * scale
            hy = data[idx + i * 2 + 1] * scale

            if i == 0:
                lines.append(f"G0 X{hx:.3f} Y{hy:.3f}")
                lines.append(f"M3 S{POWER}")
            else:
                lines.append(f"G1 X{hx:.3f} Y{hy:.3f} F{FEED_RATE}")

        lines.append("M5")
        idx += n * 2

    lines.append("G0 X0 Y0")
    lines.append("; End")
    return "\\n".join(lines) + "\\n"


def get_filename(char):
    if char in SYMBOL_NAMES:
        return f"{SYMBOL_NAMES[char]}_Gcode.nc"
    elif 'A' <= char <= 'Z':
        return f"{char}_Gcode.nc"
    elif 'a' <= char <= 'z':
        return f"{char}_lower_Gcode.nc"
    elif '0' <= char <= '9':
        return f"{char}_Gcode.nc"
    return f"{char}_Gcode.nc"


if __name__ == "__main__":
    out_dir = os.path.dirname(os.path.abspath(__file__))

    count = 0
    for char, font_key, width in LOOKUP:
        gcode = gen_gcode(char, font_key, width)
        fname = get_filename(char)
        fpath = os.path.join(out_dir, fname)
        with open(fpath, 'w', encoding='utf-8') as f:
            f.write(gcode)
        print(f"  {fname}")
        count += 1

    print(f"\\nGenerating finished: {count} files -> {out_dir}")
'''

with open('generate_all.py', 'w', encoding='utf-8') as f:
    f.write(py_out)
print("\ngenerate_all.py written!")

# Generate C++ header
cpp = '''#ifndef __HERSHEY_FONT_DATA_H__
#define __HERSHEY_FONT_DATA_H__

#include <cstdint>

// Hershey Simplex Roman (rowmans.jhf) — Public Domain
// Dr. Allen V. Hershey, U.S. National Bureau of Standards
//
// Encoding: each stroke begins with vertex count N,
// followed by N pairs of (x,y) int8_t coordinates.
// N=0 marks end of character. Pen-up between strokes is implicit.

#define HERSHEY_FONT_HEIGHT 21.0f
#define HERSHEY_STROKE_END  0

struct HersheyChar {
    char code;
    const int8_t* strokes;
    uint8_t width;
};

inline const HersheyChar* hershey_lookup_table(int& count) {
'''

for key in sorted(FONT.keys()):
    data = FONT[key]
    cpp_name = f'_CH_{key}'
    cpp += f'    static const int8_t {cpp_name}[] = {{\n'
    items = [str(d) for d in data]
    line = '        '
    for val in items:
        test = (line + val + ', ').rstrip()
        if len(test) > 85 and line.strip():
            cpp += line.rstrip() + '\n'
            line = '        ' + val + ', '
        else:
            line += val + ', '
    cpp += line.rstrip(', ') + '\n'
    cpp += '    };\n\n'

cpp += '    static const HersheyChar table[] = {\n'
def cpp_escape(ch):
    if ch == '\\': return '\\\\'
    if ch == '\'': return '\\\''
    return ch

for ch, key, width in LOOKUP:
    cpp_name = f'_CH_{key}'
    cp_ch = cpp_escape(ch)
    cpp += f"        {{'{cp_ch}', {cpp_name}, {width}}},\n"
cpp += '    };\n\n'
cpp += '    count = sizeof(table) / sizeof(HersheyChar);\n'
cpp += '    return table;\n'
cpp += '}\n\n'
cpp += '#endif // __HERSHEY_FONT_DATA_H__\n'

with open('../main/boards/common/hershey_font_data.h', 'w', encoding='utf-8') as f:
    f.write(cpp)
print(f"C++ header written: {len(FONT)} characters")
print("Done!")
