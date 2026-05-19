#!/usr/bin/env python3
"""
构建统一字体数据: KanjiVG (ASCII 32-126) + Make Me a Hanzi (3500汉字)
输出: C++ 头文件 + G-code 测试文件
"""

import os, re, math, json, sys

GRID_SIZE = 127          # 归一化坐标范围 0-127
FONT_HEIGHT = 127.0      # 字体空间高度

# ========== KanjiVG SVG 解析 (from parse_kanjivg.py) ==========
def parse_svg_path(d_str):
    tokens = re.findall(r'[a-zA-Z]|[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', d_str)
    points = []; cur_x = cur_y = 0.0; i = 0
    while i < len(tokens):
        cmd = tokens[i]; i += 1
        if cmd == 'M': cur_x=float(tokens[i]); cur_y=float(tokens[i+1]); i+=2; points.append((cur_x,cur_y))
        elif cmd == 'm': cur_x+=float(tokens[i]); cur_y+=float(tokens[i+1]); i+=2; points.append((cur_x,cur_y))
        elif cmd == 'c':
            p1=(cur_x+float(tokens[i]),cur_y+float(tokens[i+1])); i+=2
            p2=(cur_x+float(tokens[i]),cur_y+float(tokens[i+1])); i+=2
            p3=(cur_x+float(tokens[i]),cur_y+float(tokens[i+1])); i+=2
            points.extend(sample_cubic((cur_x,cur_y),p1,p2,p3)[1:]); cur_x,cur_y=p3
        elif cmd == 'C':
            p1=(float(tokens[i]),float(tokens[i+1])); i+=2
            p2=(float(tokens[i]),float(tokens[i+1])); i+=2
            p3=(float(tokens[i]),float(tokens[i+1])); i+=2
            points.extend(sample_cubic((cur_x,cur_y),p1,p2,p3)[1:]); cur_x,cur_y=p3
        elif cmd == 'l': cur_x+=float(tokens[i]); cur_y+=float(tokens[i+1]); i+=2; points.append((cur_x,cur_y))
        elif cmd == 'L': cur_x=float(tokens[i]); cur_y=float(tokens[i+1]); i+=2; points.append((cur_x,cur_y))
        elif cmd in ('s','S'): i+=4
        elif cmd in ('q','Q','t','T'): i+=2
        elif cmd in ('a','A'): i+=7
        elif cmd in ('h','H','v','V'): i+=1
        else: break
    return points

def sample_cubic(p0,p1,p2,p3,n=12):
    r=[];
    for j in range(n+1):
        t=j/n; u=1-t
        r.append((u**3*p0[0]+3*u**2*t*p1[0]+3*u*t**2*p2[0]+t**3*p3[0], u**3*p0[1]+3*u**2*t*p1[1]+3*u*t**2*p2[1]+t**3*p3[1]))
    return r

def parse_kanjivg_svg(filepath):
    with open(filepath,'r',encoding='utf-8') as f: content=f.read()
    vb=re.search(r'viewBox="0 0 (\d+) (\d+)"',content)
    vw,vh=(109,109)
    if vb: vw,vh=int(vb.group(1)),int(vb.group(2))
    paths=re.search(r'id="kvg:StrokePaths[^"]*"(.*?)(?=</g>\s*<g id="kvg:StrokeNumbers)',content,re.DOTALL)
    if not paths: paths=re.search(r'id="kvg:StrokePaths[^"]*"(.*?)(?=</svg>)',content,re.DOTALL)
    strokes=[]
    if paths:
        for m in re.finditer(r'<path[^>]*d="([^"]*)"',paths.group(1)):
            pts=parse_svg_path(m.group(1))
            if len(pts)>=2: strokes.append(pts)
    return strokes,(vw,vh)

# ========== Douglas-Peucker ==========
def dp(points, eps=1.5):
    if len(points)<=2: return points
    dmax,idx=0,0
    for i in range(1,len(points)-1):
        d=perp_dist(points[i],points[0],points[-1])
        if d>dmax: dmax=d; idx=i
    if dmax>eps:
        l=dp(points[:idx+1],eps); r=dp(points[idx:],eps)
        return l[:-1]+r
    return [points[0],points[-1]]

def perp_dist(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    if dx==0 and dy==0: return math.hypot(p[0]-a[0],p[1]-a[1])
    t=max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)

# ========== 归一化 ==========
def normalize_strokes(strokes, vw, vh, flip_y=True):
    """归一化到 0..GRID_SIZE 范围"""
    all_pts = [(x,y) for s in strokes for x,y in s]
    min_x = min(p[0] for p in all_pts)
    max_x = max(p[0] for p in all_pts)
    min_y = min(p[1] for p in all_pts)
    max_y = max(p[1] for p in all_pts)

    ch_w = max_x - min_x
    ch_h = max_y - min_y
    if ch_w <= 0: ch_w = vw
    if ch_h <= 0: ch_h = vh

    # 统一高度归一化: 所有字缩放到相同高度, 宽字按比例扩展不溢出即可
    scale = GRID_SIZE / ch_h
    if ch_w * scale > GRID_SIZE * 2:
        scale = (GRID_SIZE * 2) / ch_w  # 极宽字防溢出

    result = []
    for s in strokes:
        ns = []
        for x, y in s:
            nx = int(round((x - min_x) * scale))
            if flip_y:
                ny = int(round((max_y - y) * scale))
            else:
                ny = int(round((y - min_y) * scale))
            nx = max(0, min(GRID_SIZE, nx))
            ny = max(0, min(GRID_SIZE, ny))
            ns.append((nx, ny))
        result.append(ns)
    return result

def strokes_to_array(strokes):
    """转为 C 数组格式: [n, x0, y0, x1, y1, ..., n2, ..., 0]"""
    data = []
    for s in strokes:
        if len(s) >= 2:
            data.append(len(s))
            for x, y in s:
                data.append(x)
                data.append(y)
    data.append(0)
    return data

def get_char_width(strokes):
    """计算字符宽度（归一化后）"""
    all_x = [x for s in strokes for x,_ in s]
    return max(all_x) - min(all_x) if all_x else GRID_SIZE

# ========== 从 generate_all.py 提取 Hershey ASCII 数据 ==========
import importlib.util
spec = importlib.util.spec_from_file_location("gen", "../font_test/generate_all.py")
gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gen)

# Hershey FONT dict 的 key 到字符的映射
HERSHEY_LOOKUP = {k: ch for ch, k, w in gen.LOOKUP}

# ========== 主流程 ==========
HANZI_FILE = "graphics.txt"
CHINESE_FILE = "3500常用汉字.txt"

font_data = {}
char_order = []

# --- 1. Hershey ASCII ---
print("=== Hershey ASCII ===")
ascii_count = 0
for ch, key, w_orig in gen.LOOKUP:
    if key not in gen.FONT: continue
    data = gen.FONT[key]
    # data 格式: [n, x0, y0, ..., n, ..., 0]
    # 先从 data 提取 strokes 列表
    strokes = []; idx = 0
    while data[idx] != 0:
        n = data[idx]; idx += 1
        s = [(data[idx+i*2], data[idx+i*2+1]) for i in range(n)]
        strokes.append(s)
        idx += n * 2
    # Hershey ASCII: 保持原始坐标 (由控制器处理 Y 翻转和缩放)
    if not strokes:
        if w_orig > 0:
            font_data[ch] = ([], max(w_orig, 1))
            char_order.append((ch, ch))
        continue
    norm = []
    for s in strokes:
        ns = []
        for x,y in s: ns.append((x, y))  # 原始坐标不作任何归一化
        norm.append(ns)
    w = w_orig if w_orig > 0 else get_char_width(norm)
    if w == 0: w = 1
    font_data[ch] = (norm, max(w, 1))
    char_order.append((ch, ch))
    ascii_count += 1
print(f"  ASCII: {ascii_count}")

# --- 2. Make Me a Hanzi 3500 Chinese ---
print("=== Make Me a Hanzi Chinese ===")
with open(CHINESE_FILE, 'r', encoding='utf-8') as f:
    cn_chars = [c for c in f.read() if not c.isspace()]

# Build lookup for graphics.txt
print("  Loading graphics.txt...")
hanzi_data = {}
with open(HANZI_FILE, 'r', encoding='utf-8') as f:
    for line in f:
        d = json.loads(line)
        hanzi_data[d['character']] = d['medians']
print(f"  graphics.txt: {len(hanzi_data)} chars")

cn_count = 0
for ch in cn_chars:
    if ch not in hanzi_data:
        continue
    medians = hanzi_data[ch]
    if not medians:
        continue
    # Simplify
    simple = [dp(s, 2.5) for s in medians if len(s) >= 2]
    simple = [s for s in simple if len(s) >= 2]
    if not simple:
        continue
    # Normalize (NO flip: Make Me a Hanzi Y+ up, same as G-code)
    norm = normalize_strokes(simple, 1024, 1024, flip_y=False)
    w = get_char_width(norm)
    font_data[ch] = (norm, w)
    char_order.append((ch, ch))
    cn_count += 1

print(f"  Chinese: {cn_count}")
print(f"  Total characters: {len(font_data)}")

# ========== 生成 C++ 头文件 ==========
print("\n=== Generating C++ header ===")

# Sort by ASCII value
char_order.sort(key=lambda x: ord(x[0]))

SYMBOL_KEYS = {c for c, k in char_order if k != c}

cpp = """#ifndef __FONT_DATA_H__
#define __FONT_DATA_H__

#include <cstdint>

// 统一字体数据: KanjiVG (ASCII) + Make Me a Hanzi (汉字)
// KanjiVG: Copyright (C) Ulrich Apel, CC BY-SA 3.0
// Make Me a Hanzi: derived from Arphic fonts, Arphic Public License
//
// 编码: 每笔画以顶点数 N 开头, 后跟 N 对 (x,y) int8_t 坐标 (0-127)
// 笔画间自动抬刀, N=0 标记字符结束

#define FONT_GRID_SIZE 127.0f
#define FONT_STROKE_END 0

struct FontChar {
    uint16_t code;       // Unicode 码点
    const int8_t* strokes;
    uint8_t width;        // 字符宽度 (归一化坐标)
};

inline const FontChar* font_lookup_table(int& count) {
"""

# Write stroke arrays
for ch, key in char_order:
    norm, w = font_data[key]
    arr = strokes_to_array(norm)
    cpp_name = f"_F_{ord(ch):04X}"
    cpp += f'    static const int8_t {cpp_name}[] = {{\n'
    items = [str(d) for d in arr]
    line = '        '
    for val in items:
        test = line + val + ', '
        if len(test.rstrip()) > 85 and line.strip():
            cpp += line.rstrip() + '\n'
            line = '        ' + val + ', '
        else:
            line += val + ', '
    cpp += line.rstrip(', ') + '\n'
    cpp += '    };\n\n'

# Write lookup table
cpp += '    static const FontChar table[] = {\n'
for ch, key in char_order:
    norm, w = font_data[key]
    cpp_name = f"_F_{ord(ch):04X}"
    cpp += f"        {{0x{ord(ch):04X}, {cpp_name}, {w}}},\n"
cpp += '    };\n\n'
cpp += '    count = sizeof(table) / sizeof(FontChar);\n'
cpp += '    return table;\n'
cpp += '}\n\n'
cpp += '#endif // __FONT_DATA_H__\n'

with open('../main/boards/common/font_data.h', 'w', encoding='utf-8') as f:
    f.write(cpp)
print("  -> main/boards/common/font_data.h")

# ========== 数据统计 ==========
total_bytes = sum(len(strokes_to_array(font_data[k][0])) for k in font_data)
total_strokes = sum(len(font_data[k][0]) for k in font_data)
total_vertices = sum(sum(len(s) for s in font_data[k][0]) for k in font_data)
print(f"\n  Size: {total_bytes} bytes, {total_strokes} strokes, {total_vertices} vertices")

# ========== 生成 G-code 测试文件 ==========
print("\n=== Generating G-code test files ===")
SIZE_MM = 10.0; POWER = 800; FEED = 800

def to_gcode(ch, strokes, w):
    scale = SIZE_MM / FONT_HEIGHT
    lines = [f"; {ch}  KanjiVG/Hanzi  {SIZE_MM}mm"]
    lines.append("G21 ; mm"); lines.append("G90 ; absolute")
    for s in strokes:
        simple = dp(s, 1.0)
        for i, (x, y) in enumerate(simple):
            gx = x * scale
            gy = y * scale  # 已经归一化到 Y+ 向上
            if i == 0:
                lines.append(f"G0 X{gx:.3f} Y{gy:.3f}")
                lines.append(f"M3 S{POWER}")
            else:
                lines.append(f"G1 X{gx:.3f} Y{gy:.3f} F{FEED}")
        lines.append("M5")
    lines.append("G0 X0 Y0\n")
    return "\n".join(lines)

# Generate a few test files
os.makedirs("gcode_test", exist_ok=True)
test = ["A","B","C","0","1","2","!","@","不","中","国","你","好","永","世","界"]
for ch in test:
    for c, k in char_order:
        if c == ch:
            norm, w = font_data[k]
            gcode = to_gcode(ch, norm, w)
            fname = f"gcode_test/{k if k != ch else ch}_{ord(ch):04X}_Gcode.nc"
            with open(fname, 'w', encoding='utf-8') as f:
                f.write(gcode)
            print(f"  {fname}")
            break
    else:
        print(f"  {ch}: NOT FOUND")

print(f"\nDone! {len(font_data)} characters in font.")
