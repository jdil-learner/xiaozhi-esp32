#!/usr/bin/env python3
"""
从 KanjiVG 构建统一字体数据 (ASCII + 3500常用汉字)
生成: C++ 头文件 + G-code 测试文件
"""

import os, re, math, json, sys

# ─── SVG 解析 ───
def parse_svg_path(d_str):
    tokens = re.findall(r'[a-zA-Z]|[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', d_str)
    points = []
    cur_x, cur_y = 0.0, 0.0
    i = 0
    while i < len(tokens):
        cmd = tokens[i]; i += 1
        if cmd == 'M':
            cur_x = float(tokens[i]); cur_y = float(tokens[i+1]); i += 2
            points.append((cur_x, cur_y))
        elif cmd == 'm':
            cur_x += float(tokens[i]); cur_y += float(tokens[i+1]); i += 2
            points.append((cur_x, cur_y))
        elif cmd == 'c':
            p1 = (cur_x + float(tokens[i]), cur_y + float(tokens[i+1])); i += 2
            p2 = (cur_x + float(tokens[i]), cur_y + float(tokens[i+1])); i += 2
            p3 = (cur_x + float(tokens[i]), cur_y + float(tokens[i+1])); i += 2
            sampled = sample_cubic((cur_x, cur_y), p1, p2, p3)
            points.extend(sampled[1:])
            cur_x, cur_y = p3
        elif cmd == 'C':
            p1 = (float(tokens[i]), float(tokens[i+1])); i += 2
            p2 = (float(tokens[i]), float(tokens[i+1])); i += 2
            p3 = (float(tokens[i]), float(tokens[i+1])); i += 2
            sampled = sample_cubic((cur_x, cur_y), p1, p2, p3)
            points.extend(sampled[1:])
            cur_x, cur_y = p3
        elif cmd == 'l':
            cur_x += float(tokens[i]); cur_y += float(tokens[i+1]); i += 2
            points.append((cur_x, cur_y))
        elif cmd == 'L':
            cur_x = float(tokens[i]); cur_y = float(tokens[i+1]); i += 2
            points.append((cur_x, cur_y))
        elif cmd in ('s','S'): i += 4
        elif cmd in ('q','Q','t','T'): i += 2
        elif cmd in ('a','A'): i += 7
        elif cmd in ('h','H','v','V'): i += 1
        else: break
    return points

def sample_cubic(p0, p1, p2, p3, n=15):
    result = []
    for t_idx in range(n + 1):
        t = t_idx / n
        u = 1 - t
        x = u**3*p0[0] + 3*u**2*t*p1[0] + 3*u*t**2*p2[0] + t**3*p3[0]
        y = u**3*p0[1] + 3*u**2*t*p1[1] + 3*u*t**2*p2[1] + t**3*p3[1]
        result.append((x, y))
    return result

def dp_simplify(points, eps=1.2):
    if len(points) <= 2: return points
    dmax, idx = 0, 0
    for i in range(1, len(points) - 1):
        d = pdist(points[i], points[0], points[-1])
        if d > dmax: dmax = d; idx = i
    if dmax > eps:
        l = dp_simplify(points[:idx+1], eps)
        r = dp_simplify(points[idx:], eps)
        return l[:-1] + r
    return [points[0], points[-1]]

def pdist(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    if dx == 0 and dy == 0: return math.hypot(p[0]-a[0], p[1]-a[1])
    t = max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
    return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)

def parse_char_svg(filepath):
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()
    vb = re.search(r'viewBox="0 0 (\d+) (\d+)"', content)
    vw, vh = (109, 109)
    if vb: vw, vh = int(vb.group(1)), int(vb.group(2))

    paths = re.search(r'id="kvg:StrokePaths[^"]*"(.*?)(?=</g>\s*<g id="kvg:StrokeNumbers)', content, re.DOTALL)
    if not paths:
        paths = re.search(r'id="kvg:StrokePaths[^"]*"(.*?)(?=</svg>)', content, re.DOTALL)

    strokes = []
    if paths:
        for m in re.finditer(r'<path[^>]*d="([^"]*)"', paths.group(1)):
            pts = parse_svg_path(m.group(1))
            if len(pts) >= 2:
                simple = dp_simplify(pts, 1.2)
                if len(simple) >= 2:
                    strokes.append(simple)
    return strokes, (vw, vh)

# ─── 归一化到 int8_t 坐标 ───
def normalize_strokes(strokes, viewbox):
    """归一化到 0..127 范围 (int8_t), 翻转 Y"""
    vw, vh = viewbox
    scale = 127.0 / max(vw, vh)

    result = []
    for s in strokes:
        ns = []
        for x, y in s:
            nx = int(round(x * scale))
            ny = int(round((vh - y) * scale))
            nx = max(0, min(127, nx))
            ny = max(0, min(127, ny))
            ns.append(nx)
            ns.append(ny)
        result.append(ns)
    return result

def strokes_to_bytes(strokes):
    """将归一化笔画转为字节数组格式: [n, x0, y0, x1, y1, ..., n, x0, ..., 0]"""
    data = []
    for s in strokes:
        data.append(len(s) // 2)  # 顶点数
        data.extend(s)
    data.append(0)  # 结束
    return data

# ─── 主流程 ───
SVG_DIR = "kanji"

# 读取 3500 常用汉字
with open("3500常用汉字.txt", 'r', encoding='utf-8') as f:
    cn_chars = [c for c in f.read() if not c.isspace()]

print(f"Chinese chars to process: {len(cn_chars)}")

# 构建所有字符列表
all_chars = {}
found = 0
missing = 0
missing_list = []

for ch in cn_chars:
    code = f"{ord(ch):05x}"
    fpath = os.path.join(SVG_DIR, f"{code}.svg")
    if os.path.exists(fpath):
        all_chars[ch] = code
        found += 1
    else:
        missing += 1
        missing_list.append(ch)

print(f"Chinese: {found} found, {missing} missing")

# ASCII 32-126
for ascii_code in range(32, 127):
    ch = chr(ascii_code)
    code = f"{ascii_code:05x}"
    fpath = os.path.join(SVG_DIR, f"{code}.svg")
    if os.path.exists(fpath):
        all_chars[ch] = code
        found += 1

print(f"Total characters: {len(all_chars)}")

# 批量处理
font_data = {}
total_strokes = 0
total_vertices = 0

for ch, code in sorted(all_chars.items(), key=lambda x: x[0]):
    fpath = os.path.join(SVG_DIR, f"{code}.svg")
    strokes, vb = parse_char_svg(fpath)
    if not strokes:
        continue
    norm = normalize_strokes(strokes, vb)
    data = strokes_to_bytes(norm)
    font_data[ch] = data
    total_strokes += len(norm)
    total_vertices += sum(len(s)//2 for s in norm)

print(f"Processed: {len(font_data)} characters")
print(f"Total strokes: {total_strokes}")
print(f"Total vertices: {total_vertices}")
print(f"Estimated data size: {sum(len(d) for d in font_data.values())} bytes")

if missing_list:
    print(f"\nMissing Chinese chars ({len(missing_list)}):")
    print(' '.join(missing_list[:50]))

print("\nDone processing!")
