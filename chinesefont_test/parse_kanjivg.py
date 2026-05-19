#!/usr/bin/env python3
"""
解析 KanjiVG SVG 笔画数据 → 生成 G-code
KanjiVG: Copyright (C) Ulrich Apel, CC BY-SA 3.0
"""

import os, re, math, glob

# ─── SVG Path 解析 ───
def parse_svg_path(d_str):
    """解析 SVG d 属性，采样为有序顶点列表"""
    # 提取命令和坐标
    tokens = re.findall(r'[a-zA-Z]|[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', d_str)

    points = []
    cur_x, cur_y = 0.0, 0.0
    i = 0

    while i < len(tokens):
        cmd = tokens[i]
        i += 1

        if cmd == 'M':  # 绝对移动
            cur_x = float(tokens[i]); cur_y = float(tokens[i+1])
            i += 2
            points.append((cur_x, cur_y))
        elif cmd == 'm':  # 相对移动
            cur_x += float(tokens[i]); cur_y += float(tokens[i+1])
            i += 2
            points.append((cur_x, cur_y))
        elif cmd == 'c':  # 相对三次贝塞尔
            pts = []
            for _ in range(3):
                pts.append((cur_x + float(tokens[i]), cur_y + float(tokens[i+1])))
                i += 2
            sampled = sample_cubic((cur_x, cur_y), pts[0], pts[1], pts[2])
            points.extend(sampled[1:])  # 跳过起点
            cur_x, cur_y = pts[2]
        elif cmd == 'C':  # 绝对三次贝塞尔
            pts = []
            for _ in range(3):
                pts.append((float(tokens[i]), float(tokens[i+1])))
                i += 2
            sampled = sample_cubic((cur_x, cur_y), pts[0], pts[1], pts[2])
            points.extend(sampled[1:])
            cur_x, cur_y = pts[2]
        elif cmd == 'l':  # 相对直线
            cur_x += float(tokens[i]); cur_y += float(tokens[i+1])
            i += 2
            points.append((cur_x, cur_y))
        elif cmd == 'L':  # 绝对直线
            cur_x = float(tokens[i]); cur_y = float(tokens[i+1])
            i += 2
            points.append((cur_x, cur_y))
        elif cmd in ('s', 'S', 'q', 'Q', 't', 'T', 'a', 'A', 'h', 'H', 'v', 'V', 'z', 'Z'):
            # 简化处理: 跳过其他命令
            if cmd in ('s', 'S'):
                i += 4
            elif cmd in ('q', 'Q', 't', 'T'):
                i += 2
            elif cmd in ('a', 'A'):
                i += 7
            elif cmd in ('h', 'H', 'v', 'V'):
                i += 1
        else:
            break

    return points

def sample_cubic(p0, p1, p2, p3, n=20):
    """在三次贝塞尔曲线上均匀采样 n 个点"""
    result = []
    for t_idx in range(n + 1):
        t = t_idx / n
        # Bezier: (1-t)^3*p0 + 3(1-t)^2*t*p1 + 3(1-t)*t^2*p2 + t^3*p3
        u = 1 - t
        x = u**3 * p0[0] + 3*u**2*t * p1[0] + 3*u*t**2 * p2[0] + t**3 * p3[0]
        y = u**3 * p0[1] + 3*u**2*t * p1[1] + 3*u*t**2 * p2[1] + t**3 * p3[1]
        result.append((x, y))
    return result

# ─── Douglas-Peucker 简化 ───
def dp_simplify(points, eps=1.0):
    if len(points) <= 2:
        return points
    dmax, idx = 0, 0
    for i in range(1, len(points) - 1):
        d = pdist(points[i], points[0], points[-1])
        if d > dmax:
            dmax = d; idx = i
    if dmax > eps:
        l = dp_simplify(points[:idx + 1], eps)
        r = dp_simplify(points[idx:], eps)
        return l[:-1] + r
    return [points[0], points[-1]]

def pdist(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    if dx == 0 and dy == 0:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    t = max(0, min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx*dx + dy*dy)))
    return math.hypot(p[0] - a[0] - t*dx, p[1] - a[1] - t*dy)

# ─── SVG 解析 ───
def parse_char_svg(filepath):
    """解析单个字符的 SVG，返回笔画列表"""
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()

    # 找到 viewBox
    vb_match = re.search(r'viewBox="0 0 (\d+) (\d+)"', content)
    vw, vh = 109, 109
    if vb_match:
        vw = int(vb_match.group(1))
        vh = int(vb_match.group(2))

    # 提取所有 stroke path（排除 StrokeNumbers 组）
    # 找到 StrokePaths 组
    paths_match = re.search(r'id="kvg:StrokePaths[^"]*"(.*?)(?=</g>\s*<g id="kvg:StrokeNumbers)', content, re.DOTALL)
    if not paths_match:
        paths_match = re.search(r'id="kvg:StrokePaths[^"]*"(.*?)(?=</svg>)', content, re.DOTALL)

    strokes = []
    if paths_match:
        paths_text = paths_match.group(1)
        # 提取所有 path d 属性
        for m in re.finditer(r'<path[^>]*d="([^"]*)"', paths_text):
            d_str = m.group(1)
            pts = parse_svg_path(d_str)
            if len(pts) >= 2:
                simplified = dp_simplify(pts, 1.5)
                strokes.append(simplified)

    return strokes, (vw, vh)

# ─── 生成 G-code ───
def to_gcode(char, strokes, viewbox):
    vw, vh = viewbox
    SIZE_MM = 10.0
    scale = SIZE_MM / max(vw, vh)
    POWER = 800
    FEED = 800

    lines = [f"; {char}  KanjiVG  {SIZE_MM}mm"]
    lines.append("G21 ; mm")
    lines.append("G90 ; absolute")

    for s in strokes:
        for i, (x, y) in enumerate(s):
            gx = x * scale
            gy = (vh - y) * scale  # 翻转 Y (SVG Y+朝下)
            if i == 0:
                lines.append(f"G0 X{gx:.3f} Y{gy:.3f}")
                lines.append(f"M3 S{POWER}")
            else:
                lines.append(f"G1 X{gx:.3f} Y{gy:.3f} F{FEED}")
        lines.append("M5")

    lines.append("G0 X0 Y0\n")
    return "\n".join(lines)

# ─── 主流程 ───
def process_char(char, svg_dir="kanji"):
    code = f"{ord(char):05x}"
    fpath = os.path.join(svg_dir, f"{code}.svg")

    if not os.path.exists(fpath):
        print(f"  {char}: SVG not found ({code})")
        return None

    strokes, vb = parse_char_svg(fpath)

    if not strokes:
        print(f"  {char}: no strokes extracted")
        return None

    gcode = to_gcode(char, strokes, vb)
    fname = f"{char}_{code}_Gcode.nc"
    with open(fname, 'w', encoding='utf-8') as f:
        f.write(gcode)

    total_v = sum(len(s) for s in strokes)
    print(f"  {char} ({code}): {len(strokes)} strokes, {total_v} vertices -> {fname}")
    return strokes

if __name__ == "__main__":
    test = ["不", "中", "国", "人", "大", "小", "你", "好", "世", "界", "永"]
    for ch in test:
        process_char(ch)
    print("\nDone!")
