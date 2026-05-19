#!/usr/bin/env python3
"""测试 Make Me a Hanzi → G-code"""

import json, math

SIZE_MM = 10.0
POWER = 800
FEED = 800

def find_char(char):
    with open('graphics.txt', 'r', encoding='utf-8') as f:
        for line in f:
            d = json.loads(line)
            if d['character'] == char:
                return d['medians']
    return None

def dp(points, eps=2.0):
    if len(points) <= 2: return points
    dmax, idx = 0, 0
    for i in range(1, len(points)-1):
        dx = points[-1][0]-points[0][0]; dy = points[-1][1]-points[0][1]
        if dx==0 and dy==0: d = math.hypot(points[i][0]-points[0][0], points[i][1]-points[0][1])
        else:
            t = max(0,min(1,((points[i][0]-points[0][0])*dx+(points[i][1]-points[0][1])*dy)/(dx*dx+dy*dy)))
            px = points[0][0]+t*dx; py = points[0][1]+t*dy
            d = math.hypot(points[i][0]-px, points[i][1]-py)
        if d>dmax: dmax=d; idx=i
    if dmax>eps:
        l=dp(points[:idx+1],eps); r=dp(points[idx:],eps)
        return l[:-1]+r
    return [points[0],points[-1]]

def to_gcode(char, medians):
    # 找 min/max
    all_pts = [p for s in medians for p in s]
    min_x = min(p[0] for p in all_pts)
    max_y = max(p[1] for p in all_pts)  # Y翻转
    min_y = min(p[1] for p in all_pts)

    total_h = (max_y - min_y)
    scale = SIZE_MM / total_h if total_h > 0 else SIZE_MM / 1024

    lines = [f"; {char}  MakeMeAHanzi  {SIZE_MM}mm"]
    lines.append("G21 ; mm"); lines.append("G90 ; absolute")

    for s in medians:
        simple = dp(s, 2.5)
        for i, (x, y) in enumerate(simple):
            gx = (x - min_x) * scale
            gy = (y - min_y) * scale  # MakeMeAHanzi Y+向上，与G-code一致
            if i == 0:
                lines.append(f"G0 X{gx:.3f} Y{gy:.3f}")
                lines.append(f"M3 S{POWER}")
            else:
                lines.append(f"G1 X{gx:.3f} Y{gy:.3f} F{FEED}")
        lines.append("M5")
    lines.append("G0 X0 Y0\n")
    return "\n".join(lines)

for ch in ["不","中","国","你","好","永"]:
    med = find_char(ch)
    if med:
        gcode = to_gcode(ch, med)
        fname = f"{ch}_hanzi_Gcode.nc"
        with open(fname, 'w', encoding='utf-8') as f:
            f.write(gcode)
        pts = sum(len(s) for s in med)
        print(f"  {ch}: {len(med)} strokes, {pts} pts -> {fname}")
    else:
        print(f"  {ch}: NOT FOUND")
print("\nDone!")
