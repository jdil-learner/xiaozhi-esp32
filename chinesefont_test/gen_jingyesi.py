import json, math, os

GRID_SIZE = 127; FONT_HEIGHT = 127.0; SIZE_MM = 5.0; POWER = 1000; FEED = 400
MAX_W = 42.0; MAX_H = 50.0; LINE_SPACING = 2.0; COL_SPACING = 2.0

def dp(points, eps=1.5):
    if len(points) <= 2: return points
    dmax, idx = 0, 0
    for i in range(1, len(points)-1):
        dx = points[-1][0] - points[0][0]; dy = points[-1][1] - points[0][1]
        if dx == 0 and dy == 0:
            d = math.hypot(points[i][0]-points[0][0], points[i][1]-points[0][1])
        else:
            t = max(0, min(1, ((points[i][0]-points[0][0])*dx + (points[i][1]-points[0][1])*dy)/(dx*dx+dy*dy)))
            d = math.hypot(points[i][0]-points[0][0]-t*dx, points[i][1]-points[0][1]-t*dy)
        if d > dmax: dmax, idx = d, i
    if dmax > eps:
        l = dp(points[:idx+1], eps); r = dp(points[idx:], eps)
        return l[:-1] + r
    return [points[0], points[-1]]

def normalize_strokes(strokes):
    all_pts = [(x,y) for s in strokes for x,y in s]
    mnx,mxx=min(p[0]for p in all_pts),max(p[0]for p in all_pts)
    mny,mxy=min(p[1]for p in all_pts),max(p[1]for p in all_pts)
    ch_h,ch_w=mxy-mny,mxx-mnx
    if ch_h<=0:ch_h=1
    if ch_w<=0:ch_w=1
    sc=GRID_SIZE/ch_h
    if ch_w*sc>GRID_SIZE:sc=GRID_SIZE/ch_w
    result=[]
    for st in strokes:
        ns=[(max(0,min(GRID_SIZE,int(round((x-mnx)*sc)))),
             max(0,min(GRID_SIZE,int(round((y-mny)*sc)))))for x,y in st]
        result.append(ns)
    return result

hanzi={}
with open('graphics.txt',encoding='utf-8')as f:
    for line in f:d=json.loads(line);hanzi[d['character']]=d['medians']

font={}
for ch,med in hanzi.items():
    if not med:continue
    s=[dp(si,2.5)for si in med if len(si)>=2]
    s=[si for si in s if len(si)>=2]
    if not s:continue
    n=normalize_strokes(s)
    if ch=='口':
        sf=0.7
        ax=[x for st in n for x,_ in st];ay=[y for st in n for _,y in st]
        ncx=(min(ax)+max(ax))/2.0;ncy=(min(ay)+max(ay))/2.0
        n=[[(max(0,min(GRID_SIZE,int(round((x-ncx)*sf+ncx)))),
             max(0,min(GRID_SIZE,int(round((y-ncy)*sf+ncy)))))for x,y in st]for st in n]
    font[ch]=(n,max(p[0]for st in n for p in st)-min(p[0]for st in n for p in st))

PUNCT = {0x3002, 0xFF0C, 0x3001, 0xFF01, 0xFF1F, 0xFF1B, 0xFF1A}

def gen_char(lines, ch, x_cursor, y_base):
    sc = SIZE_MM / FONT_HEIGHT
    if ch not in font: return x_cursor + 8.0 * sc
    strokes, cw = font[ch]
    mnx=127; mny=127; mxy=-128
    for st in strokes:
        for cx,cy in st:
            if cx<mnx:mnx=cx
            if cy<mny:mny=cy
            if cy>mxy:mxy=cy
    x_off=x_cursor-mnx*sc; y_off=y_base-mny*sc
    chh=(mxy-mny)*sc
    if chh<SIZE_MM*0.75:y_off+=(SIZE_MM-chh)*0.5
    lines.append('; Char: %s (U+%04X)'%(ch,ord(ch)))
    for st in strokes:
        for i,(cx,cy)in enumerate(st):
            gx=x_off+cx*sc; gy=y_off+cy*sc
            if i==0:lines.append('G0 X%.3f Y%.3f'%(gx,gy));lines.append('M3 S%d'%POWER)
            else:lines.append('G1 X%.3f Y%.3f F%d'%(gx,gy,FEED))
        lines.append('M5')
    return x_cursor + cw * sc

text = "床前明月光，疑是地上霜。举头望明月，低头思故乡。"

# ===== Horizontal =====
lines = ['; %s  Horizontal  %.0fmm  S%d'%(text,SIZE_MM,POWER), 'G21 ; mm', 'G90 ; absolute']
units = []
i = 0
while i < len(text):
    ch = text[i]; cp = ord(ch); 
    if cp in PUNCT:
        units.append({'ch':ch,'w':SIZE_MM/FONT_HEIGHT*8,'punct':True,'space':False,'cp':cp})
    elif ch == ' ':
        units.append({'ch':ch,'w':SIZE_MM/21.0*4,'punct':False,'space':True,'cp':cp})
    else:
        w = SIZE_MM/FONT_HEIGHT*8
        for fch,(_,fw) in font.items():
            if ord(fch)==cp: w = fw*SIZE_MM/FONT_HEIGHT; break
        units.append({'ch':ch,'w':w,'punct':False,'space':False,'cp':cp})
    i += 1

row_h = SIZE_MM + LINE_SPACING
all_lines = []; cur = []; cur_w = 0
for u in units:
    if u['punct'] and cur_w > 0:
        cur.append(u); all_lines.append(cur); cur = []; cur_w = 0; continue
    if cur_w + u['w'] > MAX_W and cur_w > 0:
        all_lines.append(cur); cur = [u]; cur_w = u['w']; continue
    cur.append(u); cur_w += u['w']
if cur: all_lines.append(cur)

total_h = len(all_lines) * row_h - LINE_SPACING
render_y = 5.0 if total_h <= MAX_H else MAX_H - total_h

for li, lu in enumerate(all_lines):
    ly = render_y + (len(all_lines)-1-li)*row_h
    lines.append('; Line %d: Y=%.1f'%(li+1,ly))
    xc = 0.0
    for u in lu:
        if u['space']: xc += u['w']; continue
        xc = gen_char(lines, u['ch'], xc, ly)
lines.append('G0 X0 Y0')
with open('gcode_test/静夜思_横版_Gcode.nc','w',encoding='utf-8')as f:f.write('\n'.join(lines))
print('横版 %d lines  %d lines  total_h=%.1f'%(len(lines), len(all_lines), total_h))

# ===== Vertical =====
lines = ['; %s  Vertical  %.0fmm  S%d'%(text,SIZE_MM,POWER), 'G21 ; mm', 'G90 ; absolute']
units = []
i = 0
while i < len(text):
    ch = text[i]; cp = ord(ch); 
    if cp in PUNCT: units.append({'ch':ch,'punct':True,'space':False,'cp':cp})
    elif ch == ' ': units.append({'ch':ch,'punct':False,'space':True,'cp':cp})
    else: units.append({'ch':ch,'punct':False,'space':False,'cp':cp})
    i += 1

cols = []; col_start = 0; cur_chars = 0
for ui, u in enumerate(units):
    if u['space']: continue
    if u['punct']:
        if cur_chars > 0: cols.append({'start':col_start,'count':ui-col_start,'chars':cur_chars}); cur_chars=0
        col_start = ui+1; continue
    if u['cp'] < 128: continue
    if (cur_chars+1)*SIZE_MM > MAX_H and cur_chars > 0:
        cols.append({'start':col_start,'count':ui-col_start,'chars':cur_chars}); col_start=ui; cur_chars=1
    else: cur_chars += 1
if cur_chars > 0: cols.append({'start':col_start,'count':len(units)-col_start,'chars':cur_chars})

total_w = len(cols) * SIZE_MM + (len(cols)-1) * COL_SPACING
# 锚点在右上角(42,50)，文字从右向左延伸
anchor_x = 42.0
render_x = anchor_x
if anchor_x + total_w > MAX_W:
    render_x = anchor_x - total_w

col_dx = SIZE_MM + COL_SPACING
for ci, col in enumerate(cols):
    col_x = render_x + (len(cols)-1-ci)*col_dx
    lines.append('; Col %d: X=%.1f'%(ci+1,col_x))
    rendered = 0
    for ui in range(col['start'], col['start']+col['count']):
        u = units[ui]
        if u['punct'] or u['space']: continue
        if u['cp'] < 128: continue
        ch_y = -rendered * SIZE_MM
        gen_char(lines, u['ch'], col_x, ch_y)
        rendered += 1
lines.append('G0 X0 Y0')
with open('gcode_test/静夜思_竖版_Gcode.nc','w',encoding='utf-8')as f:f.write('\n'.join(lines))
print('竖版 %d lines  %d cols  total_w=%.1f  render_x=%.1f'%(len(lines), len(cols), total_w, render_x))
