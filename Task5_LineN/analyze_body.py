# -*- coding: utf-8 -*-
"""抠出红笔轨迹主体(comp0区域)，分析内部结构；检测格子线标定px/mm。"""
from PIL import Image

src = r"C:\Users\FifiS\Desktop\ArmMe2\Task5_LineN\redN_trace.jpg"
im = Image.open(src).convert("RGB")
W, H = im.size
px = im.load()

# 主体区域
X0, X1, Y0, Y1 = 709, 1660, 590, 998
red = [[0]*W for _ in range(H)]
for y in range(H):
    for x in range(W):
        r, g, b = px[x, y]
        if r > 140 and r - g > 40 and r - b > 40:
            red[y][x] = 1

# 主体内列直方图
colh = [0]*(X1-X0+1)
for y in range(Y0, Y1+1):
    for x in range(X0, X1+1):
        if red[y][x]:
            colh[x-X0] += 1
print("=== col hist (x=%d..%d) ===" % (X0, X1))
# 分段输出每 50px 的计数峰值，方便看 x 峰
for s in range(0, X1-X0+1, 40):
    seg = colh[s:s+40]
    if max(seg) > 0:
        peak = s + seg.index(max(seg))
        print("  x[%d..%d) peak_x=%d n=%d" % (X0+s, X0+s+40, X0+peak, max(seg)))

# 主体内行直方图
rowh = [0]*(Y1-Y0+1)
for y in range(Y0, Y1+1):
    n = 0
    for x in range(X0, X1+1):
        if red[y][x]:
            n += 1
    rowh[y-Y0] = n
print("=== row hist (y=%d..%d) ===" % (Y0, Y1))
for s in range(0, Y1-Y0+1, 40):
    seg = rowh[s:s+40]
    if max(seg) > 0:
        peak = s + seg.index(max(seg))
        print("  y[%d..%d) peak_y=%d n=%d" % (Y0+s, Y0+s+40, Y0+peak, max(seg)))

# 格子线检测：在主体上方区域找水平暗线（避开红色）
# 用 y=300..580 x=0..700 区域（干净格子区）
gray_th = 160
line_rows = []
for y in range(300, 580):
    dark = 0
    for x in range(0, 700, 2):
        r, g, b = px[x, y]
        if (r+g+b)/3 < gray_th:
            dark += 1
    if dark > 150:
        line_rows.append(y)
# 合并相邻行
rows_merged = []
for y in line_rows:
    if rows_merged and y - rows_merged[-1][1] <= 2:
        rows_merged[-1] = (rows_merged[-1][0], y)
    else:
        rows_merged.append((y, y))
print("=== horizontal grid lines y=300..580 ===")
prev = None
for s, e in rows_merged:
    y = (s+e)//2
    gap = y - prev if prev else None
    print("  grid_line y=%d gap=%d" % (y, gap))
    prev = y

# 垂直格线
line_cols = []
for x in range(0, 700):
    dark = 0
    for y in range(300, 580, 2):
        r, g, b = px[x, y]
        if (r+g+b)/3 < gray_th:
            dark += 1
    if dark > 70:
        line_cols.append(x)
cols_merged = []
for x in line_cols:
    if cols_merged and x - cols_merged[-1][1] <= 2:
        cols_merged[-1] = (cols_merged[-1][0], x)
    else:
        cols_merged.append((x, x))
print("=== vertical grid lines x=0..700 ===")
prev = None
for s, e in cols_merged:
    x = (s+e)//2
    gap = x - prev if prev else None
    print("  grid_line x=%d gap=%d" % (x, gap))
    prev = x
