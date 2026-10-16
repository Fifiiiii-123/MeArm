# -*- coding: utf-8 -*-
"""分析红笔轨迹图：提取红色像素分布，量化轨迹位置。"""
from PIL import Image
import sys

src = r"C:\Users\FifiS\Desktop\ArmMe2\Task5_LineN\redN_trace.jpg"
im = Image.open(src).convert("RGB")
W, H = im.size
print("size:", W, H)
px = im.load()

# 红色掩膜：R 明显高于 G/B（红笔），排除黑字/铅笔/格子
red = [[0]*W for _ in range(H)]
red_px = 0
minx, miny, maxx, maxy = W, H, -1, -1
for y in range(H):
    for x in range(W):
        r, g, b = px[x, y]
        if r > 140 and r - g > 40 and r - b > 40:
            red[y][x] = 1
            red_px += 1
            if x < minx: minx = x
            if x > maxx: maxx = x
            if y < miny: miny = y
            if y > maxy: maxy = y
print("red_px:", red_px)
if red_px == 0:
    print("NO RED FOUND")
    sys.exit(0)
print("red bbox: x[%d,%d] y[%d,%d]  (w=%d h=%d)" % (minx, maxx, miny, maxy, maxx-minx, maxy-miny))

# 保存掩膜可视化（红=白 背景=黑）
out = Image.new("L", (W, H), 0)
op = out.load()
for y in range(H):
    for x in range(W):
        if red[y][x]:
            op[x, y] = 255
out.save(r"C:\Users\FifiS\Desktop\ArmMe2\Task5_LineN\red_mask.png")
print("mask saved")

# 列投影（每列红像素数）——看轨迹的 x 分布
colhist = [0]*W
for y in range(H):
    for x in range(W):
        if red[y][x]:
            colhist[x] += 1
# 找主要的 x 峰群
runs = []
inrun = False
start = 0
for x in range(W):
    if colhist[x] > 3:
        if not inrun:
            start = x; inrun = True
    else:
        if inrun:
            runs.append((start, x-1)); inrun = False
if inrun:
    runs.append((start, W-1))
# 合并邻近 runs
merged = []
for s, e in runs:
    if merged and s - merged[-1][1] <= 15:
        merged[-1] = (merged[-1][0], e)
    else:
        merged.append((s, e))
print("x clusters (red>3px):")
for s, e in merged:
    peak = max(range(s, e+1), key=lambda x: colhist[x])
    print("  x[%d,%d] w=%d peak_x=%d peak_n=%d" % (s, e, e-s, peak, colhist[peak]))

# 行投影（每行红像素数）——看轨迹的 y 分布
rowhist = [0]*H
for y in range(H):
    n = sum(red[y])
    rowhist[y] = n
runsy = []
inrun = False
start = 0
for y in range(H):
    if rowhist[y] > 3:
        if not inrun:
            start = y; inrun = True
    else:
        if inrun:
            runsy.append((start, y-1)); inrun = False
if inrun:
    runsy.append((start, H-1))
mergedy = []
for s, e in runsy:
    if mergedy and s - mergedy[-1][1] <= 15:
        mergedy[-1] = (mergedy[-1][0], e)
    else:
        mergedy.append((s, e))
print("y clusters:")
for s, e in mergedy:
    peak = max(range(s, e+1), key=lambda y: rowhist[y])
    print("  y[%d,%d] h=%d peak_y=%d peak_n=%d" % (s, e, e-s, peak, rowhist[peak]))
