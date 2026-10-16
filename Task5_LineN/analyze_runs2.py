# -*- coding: utf-8 -*-
"""最新轨迹：每行分离线段（run）分析，分辨多条轨迹线。"""
from PIL import Image

src = r"C:\Users\FifiS\Desktop\ArmMe2\Task5_LineN\redN_trace2.jpg"
im = Image.open(src).convert("RGB")
W, H = im.size
px = im.load()

X0, X1, Y0, Y1 = 406, 2245, 344, 1036
red = [[0]*W for _ in range(H)]
for y in range(H):
    for x in range(W):
        r, g, b = px[x, y]
        if r > 140 and r - g > 40 and r - b > 40:
            red[y][x] = 1

print("=== row runs (每14行) ===")
for y in range(Y0, Y1+1, 14):
    runs = []
    inrun = False
    start = 0
    for x in range(X0, X1+1):
        if red[y][x]:
            if not inrun:
                start = x; inrun = True
        else:
            if inrun:
                runs.append((start, x-1)); inrun = False
    if inrun:
        runs.append((start, X1))
    # 合并相邻 <8px 的 run
    merged = []
    for s, e in runs:
        if merged and s - merged[-1][1] <= 8:
            merged[-1] = (merged[-1][0], e)
        else:
            merged.append((s, e))
    if merged:
        desc = "  ".join("x[%d..%d]" % (s, e) for s, e in merged)
        print("  y=%4d (%d条): %s" % (y, len(merged), desc))
