# -*- coding: utf-8 -*-
"""逐行展开红笔轨迹主体，输出拓扑；生成放大可视PNG。"""
from PIL import Image

src = r"C:\Users\FifiS\Desktop\ArmMe2\Task5_LineN\redN_trace.jpg"
im = Image.open(src).convert("RGB")
W, H = im.size
px = im.load()

X0, X1, Y0, Y1 = 709, 1660, 590, 998
red = [[0]*W for _ in range(H)]
for y in range(H):
    for x in range(W):
        r, g, b = px[x, y]
        if r > 140 and r - g > 40 and r - b > 40:
            red[y][x] = 1

print("=== per-10-row span (x min->max, n) ===")
for y in range(Y0, Y1+1, 10):
    xs = [x for x in range(X0, X1+1) if red[y][x]]
    if xs:
        print("  y=%4d  x[%4d..%4d]  n=%3d" % (y, min(xs), max(xs), len(xs)))
    else:
        print("  y=%4d  (empty)" % y)

# 放大可视：主体区域 2x 白底红线
scale = 2
w = (X1-X0+1)*scale
h = (Y1-Y0+1)*scale
out = Image.new("RGB", (w, h), (255, 255, 255))
op = out.load()
for y in range(Y0, Y1+1):
    for x in range(X0, X1+1):
        if red[y][x]:
            for dy in range(scale):
                for dx in range(scale):
                    op[(x-X0)*scale+dx, (y-Y0)*scale+dy] = (255, 0, 0)
out.save(r"C:\Users\FifiS\Desktop\ArmMe2\Task5_LineN\red_body_zoom.png")
print("saved red_body_zoom.png size=%dx%d" % (w, h))
