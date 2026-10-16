# -*- coding: utf-8 -*-
"""最新轨迹：逐行/逐列展开拓扑 + 放大可视PNG。"""
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

print("=== row span (每12行) ===")
for y in range(Y0, Y1+1, 12):
    xs = [x for x in range(X0, X1+1) if red[y][x]]
    if xs:
        print("  y=%4d  x[%4d..%4d]  w=%3d  n=%3d" % (y, min(xs), max(xs), max(xs)-min(xs), len(xs)))
    else:
        print("  y=%4d  (empty)" % y)

print("=== col span (每20列) ===")
for x in range(X0, X1+1, 20):
    ys = [y for y in range(Y0, Y1+1) if red[y][x]]
    if ys:
        print("  x=%4d  y[%4d..%4d]  h=%3d  n=%3d" % (x, min(ys), max(ys), max(ys)-min(ys), len(ys)))
    else:
        print("  x=%4d  (empty)" % x)

# 放大可视 1.6x
scale = 1
w = (X1-X0+1)*scale; h = (Y1-Y0+1)*scale
out = Image.new("RGB", (w, h), (255, 255, 255))
op = out.load()
for y in range(Y0, Y1+1):
    for x in range(X0, X1+1):
        if red[y][x]:
            op[x-X0, y-Y0] = (255, 0, 0)
out.save(r"C:\Users\FifiS\Desktop\ArmMe2\Task5_LineN\red_body2_zoom.png")
print("saved zoom %dx%d" % (w, h))
