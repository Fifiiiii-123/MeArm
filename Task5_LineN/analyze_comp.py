# -*- coding: utf-8 -*-
"""连通域分析：找出红笔轨迹的主要连通块。"""
from PIL import Image
from collections import deque

src = r"C:\Users\FifiS\Desktop\ArmMe2\Task5_LineN\redN_trace.jpg"
im = Image.open(src).convert("RGB")
W, H = im.size
px = im.load()

red = [[0]*W for _ in range(H)]
for y in range(H):
    for x in range(W):
        r, g, b = px[x, y]
        if r > 140 and r - g > 40 and r - b > 40:
            red[y][x] = 1

# BFS 连通域（8邻域）
seen = [[False]*W for _ in range(H)]
comps = []
for y in range(H):
    for x in range(W):
        if red[y][x] and not seen[y][x]:
            q = deque([(x, y)])
            seen[y][x] = True
            comp = []
            while q:
                cx, cy = q.popleft()
                comp.append((cx, cy))
                for dy in (-1,0,1):
                    for dx in (-1,0,1):
                        nx, ny = cx+dx, cy+dy
                        if 0<=nx<W and 0<=ny<H and red[ny][nx] and not seen[ny][nx]:
                            seen[ny][nx] = True
                            q.append((nx, ny))
            comps.append(comp)

comps.sort(key=len, reverse=True)
print("total comps:", len(comps))
for i, c in enumerate(comps[:12]):
    xs = [p[0] for p in c]; ys = [p[1] for p in c]
    print("comp%d: n=%d bbox x[%d,%d] y[%d,%d] w=%d h=%d" % (
        i, len(c), min(xs), max(xs), min(ys), max(ys), max(xs)-min(xs), max(ys)-min(ys)))
