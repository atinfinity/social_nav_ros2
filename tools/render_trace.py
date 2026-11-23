#!/usr/bin/env python3
"""social_nav_sim の --trace 出力 2 本(左=social, 右=baseline)を横並びで描画し、
rgb24 の rawvideo(1280x480, 20 fps)を標準出力へ出す。標準ライブラリのみ。

usage: render_trace.py LEFT_TRACE RIGHT_TRACE [MAX_SECONDS] | ffmpeg -f rawvideo -pix_fmt rgb24 -s 1280x480 -r 20 -i - ...
"""
import bisect
import math
import sys

PW, PH = 640, 480          # 1 パネルの大きさ
FPS = 20
VIEW_W = 12.0              # カメラが映す幅 [m]。ロボットに追従する
SCALE = PW / VIEW_W        # px/m
CY = 250                   # y=0 の画素行
HOLD_S = 1.5               # 終了後に最終フレームを保持する秒数
PERSONAL = 1.2             # M4 のパーソナルスペース半径 [m](図示のみ)

BG = (24, 26, 32)
FLOOR = (52, 56, 66)
WALL = (190, 190, 200)
ROBOT = (70, 140, 255)
PED = (255, 170, 60)
HIT = (240, 60, 60)
GOAL = (80, 220, 120)
RING = (120, 100, 70)
TRAIL = (110, 160, 230)
PHONE = (230, 80, 220)     # 歩きスマホ(kind 2)
FAMILY = [(255, 235, 60), (80, 230, 230), (190, 130, 255), (255, 120, 150)]   # 家族(kind 1)は群れごとに 1 色
ARROW = (255, 255, 255)
ARROW_DT = 0.3             # 速度ベクトルは [t-0.3, t] の位置差から出す
ARROW_LEN = 0.8            # 矢印の長さ = 速度[m/s] x 0.8 [m](1.3 m/s で約 1 m)
BEHAVIOR = {"PROCEED": (60, 200, 90), "FOLLOW": (60, 200, 220), "YIELD": (240, 200, 50), "WAIT": (230, 70, 70)}


def load(path):
    t = {"walls": [], "frames": [], "ped_info": {}}
    with open(path) as f:
        for line in f:
            p = line.split()
            if not p:
                continue
            k = p[0]
            if k == "H":
                t["scenario"], t["seed"], t["controller"] = p[1], p[2], p[3]
            elif k == "W":
                t["walls"].append(tuple(float(v) for v in p[1:5]))
            elif k == "S":
                t["start"] = (float(p[1]), float(p[2]))
            elif k == "G":
                t["goal"] = (float(p[1]), float(p[2]))
            elif k == "R":
                t["rr"], t["pr"] = float(p[1]), float(p[2])
            elif k == "P":
                t["ped_info"][int(p[1])] = (int(p[2]), int(p[3]))
            elif k == "F":
                n = int(p[6])
                peds = [(float(p[7 + 2 * i]), float(p[8 + 2 * i])) for i in range(n)]
                t["frames"].append((float(p[1]), float(p[2]), float(p[3]), float(p[4]), p[5], peds))
    return t


class Panel:
    def __init__(self, tr):
        self.tr = tr
        self.buf = bytearray(PW * PH * 3)
        self.trail = []
        self.idx = 0
        # 壁の |y| が 8 m を超えるとき(屋内シナリオの枝通路。既定の 6 本は 7 m 以下)だけ、縮尺を下げて収める。既定の動画は不変
        ys = [w[1] for w in tr["walls"]] + [w[3] for w in tr["walls"]]
        self.scale, self.cy, self.view_w = SCALE, CY, VIEW_W
        ext = max(abs(max(ys)), abs(min(ys)))
        if ext > 8.0:
            self.scale = (PH - 40) / (2.0 * ext)
            self.cy = 14 + (PH - 14) / 2
            self.view_w = PW / self.scale

    def fill_rect(self, x0, y0, x1, y1, c):
        x0, x1 = max(0, int(x0)), min(PW, int(x1))
        y0, y1 = max(0, int(y0)), min(PH, int(y1))
        if x1 <= x0:
            return
        row = bytes(c) * (x1 - x0)
        for y in range(y0, y1):
            o = (y * PW + x0) * 3
            self.buf[o:o + 3 * (x1 - x0)] = row

    def disc(self, cx, cy, r, c, ring=0):
        r2 = r * r
        ri2 = (r - ring) ** 2 if ring else -1
        for y in range(max(0, int(cy - r)), min(PH, int(cy + r) + 1)):
            dy = y - cy
            h2 = r2 - dy * dy
            if h2 < 0:
                continue
            hw = math.sqrt(h2)
            for x in range(max(0, int(cx - hw)), min(PW, int(cx + hw) + 1)):
                if ring and (x - cx) ** 2 + dy * dy < ri2:
                    continue
                o = (y * PW + x) * 3
                self.buf[o:o + 3] = bytes(c)

    def line(self, x0, y0, x1, y1, c, w=1):
        n = int(max(abs(x1 - x0), abs(y1 - y0))) + 1
        for i in range(n + 1):
            u = i / n
            self.disc(x0 + (x1 - x0) * u, y0 + (y1 - y0) * u, w, c)

    def draw(self, k):
        tr = self.tr
        fr = tr["frames"]
        t = k / FPS
        while self.idx + 1 < len(fr) and fr[self.idx + 1][0] <= t:
            self.idx += 1
        _, rx, ry, th, beh, peds = fr[self.idx]
        done = t >= fr[-1][0]
        cam = min(max(rx + 2.5, tr["xmin"] + self.view_w / 2), tr["xmax"] - self.view_w / 2)

        def px(x):
            return PW / 2 + (x - cam) * self.scale

        def py(y):
            return self.cy - y * self.scale

        self.buf[:] = bytes(BG) * (PW * PH)
        ys = [w[1] for w in tr["walls"]] + [w[3] for w in tr["walls"]]
        self.fill_rect(0, py(max(ys)), PW, py(min(ys)), FLOOR)
        for (x0, y0, x1, y1) in tr["walls"]:
            self.line(px(x0), py(y0), px(x1), py(y1), WALL, 3)
        gx, gy = tr["goal"]
        self.disc(px(gx), py(gy), 0.25 * self.scale, GOAL, ring=3)
        if not done and (not self.trail or t - self.trail[-1][0] >= 0.25):
            self.trail.append((t, rx, ry))
        for (_, x, y) in self.trail:
            self.disc(px(x), py(y), 2, TRAIL)
        rr, pr = tr["rr"], tr["pr"]
        for (x, y) in peds:
            self.disc(px(x), py(y), PERSONAL * self.scale, RING, ring=1)
        info = tr["ped_info"]
        kinds = [info.get(i + 1, (0, -1)) for i in range(len(peds))]
        # 群れを線で結ぶ(同じ group_id の家族。隣同士を順に)
        groups = {}
        for i, (kd, g) in enumerate(kinds):
            if kd != 2 and g >= 0:
                groups.setdefault(g, []).append(i)
        gcol = {g: FAMILY[n % len(FAMILY)] for n, g in enumerate(sorted(groups))}
        for g, ids in groups.items():
            for a, b in zip(ids, ids[1:]):
                self.line(px(peds[a][0]), py(peds[a][1]), px(peds[b][0]), py(peds[b][1]), gcol[g], 2)
        # 1 つ前の時刻の位置から速度ベクトルを出す
        ts = [f[0] for f in fr]
        jp = max(0, bisect.bisect_right(ts, fr[self.idx][0] - ARROW_DT) - 1)
        prev = fr[jp][5]
        dtv = fr[self.idx][0] - fr[jp][0]
        for i, (x, y) in enumerate(peds):
            kd, g = kinds[i]
            hit = math.hypot(x - rx, y - ry) < rr + pr
            col = PED
            if kd != 2 and g >= 0:
                col = gcol.get(g, FAMILY[0])
            elif kd == 2:
                col = PHONE
            R = pr * self.scale
            if hit:
                self.disc(px(x), py(y), R, HIT)
            else:
                self.disc(px(x), py(y), R, col)
            if kd == 2:   # 歩きスマホ: 白い輪郭と小さな端末(縦長の黒い四角)
                self.disc(px(x), py(y), R + 2, (255, 255, 255), ring=2)
                self.fill_rect(px(x) - 3, py(y) - 6, px(x) + 3, py(y) + 6, (20, 20, 20))
            if dtv > 1e-6 and i < len(prev):
                vx, vy = (x - prev[i][0]) / dtv, (y - prev[i][1]) / dtv
                sp = math.hypot(vx, vy)
                if sp > 0.05:
                    ex, ey = x + vx * ARROW_LEN, y + vy * ARROW_LEN
                    self.line(px(x), py(y), px(ex), py(ey), ARROW, 1)
                    ux, uy = vx / sp, vy / sp
                    for sgn in (1, -1):
                        hx = ex - 0.18 * (ux * 0.866 - sgn * uy * 0.5)
                        hy = ey - 0.18 * (uy * 0.866 + sgn * ux * 0.5)
                        self.line(px(ex), py(ey), px(hx), py(hy), ARROW, 1)
        self.disc(px(rx), py(ry), rr * self.scale, ROBOT)
        self.line(px(rx), py(ry), px(rx + rr * math.cos(th)), py(ry + rr * math.sin(th)), (255, 255, 255), 2)
        # ふるまい帯(上端)。色の対応は動画の凡例で示す
        self.fill_rect(0, 0, PW, 28, BEHAVIOR.get(beh, BG))
        return t

    def bytes_(self):
        return bytes(self.buf)


def main():
    if len(sys.argv) not in (3, 4):
        sys.exit("usage: render_trace.py LEFT_TRACE RIGHT_TRACE [MAX_SECONDS]")
    max_s = float(sys.argv[3]) if len(sys.argv) == 4 else None
    panels = []
    for p in sys.argv[1:3]:
        tr = load(p)
        if not tr["frames"]:
            sys.exit("empty trace: " + p)
        xs = [w[0] for w in tr["walls"]] + [w[2] for w in tr["walls"]]
        tr["xmin"], tr["xmax"] = min(xs), max(xs)
        panels.append(Panel(tr))
    last = max(p.tr["frames"][-1][0] for p in panels)
    total = last + HOLD_S
    if max_s is not None:
        total = min(total, max_s)   # 打ち切り(長い走行でもファイルが肥大しない)
    n = int(total * FPS)
    out = sys.stdout.buffer
    for k in range(n):
        for p in panels:
            p.draw(k)
        for y in range(PH):
            o = y * PW * 3
            out.write(panels[0].buf[o:o + PW * 3])
            out.write(panels[1].buf[o:o + PW * 3])
    out.flush()


if __name__ == "__main__":
    main()
