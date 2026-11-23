#!/usr/bin/env python3
"""record_costmap_demo.sh が録った rosbag(/global_costmap/costmap と /social_nav/tracked_people)を
rgb24 の rawvideo(640x480, 20 fps)として標準出力へ出す。rosbag2_py と numpy が要る(dev コンテナにある)。

画面: 世界座標 x=-4..4, y=-3..3 [m] を 80 px/m で表示。色=コスト(0 でほぼ黒、100 で赤)。
白の輪=人(半径 0.3 m)と速度矢印(1 秒分)、青い円=ロボット(静止)。
usage: render_costmap_bag.py BAG_DIR | ffmpeg -f rawvideo -pix_fmt rgb24 -s 640x480 -r 20 -i - ...
"""
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

W, H, FPS = 640, 480, 20
PPM = 80.0
X0, Y0 = -4.0, 3.0   # 左上の世界座標
LEAD_S = 2.0         # 録画開始直後を切り捨てる最低秒数(人が来てから costmap に山が出るまで 0〜2 s かかる)
MARGIN_S = 0.5       # 最初に山(コスト>0)が出た costmap の時刻から、さらに切り捨てる秒数


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    costmaps, people = [], []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        msg = deserialize_message(data, types[topic])
        if topic == '/global_costmap/costmap':
            costmaps.append((t_ns * 1e-9, msg))
        elif topic == '/social_nav/tracked_people':
            people.append((t_ns * 1e-9, msg))
    frames = {m.header.frame_id for _, m in people}
    if frames != {'map'} or any(not m.people for _, m in people):
        sys.exit('tracked_people に想定外のメッセージがある(frame=%s)。他の publisher が混ざっている' % sorted(frames))
    return costmaps, people


def make_lut():
    lut = np.zeros((256, 3), np.uint8)
    xs = [0, 1, 34, 67, 100]
    cols = np.array([(20, 22, 30), (30, 40, 110), (40, 170, 150), (250, 220, 60), (240, 50, 40)], float)
    v = np.arange(101)
    for c in range(3):
        lut[:101, c] = np.interp(v, xs, cols[:, c]).astype(np.uint8)
    lut[101:] = (240, 50, 40)
    return lut


def grid_index(info):
    xs = X0 + (np.arange(W) + 0.5) / PPM
    ys = Y0 - (np.arange(H) + 0.5) / PPM
    ix = np.floor((xs - info.origin.position.x) / info.resolution).astype(int)
    iy = np.floor((ys - info.origin.position.y) / info.resolution).astype(int)
    ok = ((iy[:, None] >= 0) & (iy[:, None] < info.height) & (ix[None, :] >= 0) & (ix[None, :] < info.width))
    return np.clip(iy, 0, info.height - 1)[:, None], np.clip(ix, 0, info.width - 1)[None, :], ok


def to_px(x, y):
    return (x - X0) * PPM, (Y0 - y) * PPM


def ring(img, cx, cy, r, color, width=2.0, fill=None):
    x0, x1 = int(max(cx - r - 3, 0)), int(min(cx + r + 4, W))
    y0, y1 = int(max(cy - r - 3, 0)), int(min(cy + r + 4, H))
    if x0 >= x1 or y0 >= y1:
        return
    yy, xx = np.mgrid[y0:y1, x0:x1]
    d = np.hypot(xx + 0.5 - cx, yy + 0.5 - cy)
    sub = img[y0:y1, x0:x1]
    if fill is not None:
        sub[d <= r] = fill
    sub[np.abs(d - r) <= width / 2] = color


def line(img, ax, ay, bx, by, color):
    n = int(max(abs(bx - ax), abs(by - ay))) + 1
    for s in np.linspace(0.0, 1.0, n + 1):
        x, y = int(ax + (bx - ax) * s), int(ay + (by - ay) * s)
        if 1 <= x < W - 1 and 1 <= y < H - 1:
            img[y - 1:y + 2, x - 1:x + 2] = color


def draw(lut, cm, ppl, idx):
    iy, ix, ok = idx
    info = cm.info
    raw = np.asarray(cm.data, np.int16).reshape(info.height, info.width)[iy, ix]
    img = lut[np.clip(raw, 0, 255)]
    img[raw < 0] = (60, 60, 60)
    img[~ok] = (60, 60, 60)
    for m in range(int(X0), int(X0 + W / PPM) + 1):   # 1 m 格子
        px = int((m - X0) * PPM)
        if 0 <= px < W:
            img[:, px] = (img[:, px] * 0.75 + 20).astype(np.uint8)
    for m in range(int(Y0 - H / PPM), int(Y0) + 1):
        py = int((Y0 - m) * PPM)
        if 0 <= py < H:
            img[py, :] = (img[py, :] * 0.75 + 20).astype(np.uint8)
    cx, cy = to_px(0.0, 0.0)
    ring(img, cx, cy, 0.3 * PPM, (70, 140, 255), 2.0, fill=(40, 80, 160))
    if ppl is not None:
        for p in ppl.people:
            px, py = to_px(p.position.x, p.position.y)
            ring(img, px, py, p.radius * PPM, (255, 255, 255), 2.0)
            ex, ey = to_px(p.position.x + p.velocity.x, p.position.y + p.velocity.y)
            line(img, px, py, ex, ey, (255, 255, 255))
    return img


def main():
    costmaps, people = read_bag(sys.argv[1])
    if not costmaps or not people:
        sys.exit('bag に costmap か tracked_people が無い')
    lut = make_lut()
    idx = grid_index(costmaps[0][1].info)
    # 山が出る前のフレームを入れると、GIF のループで山が急に現れてチラついて見える
    t_peak = next((t for t, m in costmaps if max(m.data) > 0), None)
    if t_peak is None:
        sys.exit('costmap に山(コスト>0)が無い')
    t_start = max(people[0][0] + LEAD_S, t_peak + MARGIN_S)
    t_end = people[-1][0]
    n = int((t_end - t_start) * FPS)
    ci = pi = 0
    out = sys.stdout.buffer
    for k in range(n):
        t = t_start + k / FPS
        while ci + 1 < len(costmaps) and costmaps[ci + 1][0] <= t:
            ci += 1
        while pi + 1 < len(people) and people[pi + 1][0] <= t:
            pi += 1
        out.write(np.ascontiguousarray(draw(lut, costmaps[ci][1], people[pi][1], idx)).tobytes())
    out.flush()
    print('frames: %d (%.1f s), costmaps: %d, people msgs: %d' % (n, n / FPS, len(costmaps), len(people)),
          file=sys.stderr)


if __name__ == '__main__':
    main()
