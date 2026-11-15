#!/usr/bin/env python3
# =============================================================================
#  watch_track.py — 盯著追蹤，每秒印一行（要模擬在跑）
#
#  用法：
#      python3 ~/ros2_ws/src/husky_nodered/test/sim/watch_track.py          # 60 秒
#      python3 ~/ros2_ws/src/husky_nodered/test/sim/watch_track.py 120      # 120 秒
#
#  印什麼：
#      階段（搜尋/跟隨）、車在哪、車**認為**目標在哪、兩者相距、
#      目標位置有多舊、以及 api/status 的 last_message。
#
#  怎麼看：
#      階段一直是「搜尋」      → 視覺鏈沒接上（vision:=true 開了嗎？）
#                                或目標真的不在視野裡
#      「舊」一直在長          → 跟丟了。超過 5 秒就會切去跑搜尋環線
#      相距停在站位距離附近    → 正常，那就是「已就位」
#      相距一直變大            → 目標跑得比車快，或 Nav2 卡住了
#
#  ⚠️ 「目標」和「相距」在**搜尋**階段是沒有意義的 —— 那時顯示的是
#     「最後一次看到」的位置（看「舊」幾秒就知道有多久以前）。車開過那個
#     舊位置時「相距」會變得很小，那不是真的靠近目標。只有階段是「跟隨」
#     而且「舊」接近 0 的時候，那兩欄才是當下的。
#
#  ⚠️ 「目標」那一欄是**車認為的**位置，不是真值。要對真值請用
#     gz topic -e -t /world/warehouse/dynamic_pose/info -n 1
#     不要拿「我以為我把它擺在哪」來對 —— 目標可能被殘留的速度指令推走
#     （2026-11-14 就是這樣白查了一輪）。
# =============================================================================
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0

rclpy.init()
node = Node("watch_track")
rows, last = [], [0.0]


def cb(msg):
    d = json.loads(msg.data)
    now = time.time()
    if now - last[0] < 1.0:          # 狀態是 5 Hz，每秒印一行就夠看
        return
    last[0] = now
    t = d.get("track", {})
    hx, hy = d.get("map_x"), d.get("map_y")
    tgt = t.get("target")
    dist = (math.hypot(tgt[0] - hx, tgt[1] - hy)
            if tgt and hx is not None else None)
    rows.append(dist)
    print(f"{len(rows):4d}s  {t.get('phase', '') or '--':4s}  "
          f"車({hx:+6.2f},{hy:+6.2f})  "
          f"目標{('(%+6.2f,%+6.2f)' % tuple(tgt)) if tgt else '     --      '}  "
          f"相距{('%5.2f' % dist) if dist is not None else '  -- '}  "
          f"舊{str(t.get('target_age')):>6}s  {d.get('last_message', '')[:46]}",
          flush=True)


node.create_subscription(String, "/husky/api/status", cb, 10)
t0 = time.time()
try:
    while time.time() - t0 < DUR:
        rclpy.spin_once(node, timeout_sec=0.2)
except KeyboardInterrupt:
    pass
ds = [d for d in rows if d is not None]
if ds:
    tail = ds[-10:]
    print(f"\n相距：最小 {min(ds):.2f}　最大 {max(ds):.2f}　"
          f"最後 10 秒平均 {sum(tail) / len(tail):.2f}")
