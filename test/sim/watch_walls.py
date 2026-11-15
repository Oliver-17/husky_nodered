#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""追蹤時用 Gazebo 真值記錄兩台車，事後量「有沒有撞牆」和「目標有沒有穿過去」。

    python3 test/sim/watch_walls.py 180 /tmp/out.csv

要先開好倉庫場景、並且已經呼叫 /husky/api/track。這支只看不動。

為什麼要用 Gazebo 真值、不用 /husky/odom 或 api/status：
    要驗的是「控制律有沒有把車開進牆裡」，證據不能來自待測系統自己的估計。
    目標車的位置也一樣 —— 用偵測鏈算出來的位置會把偵測誤差混進來。

量兩件事（2026-11-15 Oliver 回報的兩個問題）：
  ① 穿過去：目標車和地面車的中心距離 < PASS_THROUGH_M。
     目標車沒有碰撞箱，兩台會直接重疊，這就是「穿過去」唯一量得到的症狀。
  ② 撞牆：地面車**車身矩形**到貨架/外牆的最近距離。
     ⚠️ 量的是矩形不是中心點 —— 舊的擋牆只看中心線上一個點，這正是它
        擋不住的原因，驗證時不能再犯同一個錯。
     貨架和牆有碰撞，所以「撞到」在數字上會是 ≤ CONTACT_M（貼住推不進去）。

訂閱 gz 的 /world/<世界>/dynamic_pose/info 用 gz.transport13 的 Python binding
（和 drone_nodered/scripts/move_target.py 同一套作法）。只訂閱不做同步 request，
所以不會踩到 move_target.py 檔頭記的「callback 吃掉 request 回應」那個坑。
"""
import csv
import json
import math
import os
import sys
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from std_msgs.msg import String

WORLD = "warehouse"
HUSKY = "husky"
TARGET = "hatchback_target"
HALF_L, HALF_W = 0.495, 0.335      # 車體 0.99 x 0.67（warehouse_geometry.yaml）
PASS_THROUGH_M = 0.8               # 車體半長 0.5 + 目標車半長 0.3
CONTACT_M = 0.05                   # 車身離牆 5 cm 以內就算貼上去了
SAMPLE_S = 0.1


def load_boxes():
    path = os.path.join(get_package_share_directory("drone_nodered"),
                        "config", "warehouse_geometry.yaml")
    with open(path, encoding="utf-8") as f:
        flat = yaml.safe_load(f)["no_fly_boxes"]
    # 貨架用 x 中心命名，不用「第幾排」—— 場景檔的「第 2 排」是從 0 數的
    # （x=+2.8 那排），自己從 1 編號會和它差一號，報告時對不上。
    walls = {4: "西牆", 5: "東牆", 6: "南牆", 7: "北牆"}
    out = []
    for i in range(0, len(flat) - 3, 4):
        x0, y0, x1, y1 = flat[i:i + 4]
        k = i // 4
        name = walls.get(k) or f"貨架 x={(x0 + x1) / 2:+.1f}"
        out.append((min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1), name))
    return out


def pt_box(px, py, b):
    """點到方框的距離。點在框內回傳負值（離最近那一面多深）。"""
    dx = max(b[0] - px, 0.0, px - b[2])
    dy = max(b[1] - py, 0.0, py - b[3])
    if dx == 0.0 and dy == 0.0:
        return -min(px - b[0], b[2] - px, py - b[1], b[3] - py)
    return math.hypot(dx, dy)


def pt_rect(px, py, cx, cy, yaw):
    """點到（旋轉過的）車身矩形的距離。點在車身裡回傳 0。"""
    c, s = math.cos(yaw), math.sin(yaw)
    lx = (px - cx) * c + (py - cy) * s          # 換到車身座標
    ly = -(px - cx) * s + (py - cy) * c
    dx = max(abs(lx) - HALF_L, 0.0)
    dy = max(abs(ly) - HALF_W, 0.0)
    return math.hypot(dx, dy)


def body_clearance(cx, cy, yaw, b):
    """車身矩形到方框的最近距離（兩個凸多邊形：最近點一定落在某個頂點上）。"""
    c, s = math.cos(yaw), math.sin(yaw)
    corners = [(cx + sx * HALF_L * c - sy * HALF_W * s,
                cy + sx * HALF_L * s + sy * HALF_W * c)
               for sx in (-1, 1) for sy in (-1, 1)]
    d1 = min(pt_box(px, py, b) for px, py in corners)
    d2 = min(pt_rect(bx, by, cx, cy, yaw)
             for bx in (b[0], b[2]) for by in (b[1], b[3]))
    return min(d1, d2)


class Truth:
    def __init__(self):
        self.lock = threading.Lock()
        self.husky = None       # (x, y, yaw)
        self.target = None      # (x, y)
        self.node = GzNode()
        topic = f"/world/{WORLD}/dynamic_pose/info"
        if not self.node.subscribe(Pose_V, topic, self._on):
            sys.exit(f"訂不到 {topic} —— Gazebo 有開嗎？")

    def _on(self, msg):
        for p in msg.pose:
            if p.name not in (HUSKY, TARGET):
                continue
            q = p.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            with self.lock:
                if p.name == HUSKY:
                    self.husky = (p.position.x, p.position.y, yaw)
                else:
                    self.target = (p.position.x, p.position.y)


class Status(Node):
    def __init__(self):
        super().__init__("watch_walls")
        self.track = {}
        self.create_subscription(String, "/husky/api/status", self._on, 10)

    def _on(self, m):
        try:
            self.track = json.loads(m.data).get("track") or {}
        except Exception:                                  # noqa: BLE001
            pass


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 180.0
    out = sys.argv[2] if len(sys.argv) > 2 else "walls.csv"
    boxes = load_boxes()
    truth = Truth()
    rclpy.init()
    st = Status()
    threading.Thread(target=rclpy.spin, args=(st,), daemon=True).start()

    rows = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < secs:
        time.sleep(SAMPLE_S)
        with truth.lock:
            h, tg = truth.husky, truth.target
        if h is None or tg is None:
            continue
        clr = [(body_clearance(h[0], h[1], h[2], b), b[4]) for b in boxes]
        dmin, which = min(clr)
        rows.append({
            "t": round(time.monotonic() - t0, 2),
            "hx": round(h[0], 3), "hy": round(h[1], 3), "hyaw": round(h[2], 3),
            "tx": round(tg[0], 3), "ty": round(tg[1], 3),
            "sep": round(math.hypot(tg[0] - h[0], tg[1] - h[1]), 3),
            "clear": round(dmin, 3), "near": which,
            "phase": st.track.get("phase", ""),
            "msg": st.track.get("message", ""),
        })

    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else ["t"])
        w.writeheader()
        w.writerows(rows)

    if not rows:
        print("一筆都沒錄到")
        return

    # --- 撞牆：連續貼住算一次 ------------------------------------------------
    hits, cur = [], None
    for r in rows:
        if r["clear"] <= CONTACT_M:
            if cur is None:
                cur = dict(r, t_end=r["t"])
            else:
                cur["t_end"] = r["t"]
                if r["clear"] < cur["clear"]:
                    cur.update(clear=r["clear"])
        elif cur is not None:
            hits.append(cur)
            cur = None
    if cur:
        hits.append(cur)

    # --- 穿過去：中心距離小於門檻，連續的算一次 --------------------------------
    passes, cur = [], None
    for r in rows:
        if r["sep"] < PASS_THROUGH_M:
            if cur is None:
                cur = dict(r, t_end=r["t"])
            else:
                cur["t_end"] = r["t"]
                cur["sep"] = min(cur["sep"], r["sep"])
        elif cur is not None:
            passes.append(cur)
            cur = None
    if cur:
        passes.append(cur)

    clear_all = min(r["clear"] for r in rows)
    print(f"錄了 {len(rows)} 筆（{rows[-1]['t']:.0f} 秒）→ {out}")
    print(f"車身離貨架/牆最近：{clear_all:.2f} m")
    print(f"撞牆（≤{CONTACT_M} m）：{len(hits)} 次")
    for h in hits:
        print(f"   {h['t']:6.1f}~{h['t_end']:6.1f}s  {h['near']}  "
              f"車 ({h['hx']:+.2f},{h['hy']:+.2f}) 朝 {math.degrees(h['hyaw']):+.0f}°  "
              f"目標 ({h['tx']:+.2f},{h['ty']:+.2f})  [{h['phase']}] {h['msg'][:40]}")
    print(f"目標穿過地面車（中心 < {PASS_THROUGH_M} m）：{len(passes)} 次")
    for p in passes:
        print(f"   {p['t']:6.1f}~{p['t_end']:6.1f}s  最近 {p['sep']:.2f} m  "
              f"車 ({p['hx']:+.2f},{p['hy']:+.2f})  [{p['phase']}] {p['msg'][:40]}")


if __name__ == "__main__":
    main()
    # ⚠️ 直接 _exit，不走正常的直譯器收尾：gz.transport13 的 C++ Node 在
    #    直譯器解構時偶爾會 abort（"terminate called without an active
    #    exception"，2026-11-15 實測 3 次出現 1 次）。結果在這之前都已經寫檔、
    #    印完了，崩在收尾只會讓呼叫端以為測試失敗。
    sys.stdout.flush()
    os._exit(0)
