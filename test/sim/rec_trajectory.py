#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""錄一趟導航的真值軌跡，寫成 CSV（報告的軌跡圖用）。

    python3 test/sim/rec_trajectory.py

做的事：呼叫 /husky/api/goto 去東南角 → 再呼叫一次回家 → 全程記位置。
位置來源是 /husky/odom（由 gz 的 OdometryPublisher 直接報模型真值），
每一段結束時再用 `gz model -m husky -p` 獨立核對一次落點，
避免「用待測系統自己的估計當證據」。
"""
import csv
import json
import os
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from vehicle_nodered_interfaces.srv import GoTo

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trajectory.csv")
LEGS = [("去東南角", 5.6, -9.0), ("返航", -14.0, 9.0)]


def gz_truth():
    """用 gz 的世界狀態獨立核對落點（和 odom 是兩個不同的查詢路徑）。"""
    try:
        out = subprocess.run(["gz", "model", "-m", "husky", "-p"],
                             capture_output=True, text=True, timeout=25).stdout
        for i, ln in enumerate(out.splitlines()):
            if "Pose" in ln and "XYZ" in ln:
                # 格式是 [x y z] [r p y]，中括號要先去掉
                nums = out.splitlines()[i + 1].replace("[", " ").replace("]", " ").split()
                return float(nums[0]), float(nums[1])
    except Exception as e:                                    # noqa: BLE001
        print("  (gz 核對失敗：%s)" % e)
    return None


class Rec(Node):
    def __init__(self):
        super().__init__("rec_trajectory")
        self.pts = []
        self.leg = ""
        self.t0 = time.time()
        self.state = {}
        self.create_subscription(Odometry, "/husky/odom", self.on_odom, 20)
        self.create_subscription(String, "/husky/api/status", self.on_st, 10)
        self.cli = self.create_client(GoTo, "/husky/api/goto")

    def on_odom(self, m):
        p = m.pose.pose.position
        self.pts.append((round(time.time() - self.t0, 2),
                         round(p.x, 3), round(p.y, 3), self.leg))

    def on_st(self, m):
        self.state = json.loads(m.data)


def main():
    rclpy.init()
    n = Rec()
    if not n.cli.wait_for_service(timeout_sec=15.0):
        print("api/goto 沒上線"); sys.exit(1)

    for name, x, y in LEGS:
        n.leg = name
        print(f"--- {name} → ({x}, {y}) ---")
        seq0 = n.state.get("task", {}).get("seq", 0)
        fut = n.cli.call_async(GoTo.Request(map_x=x, map_y=y, yaw=999.0, timeout=0.0))
        rclpy.spin_until_future_complete(n, fut, timeout_sec=20.0)
        print("  服務回覆：", fut.result().message if fut.result() else "無")

        t_start, done = time.time(), False
        while time.time() - t_start < 180 and not done:
            rclpy.spin_once(n, timeout_sec=0.2)
            t = n.state.get("task", {})
            if t.get("seq", 0) != seq0 and t.get("state") != "running":
                done = True
                print(f"  {t.get('state')}　耗時 {time.time()-t_start:.0f} s")
        truth = gz_truth()
        if truth:
            err = ((truth[0] - x) ** 2 + (truth[1] - y) ** 2) ** 0.5
            print(f"  gz 真值落點 ({truth[0]:+.2f}, {truth[1]:+.2f})　誤差 {err:.2f} m")
        time.sleep(2)

    with open(OUT, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f); w.writerow(["t", "x", "y", "leg"]); w.writerows(n.pts)
    print(f"\n{len(n.pts)} 個取樣點 → {OUT}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
