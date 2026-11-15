#!/usr/bin/env python3
# =============================================================================
#  wait_ready.py — 等地面車「真的可以下指令了」（要模擬在跑）
#
#  用法：
#      python3 ~/ros2_ws/src/husky_nodered/test/sim/wait_ready.py        # 最多等 180 秒
#      python3 ~/ros2_ws/src/husky_nodered/test/sim/wait_ready.py 300
#
#  為什麼需要它：
#      ⚠️ launch 印出東西、甚至 `ros2 service list` 看得到 /husky/api/track，
#         **都不等於可以下指令**。Nav2 的 action server 還要經過整套
#         lifecycle configure/activate，而服務會因為「Nav2 沒在跑」直接拒絕。
#      2026-11-14 測追蹤時就是這樣：服務回
#         ❌ Nav2 沒在跑（/husky/navigate_to_pose 不存在）
#      整趟測試白跑，而且看起來很像程式壞了。
#
#  判定依據是 api/status 的 ready 四項全綠：
#      platform  模擬在跑、odom 有在更新
#      odom      位置估計有效
#      nav2      Nav2 的 action server 在
#      bridge    速度看門狗在跑（沒有它 gz 不會自己停車）
#
#  回傳值：0 = 就緒，1 = 逾時（可以直接寫在 && 鏈裡）
# =============================================================================
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TIMEOUT = float(sys.argv[1]) if len(sys.argv) > 1 else 180.0

rclpy.init()
node = Node("husky_wait_ready")
state = {}
node.create_subscription(String, "/husky/api/status",
                         lambda m: state.update(json.loads(m.data)), 10)

t0 = time.time()
last_print = 0.0
while time.time() - t0 < TIMEOUT:
    rclpy.spin_once(node, timeout_sec=0.5)
    r = state.get("ready") or {}
    if r and all(r.values()):
        print(f"✓ 可以下指令了（等了 {time.time() - t0:.0f} 秒）")
        sys.exit(0)
    # 每 10 秒講一次還缺什麼，不然看起來像當掉
    if time.time() - last_print > 10:
        last_print = time.time()
        miss = [k for k, v in r.items() if not v] or ["還沒收到 api/status"]
        print(f"  等待中（{time.time() - t0:.0f} 秒）… 還缺：{', '.join(miss)}",
              flush=True)

print(f"✗ 等了 {TIMEOUT:.0f} 秒還沒就緒。最後看到的 ready = {state.get('ready')}")
sys.exit(1)
