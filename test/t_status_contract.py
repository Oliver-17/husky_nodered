#!/usr/bin/env python3
# =============================================================================
#  t_status_contract.py — 兩種載具的 api/status 必須對同一份契約
#
#  跑法（不需要 Gazebo、不需要 Nav2）：
#      python3 ~/ros2_ws/src/husky_nodered/test/t_status_contract.py
#
#  為什麼要有這支：
#      「換載具不用改 Node-RED」這個主張，實際上就壓在幾個欄位上：
#          task.seq / task.state   「等待完成」積木的判定依據
#          ready.*                 儀表板的就緒燈
#          scene.home              「返航」組合積木要知道家在哪
#          map_x / map_y           儀表板地圖
#      任何一個欄位名字或狀態字串在兩邊不一致，積木就只能服務其中一種載具，
#      而且**不會報錯** —— 只會在換載具時永遠等不到完成。
#
#  檢查兩段：
#      A 靜態：兩個節點的原始碼用同一組 task 欄位與同一組狀態字串
#      B 實跑：真的把 husky_api_node 跑起來，讀一筆 status 驗結構
# =============================================================================
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HUSKY = os.path.join(HERE, "..", "scripts", "husky_api_node.py")
DRONE = os.path.join(HERE, "..", "..", "drone_nodered", "scripts", "drone_api_node.py")
fails = []

# ---------- A 靜態 -----------------------------------------------------------
TASK_INIT = r'\{"seq": 0, "name": "", "state": "idle", "message": ""\}'
for name, path in (("husky_api_node", HUSKY), ("drone_api_node", DRONE)):
    if not os.path.exists(path):
        fails.append(f"A 找不到 {path}")
        continue
    src = open(path, encoding="utf-8").read()
    if not re.search(TASK_INIT, src):
        fails.append(f"A {name} 的 _task 初始值和契約不符（seq/name/state/message）")
    for s in ('"running"', '"done" if ok else "failed"'):
        if s not in src:
            fails.append(f"A {name} 裡找不到狀態字串 {s}")
    if '"seq": self._task["seq"] + 1' not in src:
        fails.append(f"A {name} 的 _begin_task 沒有把 seq 加一")

# ---------- B 實跑 -----------------------------------------------------------
PROBE = r'''
import json, sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class P(Node):
    def __init__(self):
        super().__init__("contract_probe")
        self.got = None
        self.create_subscription(String, "/husky/api/status", self.cb, 10)
    def cb(self, m):
        if self.got is None:
            self.got = m.data

rclpy.init()
p = P()
import time
t0 = time.time()
while p.got is None and time.time() - t0 < 15:
    rclpy.spin_once(p, timeout_sec=0.2)
print(p.got if p.got else "")
rclpy.shutdown()
'''
env = dict(os.environ)
node = subprocess.Popen([sys.executable, HUSKY], stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL, env=env)
try:
    out = subprocess.run([sys.executable, "-c", PROBE], capture_output=True,
                         text=True, timeout=40, env=env).stdout.strip()
finally:
    node.terminate()
    try:
        node.wait(timeout=10)
    except subprocess.TimeoutExpired:
        node.kill()

if not out:
    fails.append("B 15 秒內收不到 /husky/api/status")
else:
    d = json.loads(out)
    need = {"connected": bool, "ready": dict, "task": dict, "nav": dict,
            "watchdog": dict, "scene": dict, "vehicle": str}
    for k, t in need.items():
        if k not in d:
            fails.append(f"B status 少了 {k}")
        elif not isinstance(d[k], t):
            fails.append(f"B status 的 {k} 型別是 {type(d[k]).__name__}，應該是 {t.__name__}")
    for k in ("seq", "name", "state", "message"):
        if k not in d.get("task", {}):
            fails.append(f"B task 少了 {k}")
    if d.get("task", {}).get("state") not in ("idle", "running", "done", "failed"):
        fails.append(f"B task.state 不是四個合法值之一：{d.get('task', {}).get('state')}")
    for k in ("platform", "odom", "nav2", "bridge"):
        if k not in d.get("ready", {}):
            fails.append(f"B ready 少了 {k}")
    if "home" not in d.get("scene", {}):
        fails.append("B scene 少了 home —— 「返航」積木會不知道家在哪"
                     "（檢查 warehouse_geometry.yaml 有沒有 husky_spawn）")
    elif len(d["scene"]["home"]) != 2:
        fails.append(f"B scene.home 不是兩個數字：{d['scene']['home']}")

if fails:
    print("✗ 狀態契約檢查沒過：")
    for f in fails:
        print("   " + f)
    sys.exit(1)
print("✓ 狀態契約檢查通過（task 四欄與狀態字串兩邊一致、ready 四項、"
      "scene.home 可用、status 結構正確）")
