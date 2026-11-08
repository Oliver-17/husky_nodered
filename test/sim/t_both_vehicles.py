#!/usr/bin/env python3
# =============================================================================
#  t_both_vehicles.py — 兩台載具同場，而且可以分別下任務
#
#  ⚠️ 需要 Gazebo + PX4 SITL，約 5 分鐘。不在 run_all.sh 裡。
#      python3 ~/ros2_ws/src/husky_nodered/test/sim/t_both_vehicles.py
#
#  檢查：
#    ① 兩台的狀態話題都在（/MAV1/api/status 與 /husky/api/status）
#    ② TF 兩組座標系並存且不撞名（base_link 與 husky/base_link）
#    ③ 兩套 Nav2 的 action 都在（/navigate_to_pose 與 /husky/navigate_to_pose）
#    ④ **同時**下任務：無人機起飛、地面車導航，兩個都要成功
#    ⑤ 全程的 CPU 負載
#
#  地面車的位置一律用 Gazebo 真值；無人機用它自己的 api/status
#  （PX4 SITL 的位置估計在模擬中本來就等於真值）。
# =============================================================================
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
procs, fails, cpu = [], [], []
HUSKY_GOAL = (5.6, -9.0)
DRONE_ALT = 3.0


def spawn(cmd, log):
    f = open(os.path.join(HERE, log), "wb")
    p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
    procs.append((p, f))
    return p


def sh(cmd, t=30):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=t).stdout
    except subprocess.TimeoutExpired:
        return ""


def status(ns):
    s = sh(["ros2", "topic", "echo", "--once", "--field", "data",
            f"/{ns}/api/status"], t=20).replace("---", "").strip()
    try:
        return json.loads(s)
    except Exception:
        return None


def truth():
    o = sh(["gz", "model", "-m", "husky", "-p"], t=12)
    m = re.search(r"\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]", o)
    return (float(m.group(1)), float(m.group(2))) if m else None


def check(name, ok, detail=""):
    print(f"  [{'通' if ok else '不通'}] {name}" + (f"　{detail}" if detail else ""))
    if not ok:
        fails.append(name)


def cpu_rec():
    while True:
        try:
            cpu.append(os.getloadavg()[0])
        except Exception:
            pass
        time.sleep(5)


try:
    print("[1] 啟動 both_all（約 90 秒：Gazebo+PX4 → 無人機 Nav2 → Husky Nav2）")
    threading.Thread(target=cpu_rec, daemon=True).start()
    spawn(["ros2", "launch", "husky_nodered", "both_all.launch.py",
           "headless:=true", "rviz:=false"], "t_both.log")
    t0 = time.time()
    d = h = None
    while time.time() - t0 < 240:
        d, h = status("MAV1"), status("husky")
        if d and h:
            break
        time.sleep(5)
    check("無人機的 /MAV1/api/status 在", d is not None)
    check("地面車的 /husky/api/status 在", h is not None,
          f"（{time.time()-t0:.0f} s）")

    print("\n[2] TF")
    ids = set(re.findall(r"(?:frame_id|child_frame_id): (\S+)",
                         sh(["ros2", "topic", "echo", "--once", "/tf"], t=20)))
    for _ in range(6):          # /tf 一次只會拿到一批，多抓幾次
        ids |= set(re.findall(r"(?:frame_id|child_frame_id): (\S+)",
                              sh(["ros2", "topic", "echo", "--once", "/tf"], t=20)))
    check("無人機的 base_link 在", "base_link" in ids)
    check("地面車的 husky/base_link 在", "husky/base_link" in ids, str(sorted(ids)))

    print("\n[3] 兩套 Nav2")
    acts = sh(["ros2", "action", "list"], t=25)
    check("/navigate_to_pose（無人機）", "/navigate_to_pose" in acts.split())
    check("/husky/navigate_to_pose（地面車）", "/husky/navigate_to_pose" in acts.split())

    print(f"\n[4] 同時下任務：無人機起飛 {DRONE_ALT} m、地面車導航到 {HUSKY_GOAL}")
    t1 = time.time()
    hp = subprocess.Popen(
        ["ros2", "service", "call", "/husky/api/goto",
         "vehicle_nodered_interfaces/srv/GoTo",
         "{map_x: %f, map_y: %f, yaw: 999.0, timeout: 0.0}" % HUSKY_GOAL],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    dout = sh(["ros2", "service", "call", "/MAV1/api/takeoff",
               "drone_nodered_interfaces/srv/Takeoff",
               "{altitude: %f}" % DRONE_ALT], t=40)
    hout = hp.communicate(timeout=60)[0]
    check("兩個服務都回 success=true",
          "success=True" in dout.replace(" ", "") and "success=True" in hout.replace(" ", ""))

    hs = ds = None
    while time.time() - t1 < 200:
        a, b = status("MAV1"), status("husky")
        ds = a["task"]["state"] if a else None
        hs = b["task"]["state"] if b else None
        if ds in ("done", "failed") and hs in ("done", "failed"):
            break
        time.sleep(3)
    a, b = status("MAV1"), status("husky")
    alt = a.get("altitude_m") if a else None
    p = truth()
    err = math.hypot(p[0] - HUSKY_GOAL[0], p[1] - HUSKY_GOAL[1]) if p else 99
    check("無人機 task.state = done", ds == "done", str(ds))
    check("無人機真的爬到高度", alt is not None and abs(alt - DRONE_ALT) < 0.5,
          f"{alt} m")
    check("地面車 task.state = done", hs == "done", str(hs))
    check("地面車真的到了（真值）", err < 0.5,
          f"({p[0]:+.2f}, {p[1]:+.2f})　誤差 {err:.2f} m" if p else "讀不到真值")
    print(f"      兩件任務同時進行，總共花 {time.time()-t1:.0f} s")

    print(f"\n[5] CPU 一分鐘負載：最高 {max(cpu):.1f} / 平均 {sum(cpu)/len(cpu):.1f}"
          f"（{os.cpu_count()} 核）")
finally:
    for p, f in procs:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        except Exception:
            pass
    time.sleep(6)
    for p, f in procs:
        if p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
        f.close()

print()
if fails:
    print(f"✗ 兩台同場驗收沒過（{len(fails)} 項）：")
    for x in fails:
        print("   " + x)
    sys.exit(1)
print("✓ 兩台同場驗收通過（狀態、TF、兩套 Nav2、可分別下任務）")
