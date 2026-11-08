#!/usr/bin/env python3
# =============================================================================
#  t_nav_e2e.py — 端到端驗收：開模擬 → 服務層 → Nav2 → 車子真的動
#
#  ⚠️ 這支**需要 Gazebo**，所以不在 test/run_all.sh 裡（那支的前提是
#     「不開模擬也能跑」）。要跑：
#         python3 ~/ros2_ws/src/husky_nodered/test/sim/t_nav_e2e.py
#     大約 3 分鐘，結束時會自己把開起來的東西關掉。
#
#  位置一律用 Gazebo 真值（gz model -m husky -p）當量尺。
#  🔴 不要用 /husky/odom 來驗 —— 那是待測系統自己的輸出。2026-11-07 用它
#     量出「誤差 0.24 m」的漂亮數字，實際上車子卡在 31 公尺外的牆角。
#
#  檢查四組：
#    ① Nav2 推進 /husky 命名空間之後，action 還在
#    ② 參數真的有吃到 —— 參數檔的 key 沒跟著加 /husky/ 前綴的話，節點會
#       **安靜地套用 Nav2 預設值**，沒有任何錯誤訊息
#    ③ TF 是 map → husky/odom → husky/base_link，而且沒有裸的 base_link
#       （有的話兩台載具同場時會撞名，兩台都會壞）
#    ④ 真的導航得到
# =============================================================================
import json
import math
import os
import re
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
procs, fails = [], []
GOAL = (5.6, -9.0)


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


def truth():
    o = sh(["gz", "model", "-m", "husky", "-p"], t=12)
    m = re.search(r"\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]", o)
    return (float(m.group(1)), float(m.group(2))) if m else None


def check(name, ok, detail=""):
    print(f"  [{'通' if ok else '不通'}] {name}" + (f"　{detail}" if detail else ""))
    if not ok:
        fails.append(name)


try:
    print("[1] 啟動 husky_all（Nav2 在 /husky 命名空間裡）")
    spawn(["ros2", "launch", "husky_nodered", "husky_all.launch.py", "gui:=false"],
          "t_nav_e2e.log")
    t0 = time.time()
    while "/husky/navigate_to_pose" not in sh(["ros2", "action", "list"], t=20):
        if time.time() - t0 > 120:
            break
        time.sleep(2)
    acts = [a for a in sh(["ros2", "action", "list"]).split() if "navigate" in a]
    check("/husky/navigate_to_pose 存在", "/husky/navigate_to_pose" in acts, str(acts))

    print("\n[2] 參數有沒有真的吃到")
    for node, key, want in (("/husky/controller_server", "FollowPath.max_vel_x", 1.0),
                            ("/husky/controller_server", "FollowPath.max_vel_y", 0.0),
                            ("/husky/controller_server",
                             "general_goal_checker.xy_goal_tolerance", 0.4),
                            ("/husky/planner_server", "use_sim_time", True)):
        out = sh(["ros2", "param", "get", node, key], t=25).strip()
        m = re.search(r"value is: *([\w.]+)", out)
        got = m.group(1) if m else out[:40]
        check(f"{node.split('/')[-1]}.{key} = {want}",
              str(want).lower() == str(got).lower(), f"實際 {got}")

    print("\n[3] TF 樹")
    # ⚠️ tf2_echo 不會自己結束。用 subprocess.run(timeout=...) 包它的話，
    #    逾時例外會把已經印出來的東西整個丟掉 —— 這條檢查就永遠抓不到任何
    #    東西，看起來像 TF 壞了。2026-11-07 被這樣騙過一次。
    #    正確做法：Popen 開著、等幾秒、自己 terminate，再讀它寫進檔案的內容。
    log = os.path.join(HERE, "tf_echo.log")
    with open(log, "wb") as lf:
        tfp = subprocess.Popen(
            ["ros2", "run", "tf2_ros", "tf2_echo", "map", "husky/base_link",
             "--ros-args", "-p", "use_sim_time:=true"],
            stdout=lf, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
        time.sleep(8)
        os.killpg(os.getpgid(tfp.pid), signal.SIGTERM)
        try:
            tfp.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(tfp.pid), signal.SIGKILL)
    e = open(log, encoding="utf-8", errors="replace").read()
    m = re.search(r"Translation: \[([^\]]+)\]", e)
    check("map → husky/base_link 查得到", bool(m), m.group(1) if m else "查不到")
    ids = set(re.findall(r"(?:frame_id|child_frame_id): (\S+)",
                         sh(["ros2", "topic", "echo", "--once", "/tf"], t=20)))
    check("沒有裸的 base_link（同場時會和無人機撞名）",
          "base_link" not in ids, str(sorted(ids)))

    print(f"\n[4] 導航到 map {GOAL}")
    out = sh(["ros2", "service", "call", "/husky/api/goto",
              "vehicle_nodered_interfaces/srv/GoTo",
              "{map_x: %f, map_y: %f, yaw: 999.0, timeout: 0.0}" % GOAL], t=40)
    check("api/goto 被接受", "success=True" in out.replace(" ", ""))
    t1 = time.time()
    state = None
    while time.time() - t1 < 150:
        s = sh(["ros2", "topic", "echo", "--once", "--field", "data",
                "/husky/api/status"], t=20).replace("---", "").strip()
        try:
            state = json.loads(s)["task"]["state"]
        except Exception:
            state = None
        if state in ("done", "failed"):
            break
        time.sleep(2)
    p = truth()
    err = math.hypot(p[0] - GOAL[0], p[1] - GOAL[1])
    check("task.state = done", state == "done", str(state))
    check("真的到了（真值誤差 < 0.5 m）", err < 0.5,
          f"落點 ({p[0]:+.2f}, {p[1]:+.2f})　誤差 {err:.2f} m　{time.time()-t1:.0f} s")
finally:
    for p, f in procs:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        except Exception:
            pass
    time.sleep(4)
    for p, f in procs:
        if p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
        f.close()

print()
if fails:
    print(f"✗ 端到端驗收沒過（{len(fails)} 項）：")
    for f_ in fails:
        print("   " + f_)
    sys.exit(1)
print("✓ 端到端驗收通過（命名空間、參數生效、TF 前綴、真的導航得到）")
