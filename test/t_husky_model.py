#!/usr/bin/env python3
# =============================================================================
#  t_husky_model.py — Husky 模型與場景的一致性檢查（不需要 ROS、不需要 Gazebo）
#
#  當初是什麼症狀：
#      Husky 的車體幾何有四個消費者，而且**不一致不會報錯**：
#        ① model.sdf 的 DiffDrive（輪距／輪半徑）→ 算錯 odom 就會有系統性誤差
#        ② model.sdf 的輪子 collision 半徑       → 和 ① 不一致等於輪子在打滑
#        ③ warehouse_geometry.yaml 的 husky_*     → 給 Nav2 的 footprint 用
#        ④ 世界檔裡 Husky 的 pose z               → 給錯會陷進地板或掉下去
#      這支測試把四個地方綁在一起，改一個忘了改其他的就會紅。
# =============================================================================
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
MODEL = os.path.join(PKG, "gz", "models", "husky", "model.sdf")
# ⚠️ 世界檔在 drone_nodered，不在這個套件（2026-11-07 改成單一一份，
#    裡面同時有無人機的目標車和 Husky）。見 gz/worlds/README.md。
WORLD = os.path.join(PKG, os.pardir, "drone_nodered", "gz", "worlds", "warehouse.sdf")
GEOM = os.path.join(PKG, os.pardir, "drone_nodered", "config",
                    "warehouse_geometry.yaml")

# 原廠 A200 的數字（來源：Fuel 模型自己帶的 husky_diff_controller 參數）
WHEEL_SEP = 0.5709          # **實際**輪距：collision 幾何與 geometry.yaml 用這個
# DiffDrive 用的是**等效**輪距（實際 × 1.46）。四輪滑移轉向轉不到理想差速的
# 角度，1.46 是 2026-11-07 用 Gazebo 真值量出來的。詳見 model.sdf 的註解。
SKID_FACTOR = 1.46
WHEEL_SEP_EFF = 0.834
WHEEL_RAD = 0.17775
MAX_SPEED = 1.0
SPAWN_Z = 0.14275

fails = []


def grab(path, pat, what):
    txt = open(path, encoding="utf-8").read()
    m = re.search(pat, txt)
    if not m:
        fails.append(f"{what}：在 {os.path.basename(path)} 裡找不到 {pat}")
        return None
    return m.group(1)


def close(a, b, tol=1e-6):
    return a is not None and abs(float(a) - b) < tol


# ① DiffDrive 的參數
sep = grab(MODEL, r"<wheel_separation>([\d.]+)</wheel_separation>", "DiffDrive 輪距")
rad = grab(MODEL, r"<wheel_radius>([\d.]+)</wheel_radius>", "DiffDrive 輪半徑")
spd = grab(MODEL, r"<max_linear_velocity>([\d.]+)</max_linear_velocity>", "速度上限")
if not close(sep, WHEEL_SEP_EFF, 1e-3):
    fails.append(f"① DiffDrive 輪距 {sep} ≠ 等效值 {WHEEL_SEP_EFF}"
                 f"（＝實際 {WHEEL_SEP} × 滑移係數 {SKID_FACTOR}）")
if abs(WHEEL_SEP * SKID_FACTOR - WHEEL_SEP_EFF) > 0.01:
    fails.append("① 等效輪距和「實際 × 滑移係數」對不上，改了一個忘了改另一個")
if not close(rad, WHEEL_RAD):
    fails.append(f"① DiffDrive 輪半徑 {rad} ≠ 原廠 {WHEEL_RAD}")
if not close(spd, MAX_SPEED):
    fails.append(f"① 速度上限 {spd} ≠ 原廠 {MAX_SPEED}")

# ② 輪子的 collision 半徑必須和 DiffDrive 用的一樣，否則等於輪子在打滑
col = grab(MODEL, r"<cylinder>\s*<radius>([\d.]+)</radius>", "輪子 collision 半徑")
if not close(col, WHEEL_RAD):
    fails.append(f"② 輪子 collision 半徑 {col} ≠ DiffDrive 的 {WHEEL_RAD}")

# ③ 四個輪子都要驅動（Husky 是四輪差速，只接兩輪會轉不動）
txt = open(MODEL, encoding="utf-8").read()
for side in ("left", "right"):
    n = len(re.findall(rf"<{side}_joint>", txt))
    if n != 2:
        fails.append(f"③ {side}_joint 有 {n} 個，應該是 2 個（前後輪都要驅動）")

# ③b 話題名稱必須寫滿絕對路徑。DiffDrive 不會自動加 model 的 namespace，
#     只寫 "cmd_vel" 會開在全域 /cmd_vel，bridge 等在 /model/husky/... 收不到
#     任何訊息（話題宣告得出來，所以更難查）。2026-11-07 踩過。
# odom / tf 現在由 OdometryPublisher（真值）發，DiffDrive 那份改名叫 *_wheel
for tag, want in (("topic", "/model/husky/cmd_vel"),
                  ("odom_topic", "/model/husky/odom_wheel"),
                  ("tf_topic", "/model/husky/tf_wheel")):
    m = re.search(rf"<{tag}>([^<]+)</{tag}>", txt)
    if not m:
        fails.append(f"③b model.sdf 裡沒有 <{tag}>")
    elif m.group(1) != want:
        fails.append(f"③b <{tag}> 是 {m.group(1)}，應該是絕對路徑 {want}")

# ③c 真值里程計一定要在，而且發的是 Nav2 在用的那組名字。
#     沒有它的話 Nav2 會接到輪式里程計 —— 滑移轉向的輪式 odom 會飄，
#     而這個專案沒有 AMCL 可以把它拉回來（2026-11-07 撞牆就是這樣來的）。
if "gz-sim-odometry-publisher-system" not in txt:
    fails.append("③c model.sdf 裡沒有 OdometryPublisher（真值里程計）")
for tag, want in (("odom_topic", "/model/husky/odom"), ("tf_topic", "/model/husky/tf")):
    if f"<{tag}>{want}</{tag}>" not in txt:
        fails.append(f"③c OdometryPublisher 的 <{tag}> 不是 {want}")

# ③d 座標系必須帶 husky/ 前綴。兩台載具同場時 TF 樹不能有兩個 base_link，
#     而撞名的症狀是「TF 時好時壞」—— 兩台都會壞，而且很難查。
for tag, want in (("frame_id", "husky/odom"), ("child_frame_id", "husky/base_link"),
                  ("odom_frame", "husky/odom"), ("robot_base_frame", "husky/base_link")):
    if f"<{tag}>{want}</{tag}>" not in txt:
        fails.append(f"③d model.sdf 的 <{tag}> 不是 {want}（兩台同場會撞名）")

# ④ 世界檔裡的 pose z
if os.path.exists(WORLD):
    z = grab(WORLD, r"<name>husky</name>\s*<pose>[-\d.]+ [-\d.]+ ([\d.]+)", "世界檔 z")
    if not close(z, SPAWN_Z, 1e-4):
        fails.append(f"④ 世界檔的 Husky pose z = {z}，應該是 {SPAWN_Z}"
                     "（輪心 0.035 + 輪半徑 0.17775）")
else:
    fails.append("④ 找不到 drone_nodered 的 warehouse.sdf —— 先跑 gen_warehouse.py")

# ⑤ 幾何 YAML 要和模型對得上
if os.path.exists(GEOM):
    g = open(GEOM, encoding="utf-8").read()
    for key, want in (("husky_wheel_separation", WHEEL_SEP),   # YAML 記的是實際值
                      ("husky_wheel_radius", WHEEL_RAD),
                      ("husky_max_speed", MAX_SPEED),
                      ("husky_spawn_z", SPAWN_Z)):
        m = re.search(rf"^{key}: *([\d.]+)", g, re.M)
        if not m:
            fails.append(f"⑤ warehouse_geometry.yaml 裡沒有 {key}")
        elif not close(m.group(1), want, 1e-4):
            fails.append(f"⑤ {key} = {m.group(1)} ≠ 模型的 {want}")
else:
    fails.append("⑤ 找不到 warehouse_geometry.yaml")

if fails:
    print("✗ Husky 模型一致性檢查沒過：")
    for f in fails:
        print("   " + f)
    sys.exit(1)
print(f"✓ Husky 模型一致性檢查通過（實際輪距 {WHEEL_SEP} / 等效 {WHEEL_SEP_EFF}、"
      f"輪半徑 {WHEEL_RAD}、四輪驅動、真值里程計、離地 {SPAWN_Z}、幾何 YAML 一致）")
