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


def read_sdf(path):
    """讀 SDF 並**去掉 XML 註解**。

    為什麼要去註解（2026-11-13 踩到）：這些檔案的註解裡會引用別的檔案的
    標籤當例子，例如相機那段寫了「無人機的深度相機是 <topic>depth_camera
    </topic>」。用正則直接掃原文的話，會把註解裡的例子當成這個檔案真的有
    那個設定 —— 測試紅了，但模型其實是對的。
    """
    txt = open(path, encoding="utf-8").read()
    return re.sub(r"<!--.*?-->", "", txt, flags=re.S)


def grab(path, pat, what):
    txt = read_sdf(path)
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
txt = read_sdf(MODEL)
for side in ("left", "right"):
    n = len(re.findall(rf"<{side}_joint>", txt))
    if n != 2:
        fails.append(f"③ {side}_joint 有 {n} 個，應該是 2 個（前後輪都要驅動）")

# ③b 話題名稱必須寫滿絕對路徑。DiffDrive 不會自動加 model 的 namespace，
#     只寫 "cmd_vel" 會開在全域 /cmd_vel，bridge 等在 /model/husky/... 收不到
#     任何訊息（話題宣告得出來，所以更難查）。2026-11-07 踩過。
# odom / tf 現在由 OdometryPublisher（真值）發，DiffDrive 那份改名叫 *_wheel
#
# ⚠️ 2026-11-13：這段原本是「抓檔案裡**第一個** <topic>」。加了相機之後
#    model.sdf 有三個 <topic>（兩顆相機 + DiffDrive），第一個不再是
#    DiffDrive 的 —— 測試會紅，但模型其實是對的。所以改成**先把 DiffDrive
#    那個 plugin 區塊切出來**，只在裡面找。
diff_drive = re.search(
    r'<plugin[^>]*gz-sim-diff-drive-system.*?</plugin>', txt, re.S)
if not diff_drive:
    fails.append("③b model.sdf 裡找不到 DiffDrive plugin")
else:
    dd = diff_drive.group(0)
    for tag, want in (("topic", "/model/husky/cmd_vel"),
                      ("odom_topic", "/model/husky/odom_wheel"),
                      ("tf_topic", "/model/husky/tf_wheel")):
        m = re.search(rf"<{tag}>([^<]+)</{tag}>", dd)
        if not m:
            fails.append(f"③b DiffDrive 裡沒有 <{tag}>")
        elif m.group(1) != want:
            fails.append(f"③b DiffDrive 的 <{tag}> 是 {m.group(1)}，"
                         f"應該是絕對路徑 {want}")

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

# ⑥ 前視相機（2026-11-13 新增）
#
#    這一項要綁住的是**三個會靜默失敗的耦合**：
#      a. gz 話題名要和無人機分開。無人機的深度相機寫死 <topic>depth_camera</topic>
#         （x500_cam_down/model.sdf），那是 gz 的全域名稱、不帶模型路徑。
#         撞名的話兩台的資料會混在一起，而且沒有任何錯誤訊息。
#      b. 兩顆相機的 <topic> 前綴要不同。gz 的 camera_info 是把話題的**最後一段
#         換成 camera_info**（2026-11-13 實測），所以 /husky/a/image 和
#         /husky/b/image 才會得到兩個不同的 camera_info。只差最後一段會撞。
#      c. launch 裡寫死的話題名要和模型對得上（husky_vision.launch.py 的
#         RGB / DEPTH 四個常數）。
RGB_TOPIC = "husky/rgb/image"
DEPTH_TOPIC = "husky/depth/image"
DRONE_MODEL = os.path.join(PKG, os.pardir, "drone_nodered", "gz", "models",
                           "x500_cam_down", "model.sdf")
VIS_LAUNCH = os.path.join(PKG, "launch", "husky_vision.launch.py")

mtxt = read_sdf(MODEL)
for want, what in ((RGB_TOPIC, "RGB"), (DEPTH_TOPIC, "深度")):
    if f"<topic>{want}</topic>" not in mtxt:
        fails.append(f"⑥ 模型裡找不到 {what} 相機的 <topic>{want}</topic>")

# a. 不能和無人機的深度相機撞名
if os.path.exists(DRONE_MODEL):
    dtxt = read_sdf(DRONE_MODEL)
    drone_topics = set(re.findall(r"<topic>([^<]+)</topic>", dtxt))
    husky_topics = set(re.findall(r"<topic>([^<]+)</topic>", mtxt))
    clash = drone_topics & husky_topics
    if clash:
        fails.append(f"⑥a 和無人機的 gz 話題撞名：{sorted(clash)}。"
                     "兩台會發同一個話題，資料混在一起且不會報錯")

# b. 兩顆相機的 camera_info 不能撞（＝<topic> 不能只差最後一段）
prefixes = [t.rsplit("/", 1)[0] for t in
            re.findall(r"<topic>([^<]+)</topic>", mtxt) if "/" in t]
if len(prefixes) != len(set(prefixes)):
    fails.append(f"⑥b 兩顆相機的 <topic> 前綴一樣（{prefixes}）—— "
                 "gz 的 camera_info 只換最後一段，兩個 camera_info 會撞在一起")

# c. launch 的常數要和模型一致
if os.path.exists(VIS_LAUNCH):
    ltxt = open(VIS_LAUNCH, encoding="utf-8").read()
    for want in (RGB_TOPIC, DEPTH_TOPIC):
        if f'"/{want}"' not in ltxt:
            fails.append(f"⑥c husky_vision.launch.py 裡沒有 /{want}，"
                         "和模型不一致（橋接會接不到東西）")
else:
    fails.append("⑥c 找不到 husky_vision.launch.py")

# ⑥e 相機的座標系名稱與靜態 TF（2026-11-14）
#
#     三件事必須一致，而且任何一個對不上**都不會報錯**：
#       ① 模型的 gz_frame_id —— 影像訊息的 frame_id，會一路傳到 target_point
#       ② launch 的第一條靜態 TF 的位移與俯角 —— 必須等於模型 camera_link 的 pose
#       ③ 第二條靜態 TF 必須是標準光學旋轉（yaw −90°、roll −90°）
#     ②對不上 → 目標位置固定偏一個方向；③少了或名字取成 camera_link →
#     目標位置整個轉 90°。兩種都沒有錯誤訊息。
OPTICAL_FRAME = "husky/camera_optical"
CAM_POSE = (0.35, 0.0, 0.45, 0.17453)      # x, y, z, pitch（和模型的 <pose> 同一組）

if f"<gz_frame_id>{OPTICAL_FRAME}</gz_frame_id>" not in mtxt:
    fails.append(f"⑥e 模型的 gz_frame_id 不是 {OPTICAL_FRAME}。"
                 "target_locator 輸出的是光學座標（x 右 y 下 z 前），"
                 "叫 camera_link 的話下游用 TF 換算會整個轉 90°")

m = re.search(r'<link name="camera_link">\s*(?:<!--.*?-->\s*)?'
              r'<pose relative_to="base_link">([^<]+)</pose>', mtxt, re.S)
if not m:
    fails.append("⑥e 模型裡找不到 camera_link 的 <pose relative_to=\"base_link\">")
else:
    v = [float(x) for x in m.group(1).split()]
    got = (v[0], v[1], v[2], v[4])          # x y z 和 pitch（第 5 個數字）
    if any(abs(a - b) > 1e-4 for a, b in zip(got, CAM_POSE)):
        fails.append(f"⑥e 模型的 camera_link pose {got} 和測試記的 {CAM_POSE} 不一致")

if os.path.exists(VIS_LAUNCH):
    ltxt = open(VIS_LAUNCH, encoding="utf-8").read()
    lm = re.search(r'CAM_X, CAM_Y, CAM_Z = "([^"]+)", "([^"]+)", "([^"]+)"', ltxt)
    lp = re.search(r'CAM_PITCH = "([^"]+)"', ltxt)
    if not lm or not lp:
        fails.append("⑥e husky_vision.launch.py 裡找不到 CAM_X/CAM_Y/CAM_Z/CAM_PITCH")
    else:
        got_l = (float(lm.group(1)), float(lm.group(2)), float(lm.group(3)),
                 float(lp.group(1)))
        if any(abs(a - b) > 1e-4 for a, b in zip(got_l, CAM_POSE)):
            fails.append(f"⑥e launch 的相機安裝 {got_l} 和模型的 {CAM_POSE} 不一致 —— "
                         "目標位置會固定偏一個方向，而且不會報錯")
    # 第二條：標準光學旋轉，而且父子框名字要對
    if not re.search(r'"--yaw", "-1\.5708".*?"--roll", "-1\.5708"', ltxt, re.S):
        fails.append("⑥e launch 裡沒有標準光學旋轉（yaw −1.5708、roll −1.5708）")
    for frame in ("husky/camera_link", OPTICAL_FRAME):
        if f'"{frame}"' not in ltxt:
            fails.append(f"⑥e launch 的靜態 TF 沒有用到 {frame}")

# d. 感測器要能被算：Gazebo 的預設 server.config 沒有 Sensors system。
#    本套件自己開 Gazebo 時靠 gz/config/server.config 補上（由 husky_sim
#    設進 GZ_SIM_SERVER_CONFIG_PATH）。少了它相機永遠零筆資料、不會報錯。
SRV_CFG = os.path.join(PKG, "gz", "config", "server.config")
if not os.path.exists(SRV_CFG):
    fails.append("⑥d 找不到 gz/config/server.config —— "
                 "沒有它的話自己開的 Gazebo 不會算相機")
elif "gz-sim-sensors-system" not in open(SRV_CFG, encoding="utf-8").read():
    fails.append("⑥d gz/config/server.config 裡沒有 gz-sim-sensors-system")
elif "GZ_SIM_SERVER_CONFIG_PATH" not in open(
        os.path.join(PKG, "launch", "husky_sim.launch.py"), encoding="utf-8").read():
    fails.append("⑥d husky_sim.launch.py 沒有設 GZ_SIM_SERVER_CONFIG_PATH，"
                 "那份 server.config 不會被用到")

if fails:
    print("✗ Husky 模型一致性檢查沒過：")
    for f in fails:
        print("   " + f)
    sys.exit(1)
print(f"✓ Husky 模型一致性檢查通過（實際輪距 {WHEEL_SEP} / 等效 {WHEEL_SEP_EFF}、"
      f"輪半徑 {WHEEL_RAD}、四輪驅動、真值里程計、離地 {SPAWN_Z}、幾何 YAML 一致、"
      f"前視相機話題不撞名且 launch 對得上）")
