#!/usr/bin/env python3
# =============================================================================
#  husky_all.launch.py — 倉庫場景的地面車：模擬 + TF + Nav2，一行起完
#
#  用法：
#      ros2 launch husky_nodered husky_all.launch.py              # 開畫面
#      ros2 launch husky_nodered husky_all.launch.py gui:=false   # 無頭
#
#  起的順序（和 drone_nodered 的 warehouse_all.launch.py 同一個道理）：
#      ① Gazebo + bridge（husky_sim.launch.py）
#      ② map → odom 的靜態 TF
#      ③ Nav2（延後 NAV2_DELAY 秒）
#      ④ husky_api_node（再延後 API_DELAY 秒，要等 ③ 的 /navigate_to_pose）
#
#  ⚠️ 順序不能顛倒：lifecycle_manager 在 configure 階段就會查 TF，
#     ② 還沒起來的話會看到一串 TF 逾時然後 lifecycle 失敗，
#     而且失敗之後不會自己重試 —— 要整個 Nav2 重開。
#
#  ── map → husky/odom 為什麼是「單位變換」 ───────────────────────────────
#  odom 由模型裡的 OdometryPublisher（真值里程計）發，它報的是**世界座標**，
#  而這個場景的 map 座標系就等於 Gazebo 的世界座標系（地圖 yaml 的 origin
#  已經把兩者對齊）。所以 odom 原點 ≡ map 原點，中間是單位變換。
#
#  ⚠️ 2026-11-07 踩過：一開始以為 odom 的原點在車子的 spawn（DiffDrive 的
#     輪式 odom 的確是那樣），就給了 (-14, +9) 的平移 —— 整個位置估計平移了
#     18 公尺，Nav2 規劃出來的路徑當然撞牆。換成真值里程計之後基準也跟著換了。
#     驗法：ros2 topic echo /husky/odom 的 position 應該直接等於
#           gz model -m husky -p 的世界座標。
#
#  ── 時鐘 ───────────────────────────────────────────────────────────────
#  整條鏈 use_sim_time: true（參數檔裡已經設好），靜態 TF 這個節點也要設，
#  否則它發的 TF 時間戳是真實時鐘，Nav2 查不到。
# =============================================================================
import os

import yaml
from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction

HUSKY_SHARE = get_package_share_directory("husky_nodered")
DRONE_SHARE = get_package_share_directory("drone_nodered")
# 秒。單獨跑地面車時的預設值；兩台同場時由 both_all.launch.py 往後推
#（兩套 Nav2 不能同時 configure —— 見 both_all 的檔頭）。
NAV2_DELAY = 12.0          # 要蓋過 gz 載入世界 + bridge 延後 5 秒
API_DELAY = 20.0           # 服務層要等 Nav2 的 action server 出現
VISION_DELAY = 8.0         # 相機橋接要等 gz 把世界載完、感測器開始發

# 等待用的小程式（裝在 lib/<pkg>/ 底下，不是 share/）
GATE_SCRIPT = os.path.join(get_package_prefix("husky_nodered"),
                           "lib", "husky_nodered", "wait_for_action.py")
PARAMS = os.path.join(HUSKY_SHARE, "config", "warehouse_nav2_husky.yaml")
MAP_YAML = os.path.join(DRONE_SHARE, "maps", "warehouse.yaml")
GEOM = os.path.join(DRONE_SHARE, "config", "warehouse_geometry.yaml")


def _spawn():
    """從幾何檔讀 Husky 的 spawn —— 不要在這裡寫死座標。"""
    with open(GEOM, encoding="utf-8") as f:
        g = yaml.safe_load(f)
    sx, sy = g["husky_spawn"]
    return str(sx), str(sy)


def generate_launch_description():
    sx, sy = _spawn()
    gui = LaunchConfiguration("gui")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(HUSKY_SHARE, "launch", "husky_sim.launch.py")),
        launch_arguments={"gui": gui,
                          "start_gz": LaunchConfiguration("start_gz")}.items())

    map_to_odom = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="husky_map_to_odom",
        # 全零：odom ≡ map（見檔頭）。spawn 座標只拿來印訊息，不進這個變換。
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--frame-id", "map", "--child-frame-id", "husky/odom"],
        parameters=[{"use_sim_time": True}],
        output="screen")

    # ⚠️ 推進 /husky 命名空間：兩台載具同場時，planner_server／controller_server
    #    這些節點名會撞。參數檔的頂層 key 也跟著加了 /husky/ 前綴 ——
    #    沒跟著改的話節點會安靜地套用預設值（見參數檔檔頭）。
    def make_nav2():
        """每次呼叫都產生一份新的 —— 同一個 action 物件不能掛在兩個地方。"""
        return GroupAction([PushRosNamespace("husky"), IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("drone_control"), "launch", "nav2.launch.py")),
        launch_arguments={
            # namespace 只影響 controller_server 的 cmd_vel remap
            # （→ /husky/cmd_vel，正好是 bridge 在等的）。frame 名稱不受影響。
            "namespace": "husky",
            "level": "full",
            "params_file": PARAMS,
            "map": MAP_YAML,
            "use_sim_time": "true",
            "rviz": LaunchConfiguration("rviz"),
            "goal_tool": "false",
        }.items())])

    # ── Nav2 什麼時候起：等事件，不要用猜的 ──────────────────────────────
    # 兩套 Nav2 不能同時 configure（後起的會卡在 Configuring map_server，
    # 而且不會自己重試）。原本是「地面車延後 35 秒」硬等，但實測每套 Nav2
    # 只要 3~4 秒就緒 —— 那 35 秒裡有 31 秒純浪費，而且機器更忙時反而可能不夠。
    #
    # 傳了 wait_for_action 就改成「等那個 action server 出現 + settle 幾秒」。
    # 沒傳（單獨跑地面車，沒有別的 Nav2 要讓）就維持原本的計時器。
    wait_for = LaunchConfiguration("wait_for_action")
    has_wait = PythonExpression(["'", wait_for, "' != ''"])

    gate = ExecuteProcess(
        cmd=[GATE_SCRIPT, wait_for,
             "--settle", LaunchConfiguration("nav2_settle"),
             "--timeout", LaunchConfiguration("nav2_wait_timeout")],
        condition=IfCondition(has_wait), output="screen")

    nav2_by_event = RegisterEventHandler(
        OnProcessExit(target_action=gate, on_exit=[
            LogInfo(msg="--- ③ 開 Nav2（差速車參數）—— 等到對方的 Nav2 好了 ---"),
            make_nav2(),
        ]))

    # 前視相機 + 偵測。延後 VISION_DELAY 是因為它要等 Gazebo 把世界載完
    # ——相機話題還沒出現時 bridge 會安靜地接不到東西（不會重試、不會報錯）。
    vision = TimerAction(period=VISION_DELAY, actions=[
        LogInfo(msg="--- 開前視相機與偵測 ---"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(HUSKY_SHARE, "launch", "husky_vision.launch.py")),
            condition=IfCondition(LaunchConfiguration("vision")),
            launch_arguments={
                "detector": LaunchConfiguration("detector"),
            }.items()),
    ])

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("vision", default_value="false",
                              description="開前視相機與偵測（/husky/detections）"),
        DeclareLaunchArgument("detector", default_value="",
                              description="color / yolo。留空 = 讀世界檔自動選"),
        DeclareLaunchArgument("start_gz", default_value="true",
                              description="false = 不開 Gazebo（同場時由 PX4 那邊開）"),
        DeclareLaunchArgument("nav2_delay", default_value=str(NAV2_DELAY)),
        DeclareLaunchArgument("api_delay", default_value=str(API_DELAY)),
        DeclareLaunchArgument("husky_api", default_value="true",
                              description="false = 服務層由底座提供"),
        DeclareLaunchArgument("rviz", default_value="false"),
        # 空字串 = 用 nav2_delay 計時（單獨跑地面車）。
        # 填一個 action 名稱 = 等那個 action server 出現才起 Nav2（兩台同場）。
        DeclareLaunchArgument(
            "wait_for_action", default_value="",
            description="等這個 action server 出現才開 Nav2，例如 /navigate_to_pose。"
                        "留空 = 用 nav2_delay 計時"),
        DeclareLaunchArgument(
            "nav2_settle", default_value="5.0",
            description="看到對方的 action 之後再等幾秒 —— Nav2 的 action server "
                        "是 on_configure 就建立的，看得到不代表 activate 完了"),
        DeclareLaunchArgument(
            "nav2_wait_timeout", default_value="120.0",
            description="等不到就放棄、照常開 Nav2（對方沒起來就沒有搶 configure 的問題）"),
        LogInfo(msg="--- ① 開 Gazebo（倉庫 + Husky）與 bridge ---"),
        sim,
        LogInfo(msg=f"--- ② 發 map → odom 的靜態 TF（單位變換；車子 spawn 在 {sx}, {sy}）---"),
        map_to_odom,
        vision,
        # ③ 兩條路二擇一（見上面 gate 的說明）：
        #    有傳 wait_for_action → 等事件；沒傳 → 原本的計時器
        gate,
        nav2_by_event,
        TimerAction(period=LaunchConfiguration("nav2_delay"),
                    condition=UnlessCondition(has_wait), actions=[
            LogInfo(msg="--- ③ 開 Nav2（差速車參數）---"),
            make_nav2(),
        ]),
        # ⚠️ 服務層預設**不在這裡開**：它屬於常駐的底座
        #    （drone_nodered 的 nodered_all.launch.py，和 drone_api_node 並列）。
        #    這裡再開一個會變成兩個同名節點同時發 api/status —— 儀表板會收到
        #    兩份互相打架的狀態，而且 ROS 2 只會印一行警告。
        #    單獨跑地面車、沒有底座時才傳 husky_api:=true。
        TimerAction(period=LaunchConfiguration("api_delay"), actions=[
            LogInfo(msg="--- ④ 開 husky_api（沒有底座時才需要）---"),
            Node(package="husky_nodered", executable="husky_api_node.py",
                 name="husky_api_node", output="screen",
                 condition=IfCondition(LaunchConfiguration("husky_api")),
                 parameters=[{"namespace": "husky", "use_sim_time": True,
                              "nav_action": "/husky/navigate_to_pose"}]),
        ]),
    ])
