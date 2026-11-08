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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction

HUSKY_SHARE = get_package_share_directory("husky_nodered")
DRONE_SHARE = get_package_share_directory("drone_nodered")
# 秒。單獨跑地面車時的預設值；兩台同場時由 both_all.launch.py 往後推
#（兩套 Nav2 不能同時 configure —— 見 both_all 的檔頭）。
NAV2_DELAY = 12.0          # 要蓋過 gz 載入世界 + bridge 延後 5 秒
API_DELAY = 20.0           # 服務層要等 Nav2 的 action server 出現

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
    nav2 = GroupAction([PushRosNamespace("husky"), IncludeLaunchDescription(
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

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("start_gz", default_value="true",
                              description="false = 不開 Gazebo（同場時由 PX4 那邊開）"),
        DeclareLaunchArgument("nav2_delay", default_value=str(NAV2_DELAY)),
        DeclareLaunchArgument("api_delay", default_value=str(API_DELAY)),
        DeclareLaunchArgument("husky_api", default_value="true",
                              description="false = 服務層由底座提供"),
        DeclareLaunchArgument("rviz", default_value="false"),
        LogInfo(msg="--- ① 開 Gazebo（倉庫 + Husky）與 bridge ---"),
        sim,
        LogInfo(msg=f"--- ② 發 map → odom 的靜態 TF（單位變換；車子 spawn 在 {sx}, {sy}）---"),
        map_to_odom,
        TimerAction(period=LaunchConfiguration("nav2_delay"), actions=[
            LogInfo(msg="--- ③ 開 Nav2（差速車參數）---"),
            nav2,
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
