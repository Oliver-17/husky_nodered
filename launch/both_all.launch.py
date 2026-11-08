#!/usr/bin/env python3
# =============================================================================
#  both_all.launch.py — 一個倉庫，兩台載具：無人機 + 地面車 Husky
#
#  用法：
#      ros2 launch husky_nodered both_all.launch.py
#      ros2 launch husky_nodered both_all.launch.py headless:=true
#
#  ── 誰負責開什麼 ────────────────────────────────────────────────────────
#      Gazebo        由無人機那邊開（start_empty_sitl.sh，PX4 要自己掛上去）
#      無人機        PX4 SITL 啟動時生成進世界，走 map → odom → base_link
#      Husky         已經寫在世界檔裡，走 map → husky/odom → husky/base_link
#  所以這支**不開第二個 Gazebo**，husky_sim 那邊傳 start_gz:=false。
#  （開兩個 gz server 的話會各跑各的世界，bridge 連到先搶到的那個，
#    畫面上看起來「車子不動」但其實是在另一個世界裡動。）
#
#  ── 啟動順序：兩套 Nav2 不能同時 configure ───────────────────────────────
#      t=0s     Gazebo + PX4 + XRCE agent            （無人機的 sim_all）
#      t=25s    px4_bridge                            （無人機）
#      t=35s    無人機的 Nav2
#      t=45s    drone_api
#      t=50s    Husky 的 bridge + 看門狗 + 靜態 TF
#      t=70s    Husky 的 Nav2（在 /husky 命名空間）
#      t=85s    husky_api
#  ⚠️ 為什麼要錯開 35 秒：lifecycle_manager 在 configure 階段會查 TF 和等
#     map_server。兩套同時 configure 時，後起的那套常常卡在
#     「Configuring map_server」—— 而且卡住之後不會自己重試，要整個重開。
#     這是專案裡記過的雷（Nav2 bringup 中有新節點加入會卡住）。
#
#  ── 兩台為什麼不會互相干擾 ───────────────────────────────────────────────
#      TF        無人機 base_link / 地面車 husky/base_link，共用 map
#      節點名    Husky 的 Nav2 整套推進 /husky 命名空間
#      服務      /MAV1/api/*  與  /husky/api/*
#      速度指令  /MAV1/cmd_vel（→ 速度橋 → PX4）與 /husky/cmd_vel（→ 看門狗 → gz）
# =============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

HUSKY_SHARE = get_package_share_directory("husky_nodered")
DRONE_SHARE = get_package_share_directory("drone_nodered")

HUSKY_NAV2_DELAY = 70.0     # 無人機的 Nav2 是 35s，錯開 35 秒
HUSKY_API_DELAY = 85.0


def generate_launch_description():
    headless = LaunchConfiguration("headless")

    # ⚠️ 這組預設值是照 systemd 的 drone-warehouse.service 抄的（那才是「開模擬」
    #    按鈕實際走的路徑）。warehouse_all 自己的 DEFAULT_MODEL 是 x500 ——
    #    那台**沒有下視相機**，視覺偵測整條鏈會接不上，而且不會報錯，
    #    只會發現 /detections 永遠是空的。
    drone = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(DRONE_SHARE, "launch", "warehouse_all.launch.py")),
        launch_arguments={"headless": headless,
                          "rviz": LaunchConfiguration("rviz"),
                          "model": LaunchConfiguration("model"),
                          "vision": LaunchConfiguration("vision"),
                          "altitude": LaunchConfiguration("altitude"),
                          # 服務層由常駐的底座提供（drone-nodered.service）。
                          # 這裡再開一個會變成兩個 drone_api_node 同時搶著回應。
                          "api": LaunchConfiguration("api")}.items())

    husky = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(HUSKY_SHARE, "launch", "husky_all.launch.py")),
        launch_arguments={
            "start_gz": "false",          # Gazebo 由無人機那邊開
            "gui": "false",               # 同上，這裡不管畫面
            "nav2_delay": str(HUSKY_NAV2_DELAY),
            "api_delay": str(HUSKY_API_DELAY),
            "husky_api": LaunchConfiguration("api"),   # 同上，預設交給底座
        }.items())

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false",
                              description="true = 不開 Gazebo 視窗"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="開 RViz（兩台同場時建議關掉，很吃 CPU）"),
        DeclareLaunchArgument("model", default_value="x500_cam_down",
                              description="無人機機體。x500 沒有下視相機，"
                                          "視覺偵測會接不上而且不報錯"),
        DeclareLaunchArgument("vision", default_value="true",
                              description="開視覺偵測鏈（YOLO）"),
        DeclareLaunchArgument("api", default_value="false",
                              description="兩台的服務層。預設 false ＝ 由常駐的"
                                          "底座提供；沒開底座時才設 true"),
        DeclareLaunchArgument("altitude", default_value="5.0",
                              description="速度橋鎖的巡航高度（倉庫貨架 7 m）"),
        LogInfo(msg="=== 兩台載具同場：Gazebo 由 PX4 那邊開，Husky 只接 bridge ==="),
        drone,
        husky,
    ])
