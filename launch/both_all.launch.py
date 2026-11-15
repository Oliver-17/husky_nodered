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
#      t=35s    無人機的 Nav2        → 約 t=39 就緒
#      t=45s    drone_api
#      t=50s    Husky 的 bridge + 看門狗 + 靜態 TF
#      ★       Husky 的 Nav2：**等無人機的 /navigate_to_pose 出現 + 5 秒**
#               （2026-11-15 從固定 t=70s 改成等事件，實測約 t=44 就起得來）
#      t=85s    husky_api（只有 api:=true 才開，預設由底座提供）
#  ⚠️ 為什麼一定要錯開：lifecycle_manager 在 configure 階段會查 TF 和等
#     map_server。兩套同時 configure 時，後起的那套常常卡在
#     「Configuring map_server」—— 而且卡住之後不會自己重試，要整個重開。
#     這是專案裡記過的雷（Nav2 bringup 中有新節點加入會卡住）。
#  ⚠️ 為什麼看到 action 之後還要再等 5 秒：Nav2 的 action server 是在
#     **on_configure** 就建立的，不是 on_activate。看得到只代表對方 configure
#     完了，可能還在 activate（實測約 2 秒）。
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

# ⚠️ 2026-11-15：地面車的 Nav2 改成**等事件**，不再用這個固定延遲。
#    理由（實測）：每套 Nav2 從啟動到就緒只要 3~4 秒 ——
#        t=35 無人機 Nav2 啟動 → t=39 就緒 → …空等 31 秒… → t=70 地面車啟動
#    那 35 秒裡有 31 秒是純浪費的，而且還是用猜的：機器更忙的時候反而可能不夠。
#    改成「等無人機的 /navigate_to_pose 出現 + settle 5 秒」就兩個問題一起解決。
#    HUSKY_NAV2_DELAY 留著只是當 wait_for_action 用不了時的退路。
DRONE_NAV_ACTION = "/navigate_to_pose"      # 無人機的 Nav2 沒有推 namespace
HUSKY_NAV2_DELAY = 70.0     # 退路：萬一改用計時模式才會用到
# ⚠️ 只有 api:=true 時才用得到（預設 false，服務層由常駐底座提供）。
#    它還是計時的 —— 要跟著改成等事件的話，要等的是 /husky/navigate_to_pose。
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
            # 等無人機的 Nav2 真的起來才開地面車的（取代固定 35 秒錯開）
            "wait_for_action": DRONE_NAV_ACTION,
            "api_delay": str(HUSKY_API_DELAY),
            "husky_api": LaunchConfiguration("api"),   # 同上，預設交給底座
            # 地面車的前視相機（2026-11-13）。和無人機的視覺鏈互不影響：
            # 話題在 /husky/ 底下，gz 那側的 <topic> 也分開了。
            "vision": LaunchConfiguration("husky_vision"),
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
                              description="開**無人機**的視覺偵測鏈（YOLO）"),
        DeclareLaunchArgument("husky_vision", default_value="false",
                              description="開**地面車**的前視相機與偵測。"
                                          "預設關：兩條 YOLO 很吃 CPU，"
                                          "要用地面車的視覺時才開"),
        DeclareLaunchArgument("api", default_value="false",
                              description="兩台的服務層。預設 false ＝ 由常駐的"
                                          "底座提供；沒開底座時才設 true"),
        DeclareLaunchArgument("altitude", default_value="5.0",
                              description="速度橋鎖的巡航高度（倉庫貨架 7 m）"),
        LogInfo(msg="=== 兩台載具同場：Gazebo 由 PX4 那邊開，Husky 只接 bridge ==="),
        drone,
        husky,
    ])
