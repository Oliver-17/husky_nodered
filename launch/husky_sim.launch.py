#!/usr/bin/env python3
# =============================================================================
#  husky_sim.launch.py — 在倉庫場景裡開一台 Husky A200（只有模擬）
#
#  用法：
#      ros2 launch husky_nodered husky_sim.launch.py             # 開畫面
#      ros2 launch husky_nodered husky_sim.launch.py gui:=false  # 無頭（只有 server）
#
#  起了什麼：
#      ① Gazebo Harmonic，載入 warehouse_husky.sdf（倉庫 + 目標車 + Husky）
#      ② ros_gz_bridge，把 Husky 的三個話題接到 ROS 2
#
#  話題對照（gz 那邊的名字由 model.sdf 的 DiffDrive 決定）：
#      ROS /husky/cmd_vel  ──> 看門狗 ──> /husky/cmd_vel_safe ──]──> gz
#                                                      ↑ 逾時 0.5 秒自動送零
#      ⚠️ 中間一定要有看門狗：gz 的 DiffDrive 沒有指令逾時，沒人送就沿用
#         最後一筆速度，會一路開到撞牆（2026-11-07 踩過）。
#      ROS /husky/odom     <──[── gz /model/husky/odom（**真值**，不是輪子算的）
#      ROS /tf             <──[── gz /model/husky/tf
#      ROS /clock          <──[── gz /clock
#
#  ⚠️ 這裡**要**橋 /clock，和無人機那邊相反。
#     原因：odom 和 TF 都是從 Gazebo 轉過來的，時間戳是模擬時間。Nav2 若用
#     真實時鐘，兩邊差好幾個數量級，會一直噴 extrapolation into the future。
#     所以 Husky 這條鏈全部 use_sim_time: true（見 warehouse_nav2_husky.yaml）。
#     bridge 自己不能用 sim time —— /clock 就是它發的，會變成先有雞還是先有蛋。
#
#  ⚠️ bridge 要延後幾秒再開：gz 還沒把話題宣告出來時，parameter_bridge 會連不上
#     而空等，不會重試也不會報錯 —— vision.launch.py 踩過同一個雷。
# =============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HUSKY_SHARE = get_package_share_directory("husky_nodered")
DRONE_SHARE = get_package_share_directory("drone_nodered")
# ⚠️ 世界檔在 drone_nodered，不在這裡（2026-11-07 改）：
#    start_empty_sitl.sh 只會去 drone_nodered/gz/worlds 找，而且 PX4_GZ_WORLD
#    必須等於 <world name>。維持單一一份 warehouse.sdf（裡面已經含 Husky），
#    兩台載具同場才不用同步三個地方。
WORLD = os.path.join(DRONE_SHARE, "gz", "worlds", "warehouse.sdf")


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")

    # model:// 要找得到兩個地方的模型：Husky 在這個套件，目標車在 drone_nodered
    res_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.pathsep.join([os.path.join(HUSKY_SHARE, "gz", "models"),
                         os.path.join(DRONE_SHARE, "gz", "models")]))

    # -r = 直接開始跑（不要停在暫停）；-s = 只有 server，不開畫面
    # start_gz=false：兩台載具同場時 Gazebo 由 PX4 那邊開（start_empty_sitl.sh），
    # 這裡只負責 bridge 與看門狗，絕對不能再開一個 —— 兩個 gz server 會各跑
    # 各的世界，而且 ros_gz_bridge 會連到先搶到的那一個。
    start_gz = LaunchConfiguration("start_gz")
    gz_gui = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-v", "2", world], output="screen",
        condition=IfCondition(PythonExpression(
            ["'", start_gz, "'=='true' and '", gui, "'=='true'"])))
    gz_headless = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", "-v", "2", world], output="screen",
        condition=IfCondition(PythonExpression(
            ["'", start_gz, "'=='true' and '", gui, "'!='true'"])))

    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="husky_bridge",
        output="screen",
        arguments=[
            # 模擬時鐘。這條一定要在，Nav2 這條鏈全靠它。
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # ] = 只有 ROS → gz（Nav2 的速度指令）
            "/model/husky/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            # [ = 只有 gz → ROS（里程計與 TF）
            "/model/husky/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/model/husky/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        remappings=[
            ("/model/husky/cmd_vel", "/husky/cmd_vel_safe"),
            ("/model/husky/odom", "/husky/odom"),
            ("/model/husky/tf", "/tf"),
        ])

    watchdog = Node(
        package="husky_nodered", executable="cmd_vel_watchdog.py",
        name="cmd_vel_watchdog", output="screen",
        parameters=[{"in_topic": "/husky/cmd_vel",
                     "out_topic": "/husky/cmd_vel_safe",
                     "timeout_s": 0.5,
                     "use_sim_time": True}])

    return LaunchDescription([
        DeclareLaunchArgument("start_gz", default_value="true",
                              description="false = 不開 Gazebo（同場時由 PX4 那邊開）"),
        DeclareLaunchArgument("gui", default_value="true",
                              description="true=開 Gazebo 畫面，false=只有 server"),
        DeclareLaunchArgument("world", default_value=WORLD,
                              description="世界檔路徑（預設是產生器寫出來的那份）"),
        res_path,
        gz_gui,
        gz_headless,
        watchdog,
        TimerAction(period=5.0, actions=[bridge]),
    ])
