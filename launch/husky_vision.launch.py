#!/usr/bin/env python3
# =============================================================================
#  husky_vision.launch.py — 地面車的前視相機接進 ROS，並啟動偵測
#
#  用法：
#      ros2 launch husky_nodered husky_vision.launch.py
#      ros2 launch husky_nodered husky_vision.launch.py detector:=color
#      ros2 launch husky_nodered husky_vision.launch.py \
#          husky_yolo_weights:=~/yolo_data/weights/yolo11m.pt
#      ros2 launch husky_nodered husky_vision.launch.py debug_image:=false
#
#  前置：Gazebo 要開著，而且世界裡的 Husky 是有相機的版本
#      （gz/models/husky/model.sdf，2026-11-13 之後才有）。
#
#  做兩件事：
#      ① 把 Gazebo 的影像搬到 ROS（gz 和 ROS 是兩套不同的通訊系統）
#      ② 啟動偵測器與目標定位（drone_vision，和無人機共用同一套節點）
#
#  ---------------------------------------------------------------------------
#  和無人機那條鏈（drone_nodered/launch/vision.launch.py）的三個差別
#  ---------------------------------------------------------------------------
#  ① 話題不用組長路徑。
#     無人機的相機沒寫 <topic>，所以 gz 用的是
#         /world/<世界>/model/<機體>/link/<連桿>/sensor/<感測器>/image
#     那支 launch 得用 PythonExpression 把世界名和機體名拼進去，換世界就壞。
#     Husky 的兩顆相機在模型裡**明確寫了** <topic>husky/rgb/image</topic> 與
#     <topic>husky/depth/image</topic>，所以這裡直接用固定名字。
#
#     ⚠️ camera_info 的名字由 gz 自己決定：它把話題的**最後一段換成
#        camera_info**。實測（2026-11-13）：
#            /husky/rgb/image    → /husky/rgb/camera_info
#            /husky/depth/image  → /husky/depth/camera_info（另外還有 .../points）
#        這就是為什麼兩顆相機要給**不同的前綴**。都放在 /husky/ 底下、
#        只有最後一段不同的話，兩個 camera_info 會撞在一起。
#
#  ② 不需要 target_world_node，改用 TF。
#     無人機那顆是「射線 + 目標離地高度的水平面求交點」。Husky 的相機是
#     **水平朝前**的（俯角只有 10°），射線幾乎平行於地面 —— 一條近乎平行的
#     射線去和 0.1175 m 高的平面求交點，車身顛一下算出來的距離就從 5 m
#     跳到 50 m，而且不會報錯。
#     地面車必須靠**深度**：target_locator_node 發的 target_point 就是
#     深度算出來的相機座標 3D 點，那個才是對的來源。
#     map 座標由 drone_vision 的 point_to_frame_node 用 TF 換算（2026-11-14），
#     發在 /husky/api/target。那顆節點沒有任何幾何計算，純粹是 tf2 的轉換。
#
#  ③ 整包推進 /husky 命名空間。
#     drone_vision 的節點話題全用相對名稱（detections / target_point /
#     target_ray），所以推命名空間就自動變成 /husky/detections 等等，
#     **節點一行都不用改**。沒推的話會和無人機的 /detections 撞在一起。
#
#     ⚠️ 橋接那個節點**故意放在命名空間外面** —— 它的話題名已經是
#        /husky/... 開頭的絕對路徑了，再推一層會變成 /husky/husky/...
#
#  ④ 🔴 權重不一樣：地面車用 COCO（yolo11s.pt），無人機用 VisDrone。
#     VisDrone 是**空拍俯視**的資料集。地面車的相機是水平朝前的，看到的是
#     車尾／車側 —— 完全不在那個分布裡。
#
#     2026-11-13 在倉庫場景量的（目標車在正前方 4 m，同一張影像）：
#         yolov8s-visdrone.pt   conf 降到 0.01 都**完全偵測不到**
#         yolo11s.pt（COCO）    conf 0.876
#         yolo11m.pt（COCO）    conf 0.877   ← 沒有比 s 好，用 s 就夠
#
#     ⚠️ 不要以為是信心門檻或尺度的問題去調參數 —— VisDrone 那顆在這個視角
#        是「什麼都沒看到」，不是「看到但不夠有把握」。
#
#     ⚠️ 探測用的簡易世界（黑地板、亮車）VisDrone 反而測得到 0.83，
#        所以**不要用對比強的測試場景驗這件事**，會得到相反的結論。
#        倉庫是灰車配灰地板，那才是真的條件。
#
#     偵測器節點不用改：它是用**類別名稱**（vehicle_classes 參數，預設
#     car,van,truck,bus）去查該權重自己的 names 表，兩套權重都適用。
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

DRONE_PKG = "drone_nodered"
WORLD = "warehouse"

# 模型裡寫死的 gz 話題（gz/models/husky/model.sdf 的 <topic>）。
# 改模型的話這四個要跟著改 —— 但不會靜默失敗：橋接收不到東西時
# /husky/rgb/image 根本不會出現，ros2 topic list 一看就知道。
# 相機在 base_link 的安裝位置與俯角。
# ⚠️ 必須和 gz/models/husky/model.sdf 的 camera_link <pose> 一字不差 ——
#    不一致**不會報錯**，只會讓算出來的目標位置固定偏一個方向。
CAM_X, CAM_Y, CAM_Z = "0.35", "0", "0.45"
CAM_PITCH = "0.17453"      # 10°，往下為正

RGB = "/husky/rgb/image"
RGB_INFO = "/husky/rgb/camera_info"
DEPTH = "/husky/depth/image"
DEPTH_INFO = "/husky/depth/camera_info"


def _default_detector():
    """讀世界檔決定預設偵測器 —— 和 warehouse_all.launch.py 同一套邏輯。

    世界裡是哪一種目標車，就只有對應的偵測器找得到它：
        magenta_target    洋紅方盒 → color
        hatchback_target  真車外觀 → yolo
    配錯的話偵測輸出是**全空**，而且不會報錯。

    為什麼讀檔不寫死：世界檔由 gen_warehouse.py --target 產生，檔名不變，
    「現在是哪一種」只有檔案內容知道。
    """
    path = os.path.join(get_package_share_directory(DRONE_PKG),
                        "gz", "worlds", f"{WORLD}.sdf")
    try:
        with open(path, encoding="utf-8") as f:
            text = f.read()
    except OSError:
        return "color"
    return "yolo" if "hatchback_target" in text else "color"


def generate_launch_description():
    detector = LaunchConfiguration("detector")
    # 空字串 = 自動（讀世界檔）。上游的 husky_all 用空字串代表「你自己決定」，
    # 它不該重複一份世界檔判讀的邏輯。
    # Python 的 '' or 'yolo' 會得到 'yolo'，所以這一個表達式就同時處理
    # 「有傳」和「沒傳」兩種情況。
    auto = _default_detector()
    is_yolo = PythonExpression(["('", detector, "' or '", auto, "') == 'yolo'"])
    is_color = PythonExpression(["('", detector, "' or '", auto, "') == 'color'"])

    # --- 1) 影像橋接（命名空間外）------------------------------------------
    # 用 parameter_bridge 不用 image_bridge：一次帶四個話題（含兩個 camera_info），
    # image_bridge 只處理影像。「[」代表只從 gz 流向 ROS。
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="husky_camera_bridge", output="screen",
        arguments=[
            f"{RGB}@sensor_msgs/msg/Image[gz.msgs.Image",
            f"{RGB_INFO}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            f"{DEPTH}@sensor_msgs/msg/Image[gz.msgs.Image",
            f"{DEPTH_INFO}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ])

    # --- 2) 相機的 TF（兩層）-------------------------------------------------
    # 為什麼要兩層而不是直接發一條 base_link → camera_optical：
    #     中間那個 camera_link 是**實體**的相機框（x 前、y 左、z 上，
    #     就是模型裡那個 pose）。RViz 上看得到相機裝在哪、朝哪。
    #     只發一條的話，TF 樹裡只剩一個轉了 90° 的框，看圖的人認不出那是相機。
    #
    # ⚠️ 第二條是 ROS 的標準光學旋轉（yaw −90°、roll −90°）。
    #    它把「x 前 y 左 z 上」轉成「x 右 y 下 z 前」。
    #    target_locator_node 輸出的點就是後者（target_locator_node.py:147-149），
    #    而訊息的 frame_id 來自模型的 gz_frame_id（husky/camera_optical）。
    #    少了這條，或是把 frame 取名叫 camera_link，目標位置會**安靜地**
    #    整個轉 90°，沒有任何錯誤訊息。
    tf_mount = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="husky_cam_mount", output="log",
        parameters=[{"use_sim_time": True}],
        arguments=["--x", CAM_X, "--y", CAM_Y, "--z", CAM_Z,
                   "--pitch", CAM_PITCH,
                   "--frame-id", "husky/base_link",
                   "--child-frame-id", "husky/camera_link"])

    tf_optical = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="husky_cam_optical", output="log",
        parameters=[{"use_sim_time": True}],
        arguments=["--yaw", "-1.5708", "--pitch", "0", "--roll", "-1.5708",
                   "--frame-id", "husky/camera_link",
                   "--child-frame-id", "husky/camera_optical"])

    # --- 3) 偵測與定位（推進 /husky）----------------------------------------
    vision = GroupAction([
        PushRosNamespace("husky"),

        # 顏色偵測
        Node(package="drone_vision", executable="color_detector_node.py",
             name="color_detector", output="screen", emulate_tty=True,
             condition=IfCondition(is_color),
             parameters=[{"image_topic": RGB,
                          "publish_debug_image":
                              LaunchConfiguration("debug_image")}]),

        # YOLO 偵測
        # ⚠️ prefix 指定 venv 的直譯器：ultralytics 只裝在 venv 裡
        #    （系統 numpy 1.21 太舊，而升級 numpy 會殺死 cv_bridge）。
        #    用 prefix 的好處是使用者不用先 source，systemd 也能直接用。
        Node(package="drone_vision", executable="yolo_detector_node.py",
             name="yolo_detector", output="screen", emulate_tty=True,
             prefix=LaunchConfiguration("yolo_python"),
             condition=IfCondition(is_yolo),
             parameters=[{"image_topic": RGB,
                          "weights": LaunchConfiguration("husky_yolo_weights"),
                          "confidence": ParameterValue(
                              LaunchConfiguration("yolo_conf"), value_type=float),
                          "publish_debug_image":
                              LaunchConfiguration("debug_image")}]),

        # 目標定位：框 + 深度 → 相機座標的 3D 點（/husky/target_point）
        # 這顆訂閱相對名稱的 detections，所以推命名空間就自動接上面兩個之一。
        Node(package="drone_vision", executable="target_locator_node.py",
             name="target_locator", output="screen", emulate_tty=True,
             parameters=[{"rgb_info_topic": RGB_INFO,
                          "depth_topic": DEPTH,
                          "depth_info_topic": DEPTH_INFO}]),

        # 相機座標的 3D 點 → map 座標。
        # remap 成 api/target，和無人機的 /MAV1/api/target 對齊 ——
        # 兩台的「目標在哪」用同一個名字，儀表板和下游才不用分兩種寫法。
        # ⚠️ 無人機那個是 PX4 本地 NED，這個是 map。語意一樣、座標系不同，
        #    因為地面車整條鏈（goto、Nav2、場景幾何）本來就都用 map。
        Node(package="drone_vision", executable="point_to_frame_node.py",
             name="target_to_map", output="screen", emulate_tty=True,
             parameters=[{"target_frame": "map", "use_sim_time": True}],
             remappings=[("target_map", "api/target")]),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "detector", default_value="",
            choices=["", "color", "yolo"],
            description="用哪個偵測器。留空 = 讀世界檔自動選"),
        DeclareLaunchArgument(
            "debug_image", default_value="true",
            description="要不要發畫了框的影像（/husky/debug_image）"),
        # 🔴 名字一定要和無人機那支不一樣（2026-11-15 踩到，藏了兩天）。
        #    ROS 2 launch 的 LaunchConfiguration 是**跨 include 共用**的，而
        #    DeclareLaunchArgument 只在「這個名字還沒有值」時才套用 default_value。
        #    both_all 先開無人機：drone_nodered/launch/vision.launch.py 宣告了
        #    yolo_weights = yolov8s-visdrone.pt；輪到這支時那個名字已經有值了，
        #    所以下面這行的 COCO 預設**被安靜地忽略**，地面車實際載入的是
        #    VisDrone —— 正好是檔頭那條「絕對不能用」的權重。
        #    症狀完全不像設定錯：偵測得到，只是時有時無（實測線上 29%，
        #    同樣的影像離線用 COCO 跑是 60%），看起來只像「視覺不穩」。
        #    判定法：ros2 param get /husky/yolo_detector weights，或看啟動日誌
        #    的「載入權重 …」那一行。
        #    ⚠️ 同一個陷阱還在 debug_image / yolo_conf / yolo_python 上，只是
        #       兩邊預設值剛好一樣所以目前無害。改其中一邊的預設值前要想到這件事。
        DeclareLaunchArgument(
            "husky_yolo_weights",
            default_value=os.path.expanduser("~/yolo_data/weights/yolo11s.pt"),
            description="YOLO 權重。地面車用 COCO，不是無人機的 VisDrone"),
        DeclareLaunchArgument(
            "yolo_python",
            default_value=os.path.expanduser("~/yolo-venv/bin/python3"),
            description="跑 YOLO 的直譯器（ultralytics 在獨立 venv）"),
        DeclareLaunchArgument("yolo_conf", default_value="0.25"),
        bridge,
        tf_mount,
        tf_optical,
        vision,
    ])
