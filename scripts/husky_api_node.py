#!/usr/bin/env python3
# =============================================================================
#  husky_api_node.py — 地面車的服務層（Node-RED 積木對接的那一層）
#
#  用法：
#      ros2 run husky_nodered husky_api_node.py --ros-args -p namespace:=husky
#
#  手動測：
#      ros2 service call /husky/api/goto vehicle_nodered_interfaces/srv/GoTo \
#          "{map_x: 5.6, map_y: -9.0, yaw: 999.0, timeout: 0.0}"
#      ros2 service call /husky/api/stop std_srvs/srv/Trigger
#      ros2 service call /husky/api/state std_srvs/srv/Trigger
#      ros2 topic echo /husky/api/status --full-length
#
#  ── 為什麼只有三個服務 ───────────────────────────────────────────────────
#  無人機那邊有八個，這裡只要三個，差別不是功能縮水而是載具不同：
#      起飛／降落／調整高度／盤旋   地面車沒有這些概念
#      巡航（DO_REPOSITION）        那是飛控內建的，地面車沒有對應品
#      追蹤目標                     需求裡沒有（感知鏈整條不接）
#  「返航」也不在這裡 —— 它是編排層把「導航到座標」和 scene.home 組起來的
#  組合積木，飛控端不提供。這是規格書 P-2 的分工，地面車照樣成立。
#
#  ── 和無人機共用的協定（不能改）─────────────────────────────────────────
#  積木判定「動作完成」看的是 api/status 裡的：
#      task.seq     每次開始新任務就 +1
#      task.state   running / done / failed
#  積木送出指令時記下當下的 seq，然後等「seq 變了而且 state 不是 running」。
#  這兩個欄位的語意必須和 drone_api_node 完全一致，否則同一個「等待完成」
#  積木沒辦法同時服務兩種載具 —— 那正是這個專案要證明的事。
#  t_status_contract.py 會檢查這件事。
# =============================================================================
import json
import math
import os
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger
from vehicle_nodered_interfaces.srv import GoTo

ODOM_STALE_S = 2.0          # 超過這麼久沒收到 odom 就當位置估計失效
STATUS_HZ = 5.0             # 和無人機一致


class HuskyApi(Node):
    def __init__(self):
        super().__init__("husky_api_node")
        self.declare_parameter("namespace", "husky")
        self.declare_parameter("goto_timeout_s", 180.0)
        self.declare_parameter("geometry_file", "")
        # Nav2 被推進 /husky 命名空間（兩台同場時節點會撞名），
        # 所以 action 不是 /navigate_to_pose。單獨跑時可以改回來。
        self.declare_parameter("nav_action", "/husky/navigate_to_pose")
        ns = self.get_parameter("namespace").value
        self.goto_timeout = float(self.get_parameter("goto_timeout_s").value)

        self._lock = threading.Lock()
        self._odom = None            # (x, y, yaw, speed)
        self._odom_t = None          # 收到的時間（monotonic）
        self._task = {"seq": 0, "name": "", "state": "idle", "message": ""}
        self._task_deadline = 0.0
        self._nav = {"state": "idle", "goal": None, "seq": 0,
                     "distance_remaining": None, "message": ""}
        self._nav_seq = 0
        self._nav_handle = None
        self._wd = {"tripped": False, "reason": ""}
        self._last_reply = ""        # 最近一次服務回覆（成功或被拒絕的理由）

        self._scene = self._load_scene()

        self.sub_odom = self.create_subscription(
            Odometry, f"/{ns}/odom", self._on_odom, 10)
        # 停車用。⚠️ 發到 Nav2 用的同一個話題，後面接看門狗再進 gz；
        #    直接發到 cmd_vel_safe 會和看門狗搶，兩個來源寫同一個話題。
        self.pub_cmd = self.create_publisher(Twist, f"/{ns}/cmd_vel", 10)
        self.pub_status = self.create_publisher(
            __import__("std_msgs.msg", fromlist=["String"]).String,
            f"/{ns}/api/status", 10)
        self._String = __import__("std_msgs.msg", fromlist=["String"]).String

        nav_action = self.get_parameter("nav_action").value
        self.nav_client = ActionClient(self, NavigateToPose, nav_action)

        self.create_service(GoTo, f"/{ns}/api/goto", self.srv_goto)
        self.create_service(Trigger, f"/{ns}/api/stop", self.srv_stop)
        self.create_service(Trigger, f"/{ns}/api/state", self.srv_state)

        self.create_timer(1.0 / STATUS_HZ, self._tick)
        self.get_logger().info(
            f"husky_api_node 起來了：/{ns}/api/{{goto,stop,state}}，"
            f"狀態發在 /{ns}/api/status（{STATUS_HZ:.0f} Hz）")

    # ---------- 場景幾何：唯一來源是 gen_warehouse.py 產生的 YAML -------------
    def _load_scene(self):
        path = self.get_parameter("geometry_file").value
        if not path:
            path = os.path.join(get_package_share_directory("drone_nodered"),
                                "config", "warehouse_geometry.yaml")
        try:
            with open(path, encoding="utf-8") as f:
                g = yaml.safe_load(f)
        except Exception as e:                                   # noqa: BLE001
            self.get_logger().warn(f"讀不到場景幾何（{path}）：{e} —— "
                                   "「返航」積木會不知道家在哪")
            return {"boxes": [], "home": None}
        boxes, flat = [], g.get("no_fly_boxes", [])
        for i in range(0, len(flat) - 3, 4):                     # 一維陣列，四個一組
            boxes.append([round(float(v), 3) for v in flat[i:i + 4]])
        home = g.get("husky_spawn")
        return {"boxes": boxes, "home": [float(home[0]), float(home[1])] if home else None}

    # ---------- 狀態 ---------------------------------------------------------
    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        v = msg.twist.twist.linear
        with self._lock:
            self._odom = (p.x, p.y, yaw, math.hypot(v.x, v.y))
            self._odom_t = time.monotonic()

    def _odom_fresh(self):
        return (self._odom_t is not None
                and time.monotonic() - self._odom_t < ODOM_STALE_S)

    def _ready(self):
        """四項就緒條件。和無人機一樣分開顯示 —— 少哪一項症狀完全不同。"""
        return {
            # 平台：模擬在跑、而且 odom 有在更新
            "platform": self._odom_fresh(),
            # 位置估計有效
            "odom": self._odom is not None and self._odom_fresh(),
            # Nav2 的 action server 在
            "nav2": self.nav_client.server_is_ready(),
            # 速度看門狗在跑（它是 cmd_vel 的訂閱者）。沒有它 gz 不會自己停車。
            "bridge": self.count_subscribers(
                f"/{self.get_parameter('namespace').value}/cmd_vel") > 0,
        }

    def _tick(self):
        self._check_task()
        self._check_watchdog()
        d = {"connected": self._odom_fresh()}
        with self._lock:
            o = self._odom
        if o:
            d["map_x"], d["map_y"] = round(o[0], 2), round(o[1], 2)
            d["map_heading_deg"] = round(math.degrees(o[2]), 1)
            d["speed_mps"] = round(o[3], 2)
        d["ready"] = self._ready()
        d["task"] = dict(self._task)
        d["nav"] = dict(self._nav)
        d["watchdog"] = dict(self._wd)
        d["scene"] = {"boxes": self._scene["boxes"]}
        if self._scene["home"] is not None:
            # 「返航」積木讀這個。拿不到就走失敗出口，不要亂猜一個 (0,0)。
            d["scene"]["home"] = list(self._scene["home"])
        # 「最近訊息」：儀表板會顯示這一行。優先顯示任務的訊息，
        # 沒有就退回導航的。無人機那邊也有同名欄位，儀表板共用同一段程式。
        # 優先顯示任務訊息；任務還沒開始（例如服務當場被拒絕）就顯示那個回覆。
        d["last_message"] = (self._task.get("message")
                             or self._nav.get("message")
                             or self._last_reply or "")
        d["vehicle"] = "husky"          # 給介面辨識用：同一組積木接不同載具
        msg = self._String()
        msg.data = json.dumps(d, ensure_ascii=False)
        self.pub_status.publish(msg)

    # ---------- 任務序號：和 drone_api_node 同一套 ----------------------------
    def _begin_task(self, name, timeout_s, done_now=False, message=""):
        """開一個新任務。

        done_now=True 用在「沒有過程」的動作（停止、狀態查詢）：送出就算完成。
        ⚠️ 這兩個動作也必須有自己的 task.seq —— 積木的「等待完成」是靠
           「seq 變了而且 state 不是 running」判定的，不開任務的話積木會一直
           等到逾時才走失敗出口。無人機的盤旋／降落是同一個作法。
        """
        self._task = {"seq": self._task["seq"] + 1, "name": name,
                      "state": "done" if done_now else "running",
                      "message": message}
        self._task_deadline = time.monotonic() + timeout_s

    def _end_task(self, ok, message=""):
        if self._task["state"] == "running":
            self._task["state"] = "done" if ok else "failed"
            self._task["message"] = message

    def _check_task(self):
        if (self._task["state"] == "running"
                and time.monotonic() > self._task_deadline):
            self._cancel_nav()
            self._stop_wheels()
            self._end_task(False, "逾時")

    def _check_watchdog(self):
        """唯一會自己動的機制：位置估計斷了就停車。"""
        if self._wd["tripped"]:
            return
        if self._task["state"] == "running" and not self._odom_fresh():
            self._wd = {"tripped": True, "reason": "超過 2 秒沒收到 odom"}
            self.get_logger().warn("看門狗：沒有位置估計了 —— 取消導航並停車")
            self._cancel_nav()
            self._stop_wheels()
            self._end_task(False, "看門狗：位置估計中斷")

    # ---------- 動作 ---------------------------------------------------------
    def _stop_wheels(self):
        self.pub_cmd.publish(Twist())

    def _cancel_nav(self):
        h = self._nav_handle
        self._nav_handle = None
        if h is not None:
            try:
                h.cancel_goal_async()
            except Exception:                                    # noqa: BLE001
                pass

    def _reply(self, resp, ok, message):
        resp.success = ok
        resp.message = message
        # ⚠️ 被拒絕的理由一定要進狀態。只回給呼叫端的話，儀表板上什麼都不會變，
        #    使用者看到的是「按了沒反應」—— 分不出是沒收到、還是被拒絕。
        #    （2026-11-07：模擬沒開時按「導航到目標」就是這個症狀。）
        self._last_reply = ("✅ " if ok else "❌ ") + message
        self.get_logger().info(("✓ " if ok else "✗ ") + message)
        return resp

    def srv_goto(self, request, response):
        name = "導航到座標"
        r = self._ready()
        if not r["odom"]:
            return self._reply(response, False, "沒有位置估計（收不到 odom）")
        if not r["nav2"]:
            return self._reply(response, False,
                               f"Nav2 沒在跑（{self.get_parameter('nav_action').value} 不存在）")
        if not r["bridge"]:
            return self._reply(response, False,
                               "速度看門狗沒在跑 —— 沒有它 gz 不會自己停車，拒絕出車")
        gx, gy = float(request.map_x), float(request.map_y)
        yaw = float(request.yaw)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        if abs(yaw) > math.pi:
            # 「不指定終點朝向」。四元數還是要給合法值（w=1），
            # 給 NaN 會讓 Nav2 整串 TF 變 NaN。反正 yaw_goal_tolerance 是 3.14。
            goal.pose.pose.orientation.w = 1.0
        else:
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        timeout = float(request.timeout) or self.goto_timeout
        self._nav_seq += 1
        seq = self._nav_seq
        self._nav = {"state": "running", "goal": [round(gx, 3), round(gy, 3)],
                     "seq": seq, "distance_remaining": None, "message": ""}
        self._begin_task(name, timeout)
        fut = self.nav_client.send_goal_async(goal, feedback_callback=self._on_fb)
        fut.add_done_callback(lambda f, q=seq: self._on_goal_response(f, q))
        return self._reply(response, True,
                           f"已送出導航目標 map=({gx:+.2f}, {gy:+.2f})，"
                           f"逾時 {timeout:.0f} 秒")

    def srv_stop(self, request, response):
        """停車：取消 Nav2 的目標，並主動送零速度。"""
        had = self._nav_handle is not None or self._task["state"] == "running"
        self._cancel_nav()
        self._stop_wheels()
        self._nav["state"] = "canceled"
        self._end_task(False, "被停止指令取消")          # 先把「正在跑的那個」判失敗
        self._wd = {"tripped": False, "reason": ""}     # 停好了就解除看門狗
        # 再開一個「停止」任務並當場完成 —— 積木要靠 seq 變化才知道做完了
        self._begin_task("停止", 5.0, done_now=True,
                         message="已停車" if had else "本來就沒有在動")
        return self._reply(response, True,
                           "已停車（取消導航並送出零速度）" if had else "已停車（本來就沒有在動）")

    def srv_state(self, request, response):
        if not self._odom_fresh():
            return self._reply(response, False, "沒有位置估計（收不到 odom）")
        with self._lock:
            x, y, yaw, v = self._odom
        self._begin_task("狀態查詢", 5.0, done_now=True,
                         message=f"map=({x:+.2f}, {y:+.2f})")
        return self._reply(response, True,
                           f"map=({x:+.2f}, {y:+.2f})　車頭 {math.degrees(yaw):+.0f}°　"
                           f"速度 {v:.2f} m/s")

    # ---------- Nav2 的回呼 ---------------------------------------------------
    def _on_fb(self, msg):
        self._nav["distance_remaining"] = round(
            float(msg.feedback.distance_remaining), 2)

    def _on_goal_response(self, fut, seq):
        if seq != self._nav_seq:
            return                       # 已被更新的 goto 取代
        try:
            h = fut.result()
        except Exception as e:                                   # noqa: BLE001
            self._nav.update(state="failed", message=f"送出失敗：{e}")
            self._end_task(False, f"導航目標送出失敗：{e}")
            return
        if not h.accepted:
            self._nav.update(state="failed", message="Nav2 拒絕了這個目標")
            self._end_task(False, "Nav2 拒絕了這個目標")
            return
        self._nav_handle = h
        h.get_result_async().add_done_callback(
            lambda f, q=seq: self._on_result(f, q))

    def _on_result(self, fut, seq):
        if seq != self._nav_seq:
            return
        self._nav_handle = None
        try:
            status = fut.result().status
        except Exception as e:                                   # noqa: BLE001
            self._nav.update(state="failed", message=str(e))
            self._end_task(False, f"導航結果讀取失敗：{e}")
            return
        # GoalStatus：4=SUCCEEDED 5=CANCELED 6=ABORTED
        if status == 4:
            self._nav.update(state="done", message="已到達", distance_remaining=0.0)
            self._end_task(True, "已到達導航目標")
        elif status == 5:
            self._nav.update(state="canceled", message="已取消")
            self._end_task(False, "導航被取消")
        else:
            self._nav.update(state="failed", message=f"Nav2 回報失敗（status={status}）")
            self._end_task(False, f"導航失敗（status={status}）")
        self._stop_wheels()              # 不論成敗都主動煞停，不要留速度給 gz


def main():
    rclpy.init()
    node = HuskyApi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_wheels()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
