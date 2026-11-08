#!/usr/bin/env python3
# =============================================================================
#  cmd_vel_watchdog.py — 速度指令的看門狗
#
#  當初是什麼症狀（2026-11-07）：
#      Husky 在倉庫裡「一直撞牆壁」。查下去發現 gz-sim 的 DiffDrive
#      **沒有指令逾時**：只要有人送過一筆 cmd_vel，之後就算再也沒有人送，
#      它也會永遠沿用最後那一筆速度。於是只要 Nav2 停止發指令
#      （目標結束、節點被關掉、程式當掉），車子就會用最後的速度一直開到撞牆，
#      而且輪子在牆邊空轉會讓輪式里程計狂飆 —— 位置估計整個失真。
#
#  這支做的事：
#      訂 <in>（Nav2 發的），原樣轉發到 <out>（bridge 送進 gz 的）。
#      超過 timeout 沒收到新指令，就持續送零速度。
#
#  無人機那邊有對應的東西：cmd_vel_to_px4_node 的 cmd_timeout_s（0.5 秒），
#  逾時會釋放控制權。這支是地面車版本，同一個道理 ——
#  安全保護要用強制機制，不能靠「上游一定會記得送零」。
# =============================================================================
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelWatchdog(Node):
    def __init__(self):
        super().__init__("cmd_vel_watchdog")
        self.declare_parameter("in_topic", "cmd_vel")
        self.declare_parameter("out_topic", "cmd_vel_safe")
        self.declare_parameter("timeout_s", 0.5)
        self.declare_parameter("rate_hz", 20.0)
        p = self.get_parameter
        self.timeout = p("timeout_s").value
        self.pub = self.create_publisher(Twist, p("out_topic").value, 10)
        self.sub = self.create_subscription(
            Twist, p("in_topic").value, self.on_cmd, 10)
        self.last = None            # 最後一筆指令
        self.last_t = None          # 收到的時間
        self.stopped = True         # 已經在送零了就不用一直印訊息
        self.create_timer(1.0 / p("rate_hz").value, self.tick)
        self.get_logger().info(
            f"看門狗啟動：{p('in_topic').value} → {p('out_topic').value}，"
            f"逾時 {self.timeout} 秒就送零速度")

    def on_cmd(self, msg):
        self.last = msg
        self.last_t = self.get_clock().now()
        if self.stopped:
            self.stopped = False

    def tick(self):
        now = self.get_clock().now()
        if self.last_t is None:
            return                                  # 還沒有人下過指令，什麼都不送
        age = (now - self.last_t).nanoseconds / 1e9
        if age <= self.timeout:
            self.pub.publish(self.last)
            return
        if not self.stopped:
            self.get_logger().warn(
                f"{age:.1f} 秒沒收到 cmd_vel —— 送零速度停車"
                "（gz 的 DiffDrive 不會自己停）")
            self.stopped = True
        self.pub.publish(Twist())                   # 全零


def main():
    rclpy.init()
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())                   # 收工前再送一次零
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
