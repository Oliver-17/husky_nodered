#!/usr/bin/env python3
# =============================================================================
#  wait_for_action.py — 等某個 action server 出現，然後結束
#
#  用法（給 launch 當閘門用，不是給人打的）：
#      wait_for_action.py /navigate_to_pose --settle 5 --timeout 120
#
#  做什麼：輪詢指定的 action server，出現之後再等 --settle 秒才結束。
#  不論成功或逾時都是**正常結束**（exit 0）—— 因為 launch 那邊是用
#  OnProcessExit 接的，非零離開碼不會改變行為，只會多印紅字。
#  真正要讓人看到的是它印出來的那一行。
#
#  ---------------------------------------------------------------------------
#  為什麼需要這支（2026-11-15）
#  ---------------------------------------------------------------------------
#  兩套 Nav2 **不能同時 configure** —— 後起的那套會卡在
#  「Configuring map_server」，而且卡住之後不會自己重試，要整個重開。
#
#  原本的解法是「地面車的 Nav2 延後 35 秒」。但實測（2026-11-15）每套 Nav2
#  從啟動到就緒只要 3~4 秒：
#      t=35 無人機 Nav2 啟動 → t=39 就緒 → ... 空等 31 秒 ... → t=70 地面車啟動
#  也就是說那 35 秒裡有 31 秒是純浪費的，而且還是用猜的 —— 機器更忙的時候
#  反而可能不夠。
#
#  改成等事件就兩個問題一起解決：快，而且慢的機器上自動等更久。
#
#  ⚠️ 為什麼看到 action server 之後還要再等 --settle 秒：
#     Nav2 的 action server 是在 **on_configure** 就建立的，不是 on_activate。
#     所以「看得到」只代表對方 configure 完了，可能還在 activate
#     （實測那段約 2 秒）。多等幾秒才真的錯開。
#
#  ⚠️ 逾時也要放行，不能永遠等：無人機那套如果根本沒起來（例如只跑地面車），
#     卡在這裡會讓地面車永遠不會動。對方沒起來時本來就沒有搶 configure 的問題。
# =============================================================================

import argparse
import sys
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("action", help="要等的 action 名稱，例如 /navigate_to_pose")
    ap.add_argument("--settle", type=float, default=5.0,
                    help="看到之後再等幾秒（讓對方 activate 完）")
    ap.add_argument("--timeout", type=float, default=120.0,
                    help="等多久就放棄（放棄也會正常結束，見檔頭）")
    args = ap.parse_args()

    rclpy.init()
    node = Node("wait_for_action")
    client = ActionClient(node, NavigateToPose, args.action)

    t0 = time.time()
    found = False
    last_log = 0.0
    while time.time() - t0 < args.timeout:
        # server_is_ready() 不阻塞；wait_for_server 會阻塞，這裡要能自己控節奏
        if client.server_is_ready():
            found = True
            break
        rclpy.spin_once(node, timeout_sec=0.2)
        if time.time() - last_log > 10:
            last_log = time.time()
            print(f"  等 {args.action} 出現…（{time.time() - t0:.0f} 秒）", flush=True)

    took = time.time() - t0
    if found:
        print(f"✓ {args.action} 出現了（{took:.0f} 秒）—— 再等 {args.settle:.0f} 秒"
              "讓它 activate 完", flush=True)
        end = time.time() + args.settle
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    else:
        print(f"⚠ 等了 {args.timeout:.0f} 秒還是沒看到 {args.action} —— "
              "照常繼續（對方沒起來的話本來就不會搶 configure）", flush=True)

    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
