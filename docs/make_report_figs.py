#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""畫「Husky 接入 Low-Code 工具」報告用的三張示意圖。

    docs/report/h01_分層架構.png   哪幾層沿用、哪幾層換掉、哪幾個是新寫的
    docs/report/h02_控制鏈路.png   指令下行與回授上行，無人機 vs 地面車對照
    docs/report/h03_TF與話題.png   兩台同場時的 TF 樹與話題命名

為什麼用 matplotlib 畫而不是截 Gazebo 的圖：
    這台機器的 Gazebo 無頭渲染走不通（EGL 建不出 dri2 screen），
    而且示意圖本來就不該是截圖 —— 截圖會隨材質改變而和文件脫節。
    現有報告的圖 1／3／4／5 也都是這樣畫的，風格要接得上。

用法：
    python3 docs/make_report_figs.py
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch   # noqa: E402

plt.rcParams["font.family"] = ["Noto Sans CJK JP", "Droid Sans Fallback"]
plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "report")

# 三種身分用顏色區分 —— 這是整份報告最重要的一個訊息：
#   沿用 = 一行都沒改；衍生 = 只改參數；新寫 = 這次才寫的程式
KEEP = dict(fc="#e8f5e9", ec="#43a047")      # 綠：沿用，零修改
DERIVE = dict(fc="#fff8e1", ec="#f9a825")    # 黃：衍生（只有參數不同）
NEW = dict(fc="#ffebee", ec="#e53935")       # 紅：這次新寫
PLAT = dict(fc="#eceff1", ec="#78909c")      # 灰：平台（外部既有）


def box(ax, x, y, w, h, text, style, fs=10, bold=False):
    ax.add_patch(FancyBboxPatch(
        (x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.08",
        linewidth=1.6, **style))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center",
            fontsize=fs, weight="bold" if bold else "normal", linespacing=1.5)


def arrow(ax, x1, y1, x2, y2, label="", fs=8.5, color="#455a64", dx=0.08):
    ax.add_patch(FancyArrowPatch(
        (x1, y1), (x2, y2), arrowstyle="-|>", mutation_scale=13,
        linewidth=1.3, color=color))
    if label:
        ax.text((x1 + x2) / 2 + dx, (y1 + y2) / 2, label, fontsize=fs,
                color=color, ha="left", va="center")


def legend(ax, x, y):
    for i, (st, name) in enumerate([
            (KEEP, "沿用（零修改）"), (DERIVE, "衍生（只改參數）"),
            (NEW, "這次新寫"), (PLAT, "平台／外部既有")]):
        ax.add_patch(FancyBboxPatch(
            (x + i * 2.45, y), 0.32, 0.22,
            boxstyle="round,pad=0.01,rounding_size=0.04",
            linewidth=1.4, **st))
        ax.text(x + i * 2.45 + 0.42, y + 0.11, name, fontsize=9, va="center")


def fig_layers():
    fig, ax = plt.subplots(figsize=(13, 8.6))
    ax.set_xlim(0, 13); ax.set_ylim(0, 8.6); ax.axis("off")
    ax.text(6.5, 8.25, "地面車接入 Low-Code 任務編排工具　分層架構",
            ha="center", fontsize=16, weight="bold")

    rows = [("操作層", 7.05), ("編排層", 5.95), ("橋接層", 4.95),
            ("決策層", 3.75), ("控制層", 2.25), ("平台層", 0.95)]
    for name, y in rows:
        ax.text(0.35, y + 0.35, name, fontsize=11, weight="bold",
                rotation=90, va="center", ha="center", color="#37474f")

    box(ax, 1.0, 7.0, 5.4, 0.85,
        "Node-RED 編輯器\n地面車 4 個積木（車：導航到座標／停止／狀態／返航）",
        KEEP, fs=10)
    box(ax, 6.8, 7.0, 5.2, 0.85,
        "Dashboard 2.0 儀表板\n新增「地面車」分頁，地圖 widget 直接沿用",
        KEEP, fs=10)

    box(ax, 1.0, 5.9, 11.0, 0.8,
        "任務流程：同一組積木、同一個「等待完成」判定\n"
        "完成協定 task.seq／task.state 與無人機完全一致", KEEP, fs=10.5)

    box(ax, 1.0, 4.9, 11.0, 0.75,
        "rosbridge_server（nodered_ros2）　ws://127.0.0.1:9090　JSON ↔ ROS 2\n"
        "本次改動 0 行 —— 這是「換載具不用改編排層」的直接證據",
        KEEP, fs=10.5, bold=False)

    box(ax, 1.0, 3.55, 5.2, 0.95,
        "husky_api_node（新寫）\napi/goto・api/stop・api/state ＋ api/status @5 Hz\n"
        "無人機 8 個服務，地面車只要 3 個", NEW, fs=9.8)
    box(ax, 6.8, 3.55, 5.2, 0.95,
        "（對照）drone_api_node\n既有，未修改\n"
        "「返航」兩台都在編排層組合，決策層不提供", KEEP, fs=9.8)

    box(ax, 1.0, 2.05, 5.2, 1.05,
        "Nav2（差速車參數）\nwarehouse_nav2_husky.yaml：footprint、max_vel_y=0、\n"
        "PathAlign／GoalAlign、RotationShimController", DERIVE, fs=9.8)
    box(ax, 6.8, 2.05, 5.2, 1.05,
        "cmd_vel_watchdog（新寫）\n0.5 秒沒有新指令就持續送零速度\n"
        "gz 的 DiffDrive 沒有指令逾時，缺這層會一路開到撞牆", NEW, fs=9.8)

    box(ax, 1.0, 0.75, 11.0, 0.95,
        "Gazebo Harmonic（取代無人機的 PX4 飛控層）\n"
        "DiffDrive 插件：v・ω → 四輪轉速　｜　OdometryPublisher：真值位姿\n"
        "ros_gz_bridge 負責 ROS 2 ↔ gz 的型別轉換", PLAT, fs=10)

    # 下行：操作 → 編排 → 橋接 → 決策 → 控制
    for y1, y2 in [(7.0, 6.7), (5.9, 5.65), (4.9, 4.5), (3.55, 3.1)]:
        arrow(ax, 3.6, y1, 3.6, y2)
    # ⚠️ 速度指令不是從 Nav2 直接下到平台，中間一定要經過看門狗
    arrow(ax, 6.2, 2.58, 6.8, 2.58)
    ax.text(6.5, 2.78, "cmd_vel", fontsize=8.5, color="#455a64", ha="center")
    arrow(ax, 9.4, 2.05, 9.4, 1.7)

    legend(ax, 1.0, 0.08)
    fig.savefig(os.path.join(OUT, "h01_分層架構.png"), dpi=130,
                bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("h01_分層架構.png")


def fig_chain():
    """速度指令的傳遞路徑：無人機 vs 地面車。

    刻意不做顏色編碼 —— 「哪些沿用、哪些新寫」是圖 1 的工作，這張只負責
    把兩條路徑擺在一起比。靠的是版面：左右對齊的兩欄、左側層次標籤、
    中間一條分界線。全部灰階，印黑白也不會失真。
    """
    NEUTRAL = dict(fc="white", ec="#5f7079")
    SHARED = dict(fc="#eceff1", ec="#5f7079")

    fig, ax = plt.subplots(figsize=(13, 10))
    ax.set_xlim(0, 13); ax.set_ylim(0, 10); ax.axis("off")

    GX, LX, LW, RX, RW = 1.25, 1.35, 4.85, 7.10, 4.85
    LC, RC = LX + LW / 2, RX + RW / 2
    SPINE = (LX + LW + RX) / 2

    ax.text(6.08, 9.62, "速度指令送到哪裡去：無人機與地面車的控制鏈路對照",
            ha="center", fontsize=16.5, weight="bold")

    for x, w, t in [(LX, LW, "無人機 x500（既有）"),
                    (RX, RW, "地面車 Husky A200（本次）")]:
        ax.add_patch(FancyBboxPatch(
            (x, 9.0), w, 0.44, boxstyle="round,pad=0.02,rounding_size=0.1",
            fc="#e0e4e7", ec="#5f7079", linewidth=1.5))
        ax.text(x + w / 2, 9.22, t, ha="center", va="center",
                fontsize=12.5, weight="bold", color="#263238")

    box(ax, 3.9, 8.25, 5.2, 0.52,
        "Node-RED 積木 → rosbridge　（兩台共用同一套）", SHARED, fs=10.5)

    rows = [
        ("決策", "/MAV1/api/goto\n（8 個服務之一）",
         "/husky/api/goto\n（3 個服務之一）"),
        ("導航", "Nav2（全向參數）", "Nav2（差速車參數）"),
        ("速度指令", "/MAV1/cmd_vel　Twist",
         "/husky/cmd_vel　Twist（只有 vx、ωz）"),
        ("安全", "cmd_vel_to_px4_node\nENU→NED、相對高度、offboard 心跳",
         "cmd_vel_watchdog\n0.5 s 沒有新指令就送零速度"),
        ("傳輸", "TrajectorySetpoint（uORB）\nuXRCE-DDS",
         "ros_gz_bridge\ngeometry_msgs/Twist → gz.msgs.Twist"),
        ("控制核心", "PX4 v1.17　四層閉環\n位置 → 速度 → 姿態 → 角速率 → 混控",
         "DiffDrive 插件\nv・ω → 四輪轉速（等效輪距 0.834）"),
        ("致動", "四顆馬達 PWM", "四個輪 joint → 物理引擎"),
    ]

    # 先算好每一列的位置，箭頭才接得準（上下兩列高度不同時尤其重要）
    top, places = 7.85, []
    for i, (_, lt, rt) in enumerate(rows):
        h = 0.78 if ("\n" in lt or "\n" in rt) else 0.56
        places.append((top - h, h))
        # 第三列之後多留一點，分界線的說明標籤要放進那個空隙
        top = top - h - (0.62 if i == 2 else 0.30)

    for i, ((y, h), (lay, lt, rt)) in enumerate(zip(places, rows)):
        if i % 2 == 0:                       # 交錯底色，左右對應不會看錯行
            ax.add_patch(plt.Rectangle((0.08, y - 0.09), 12.0, h + 0.18,
                                       fc="#f4f6f7", ec="none", zorder=0))
        ax.text(GX, y + h / 2, lay, ha="right", va="center",
                fontsize=10, color="#455a64", weight="bold")
        box(ax, LX, y, LW, h, lt, NEUTRAL, fs=9.8)
        box(ax, RX, y, RW, h, rt, NEUTRAL, fs=9.8)

    # 起點分到兩欄：先往下、再分左右，不畫斜線
    ax.plot([SPINE, SPINE], [8.25, 8.05], color="#455a64", lw=1.3)
    ax.plot([LC, RC], [8.05, 8.05], color="#455a64", lw=1.3)
    for cx in (LC, RC):
        arrow(ax, cx, 8.05, cx, places[0][0] + places[0][1])
    for (y1, _), (y2, h2) in zip(places, places[1:]):
        arrow(ax, LC, y1, LC, y2 + h2)
        arrow(ax, RC, y1, RC, y2 + h2)

    # 分界線：以上結構相同，以下各做各的
    ysep = (places[2][0] + places[3][0] + places[3][1]) / 2
    ax.plot([0.08, 12.08], [ysep, ysep], ls=(0, (6, 4)),
            color="#90a4ae", lw=1.4, zorder=1)
    ax.add_patch(FancyBboxPatch(
        (SPINE - 1.75, ysep - 0.22), 3.5, 0.44,
        boxstyle="round,pad=0.02,rounding_size=0.08",
        fc="white", ec="#90a4ae", linewidth=1.0, zorder=3))
    ax.text(SPINE, ysep, "↑ 結構相同，只有參數不同\n↓ 與平台相依，各自實作",
            fontsize=8.8, color="#455a64", ha="center", va="center",
            linespacing=1.45, zorder=4)

    for cx, t in [(LC, "有平台級保護：offboard 心跳斷掉會觸發 failsafe"),
                  (RC, "無平台級保護：DiffDrive 沒有指令逾時，看門狗是手工補的")]:
        ax.text(cx, 0.52, t, ha="center", va="center",
                fontsize=9.3, color="#37474f", style="italic")

    fig.savefig(os.path.join(OUT, "h02_控制鏈路.png"), dpi=130,
                bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("h02_控制鏈路.png")


def fig_tf():
    fig, ax = plt.subplots(figsize=(12, 6.4))
    ax.set_xlim(0, 12); ax.set_ylim(0, 6.4); ax.axis("off")
    ax.text(6, 6.05, "兩台載具同場：TF 樹與命名的隔離方式",
            ha="center", fontsize=16, weight="bold")

    box(ax, 4.9, 5.0, 2.2, 0.55, "map\n（兩台共用）", KEEP, fs=10)
    box(ax, 1.4, 3.7, 3.0, 0.55, "odom", KEEP, fs=10)
    box(ax, 7.6, 3.7, 3.0, 0.55, "husky/odom", NEW, fs=10)
    box(ax, 1.4, 2.5, 3.0, 0.55, "base_link", KEEP, fs=10)
    box(ax, 7.6, 2.5, 3.0, 0.55, "husky/base_link", NEW, fs=10)
    arrow(ax, 5.4, 5.0, 2.9, 4.25, "px4_tf_node", fs=8.5, dx=-1.5)
    arrow(ax, 6.6, 5.0, 9.1, 4.25, "靜態 TF（全零）", fs=8.5, dx=0.25)
    arrow(ax, 2.9, 3.7, 2.9, 3.05, "")
    arrow(ax, 9.1, 3.7, 9.1, 3.05, "OdometryPublisher（真值）", fs=8.5)

    ax.text(6, 1.95, "map 共用是對的 —— 兩台在同一張地圖裡；"
                     "會撞名的是 odom 與 base_link，加 husky/ 前綴即可隔離",
            ha="center", fontsize=10, color="#37474f")

    rows = [("項目", "無人機", "地面車"),
            ("導航 action", "/navigate_to_pose", "/husky/navigate_to_pose"),
            ("服務", "/MAV1/api/*", "/husky/api/*"),
            ("速度指令", "/MAV1/cmd_vel", "/husky/cmd_vel"),
            ("Nav2 節點", "全域命名空間", "推進 /husky 命名空間")]
    y0 = 1.55
    for r, (a, b, c) in enumerate(rows):
        yy = y0 - r * 0.3
        w = "bold" if r == 0 else "normal"
        ax.text(1.4, yy, a, fontsize=9.5, weight=w)
        # ⚠️ 等寬字型沒有中文字符，有中文的格子不能用，否則會變成空心方框
        mono = lambda t: "monospace" if r and t.isascii() else None   # noqa: E731
        ax.text(4.3, yy, b, fontsize=9.5, weight=w, family=mono(b))
        ax.text(8.0, yy, c, fontsize=9.5, weight=w, family=mono(c))
        if r == 0:
            ax.plot([1.3, 11.0], [yy - 0.11, yy - 0.11], color="#90a4ae", lw=1)

    fig.savefig(os.path.join(OUT, "h03_TF與話題.png"), dpi=130,
                bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("h03_TF與話題.png")


def fig_trajectory():
    """用實際錄到的真值軌跡畫圖（test/sim/trajectory.csv）。"""
    import csv as _csv
    path = os.path.join(HERE, "..", "test", "sim", "trajectory.csv")
    if not os.path.exists(path):
        print("沒有 trajectory.csv，跳過"); return
    legs = {}
    with open(path, encoding="utf-8") as f:
        for r in _csv.DictReader(f):
            legs.setdefault(r["leg"], []).append((float(r["x"]), float(r["y"])))

    # 場景幾何直接取自系統自己回報的 scene.boxes（api/status），
    # 不另外抄一份數字，圖和世界檔就不會脫節。
    shelves = [(-9.2, -7.0, -7.6, 7.0), (-3.6, -7.0, -2.0, 7.0),
               (2.0, -7.0, 3.6, 7.0), (7.6, -7.0, 9.2, 7.0)]
    walls = [(-20.0, -14.0, -18.0, 14.0), (18.0, -14.0, 20.0, 14.0),
             (-20.0, -14.0, 20.0, -12.0), (-20.0, 12.0, 20.0, 14.0)]

    fig, ax = plt.subplots(figsize=(12, 7.4))
    for x1, y1, x2, y2 in walls:
        ax.add_patch(plt.Rectangle((x1, y1), x2 - x1, y2 - y1,
                                   fc="#b0bec5", ec="none"))
    for i, (x1, y1, x2, y2) in enumerate(shelves):
        ax.add_patch(plt.Rectangle((x1, y1), x2 - x1, y2 - y1, fc="#90a4ae",
                                   ec="#546e7a", label="貨架" if i == 0 else None))
    cols = {"去東南角": "#e53935", "返航": "#1e88e5"}
    for name, pts in legs.items():
        xs, ys = zip(*pts)
        ax.plot(xs, ys, color=cols.get(name, "#333"), lw=2.2, label=f"{name}（實測軌跡）")
    for name, pts in legs.items():
        ax.plot(*pts[-1], "o", color=cols.get(name, "#333"), ms=9, mec="white")
    ax.plot(-14, 9, "s", color="#2e7d32", ms=11, mec="white", label="起點／home (-14, 9)")
    ax.plot(5.6, -9, "*", color="#fb8c00", ms=20, mec="white", label="導航目標 (5.6, -9)")
    # 直線穿過去是穿不過的 —— 這條虛線就是在講這件事
    ax.plot([-14, 5.6], [9, -9], ls=":", color="#757575", lw=1.4,
            label="起點到目標的直線（被四排貨架擋住）")

    ax.set_xlim(-21, 21); ax.set_ylim(-15, 15); ax.set_aspect("equal")
    ax.set_xlabel("map x (m)"); ax.set_ylabel("map y (m)")
    ax.set_title("地面車導航實測軌跡（Gazebo 真值，2026-11-09）",
                 fontsize=14, weight="bold")
    ax.grid(alpha=0.25); ax.legend(loc="upper right", fontsize=9, framealpha=0.95)
    fig.savefig(os.path.join(OUT, "h05_導航軌跡.png"), dpi=130,
                bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("h05_導航軌跡.png")


def fig_blocks():
    """積木面板與一條任務串接（示意圖，不是編輯器截圖 —— 編輯器有登入保護）。

    積木名稱、參數欄位、出口數都是從 nodered/flows/husky_basic.json 讀出來的，
    不是手打的，所以不會和實際裝進去的流程脫節。
    """
    import json as _json
    fp = os.path.join(HERE, "..", "nodered", "flows", "husky_basic.json")
    subs = [n for n in _json.load(open(fp, encoding="utf-8"))
            if n.get("type") == "subflow" and n.get("name", "").startswith("車：")]

    fig, ax = plt.subplots(figsize=(13, 7.2))
    ax.set_xlim(0, 13); ax.set_ylim(0, 7.2); ax.axis("off")
    ax.text(6.5, 6.95, "地面車的四個積木，以及它們和無人機共用的部分",
            ha="center", fontsize=15.5, weight="bold")

    # 左：左側面板的三個分類
    ax.text(1.9, 6.42, "編輯器左側面板", fontsize=11.5, weight="bold")
    cats = [("ROS 2（通用）", "ROS2 呼叫服務、ROS2 訂閱", KEEP, "2 個　沿用，零修改"),
            ("無人機", "起飛／降落／盤旋／追蹤目標…", KEEP, "10 個　未更動"),
            ("地面車", "車：導航到座標／停止／狀態查詢／返航", NEW, "4 個　本次新增")]
    y = 5.45
    for name, items, st, note in cats:
        box(ax, 0.5, y, 5.0, 0.78, f"{name}\n{items}", st, fs=9.4)
        ax.text(0.62, y - 0.17, note, fontsize=8.6, color="#546e7a")
        y -= 1.18

    ax.text(1.9, 1.75,
            "積木同名會被合併工具當成同一個，\n所以地面車這四個一律加「車：」前綴",
            fontsize=9, color="#c62828", linespacing=1.6)

    # 右：一個積木的樣子 + 串接
    ax.text(9.3, 6.42, "積木內部與串接方式", fontsize=11.5, weight="bold")
    sub = [x for x in subs if x["name"] == "車：導航到座標"][0]
    envs = "\n".join("・" + (e.get("ui", {}).get("label", {}).get("zh-TW") or e["name"])
                     for e in sub.get("env", []))
    box(ax, 6.6, 4.15, 5.9, 2.0,
        f"{sub['name']}\n\n{envs}", NEW, fs=9.4)
    ax.text(12.42, 5.62, "完成 →", fontsize=8.6, ha="right", color="#2e7d32")
    ax.text(12.42, 4.52, "失敗 →", fontsize=8.6, ha="right", color="#c62828")

    ax.text(9.5, 3.9, "兩個出口的語意與無人機完全相同", fontsize=9,
            ha="center", color="#546e7a")

    chain = [("儀表板按鈕", KEEP), ("車：導航到座標", NEW),
             ("車：狀態查詢", NEW), ("車：返航", NEW)]
    x = 6.6
    for i, (t, st) in enumerate(chain):
        box(ax, x, 2.45, 1.32, 0.62, t, st, fs=8.4)
        if i < len(chain) - 1:
            arrow(ax, x + 1.32, 2.76, x + 1.5, 2.76)
        x += 1.5
    ax.text(9.55, 2.15, "串接時不需要寫等待邏輯：積木自己等 task.seq 變化",
            fontsize=9, ha="center", color="#546e7a")

    box(ax, 6.6, 0.95, 5.9, 0.82,
        "判定完成的依據（兩種載具同一份協定）\n"
        "送出時記下 task.seq → 等 seq 改變且 task.state 不是 running",
        KEEP, fs=9.2)

    legend(ax, 2.6, 0.15)
    fig.savefig(os.path.join(OUT, "h06_積木.png"), dpi=130,
                bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("h06_積木.png")


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    fig_layers()
    fig_chain()
    fig_tf()
    fig_trajectory()
    fig_blocks()
    print("→", OUT)
