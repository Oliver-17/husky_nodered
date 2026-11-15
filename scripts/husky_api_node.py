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
#      ros2 service call /husky/api/track vehicle_nodered_interfaces/srv/Track \
#          "{distance: 3.0, altitude: 0.0, duration: 60.0, \
#            search_x: [5.6, 5.6, 0.0, 0.0], search_y: [-9.0, 9.0, 9.0, -9.0]}"
#      ⚠️ 環線順序和車**反向**（對頭相遇最快，19 秒 vs 同向 31 秒）。
#         碰到之後車會一路倒車，**退過路口、停在環線外側讓路**，等目標轉彎
#         從車前面過去，才翻成同向從後面追 —— 見 TRACK_POCKET_M 與 _check_corner。
#
#  ── 為什麼只有四個服務 ───────────────────────────────────────────────────
#  無人機那邊有八個，這裡只要四個，差別不是功能縮水而是載具不同：
#      起飛／降落／調整高度／盤旋   地面車沒有這些概念
#      巡航（DO_REPOSITION）        那是飛控內建的，地面車沒有對應品
#
#  ── 追蹤和無人機的作法不一樣（2026-11-14）─────────────────────────────────
#  無人機的 api/track 自己算速度指令餵 offboard；地面車**全部走 Nav2**：
#  每次都算出一個「站位點」當 NavigateToPose 的目標。理由：
#      ① 倉庫有貨架。自己發 cmd_vel 等於沒有避障。
#      ② 滑移轉向的車不能像無人機那樣側移靠過去，一定得先轉頭再前進 ——
#         那正是 Nav2 的 RotationShim 在做的事（H2 就是為了它調的參數）。
#      ③ 不用去搶 cmd_vel，不必處理和 Nav2、看門狗的互斥。
#  代價是「一個任務、很多個 Nav2 目標」，所以 Nav2 的回呼要分流 ——
#  見 _on_result 裡的說明。
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
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from nav2_msgs.action import BackUp, NavigateToPose, Spin
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger
from vehicle_nodered_interfaces.srv import GoTo, Track

ODOM_STALE_S = 2.0          # 超過這麼久沒收到 odom 就當位置估計失效
STATUS_HZ = 5.0             # 和無人機一致

# --- 追蹤（2026-11-14）---------------------------------------------------------
# 這幾個都開成節點參數，下面只是預設值。會需要調的理由寫在各自的參數說明裡。
TRACK_STANDOFF_M = 3.0      # 停在目標前方多遠
TRACK_DURATION_S = 120.0    # 追多久
TRACK_FRESH_S = 1.5         # 目標位置多新才拿來算站位
TRACK_SEARCH_AFTER_S = 5.0  # 多久沒看到才開始跑搜尋環線
# ⚠️ 下面兩個 2026-11-15 起**沒有作用**：跟隨改成直接送速度，不再一直送 Nav2
#    目標，這兩道重送閘就沒有東西可擋了。現在只剩宣告。
TRACK_GOAL_INTERVAL_S = 2.0 # （已無作用）兩次重新給 Nav2 目標的最小間隔
TRACK_GOAL_MOVE_M = 0.5     # （已無作用）站位點移動超過這麼多才值得重新給
TRACK_ARRIVE_TOL_M = 0.5    # 站位距離的容許帶。現在只用在 track_close_m 的關係檢查
TRACK_MAX_NAV_FAIL = 3      # 連續幾次 Nav2 失敗就放棄

# --- 近距離跟丟的處理（2026-11-14）-------------------------------------------
# 相機的可用範圍是 2~8 m（README 的 H6）。目標會自己朝車開過來，靠到 2 m 以內
# 就偵測不到了 —— 站位 3 m 擋不住這件事。兩段處理：
#   ① 先擋：太近就倒車。2026-11-15 起由跟隨的速度控制律直接給**負的 v**
#      （follow_cmd / follow_world），車頭照樣對著目標，不再發 BackUp。
#      2026-11-14 那版是用 Nav2 的 BackUp 行為一段一段退。
#   ② 真的跟丟了：先原地轉一圈找，再退一點找，都沒有才回去跑搜尋環線。
#      這裡的「退」還是 BackUp。⚠️ 關鍵是 **BackUp 不轉向**（2026-11-14 實測：
#      target.x=+1.0 → 沿車頭方向 -1.027 m、車頭變化 0.0°），所以退完車頭
#      還是對著原本的方向。這和「送一個在車後面的導航目標」完全不同 ——
#      那個會讓規劃器把車掉頭開過去，變成背對目標。
# ⚠️ 2.5 不是 2.0（2026-11-14 實測後調的）：目標是**朝車開過來**的，
#    等到 2.0 才開始退已經太晚 —— 退的過程中距離還在縮。早一點開始有餘裕。
# ⚠️ 2026-11-15 起「什麼時候開始倒車」改由速度控制律決定（距離小於
#    站位 − 死區，也就是 3.0 − 0.3 = 2.7 m 就開始退），這個值只剩
#    track_close_m 的關係檢查在用。
TRACK_MIN_M = 2.5
# 跟丟時「最後看到的距離」小於這個，才值得做近距離重找（轉圈／倒車）；
# 更遠的跟丟（目標繞到貨架後面）直接跑搜尋環線比較快。
#
# ⚠️ 這個值**必須大於車實際維持的距離帶**，也就是
#        max(track_min_m, track_standoff_m + track_arrive_tol_m)
#    否則重找永遠不會觸發 —— 車維持在 2.5 m，而門檻也是 2.5，
#    跟丟時 last_dist 剛好不小於門檻（2026-11-14 實測踩到，整段重找是死的）。
#    預設是 3.0 + 0.5 = 3.5，所以取 4.0 留 0.5 m 餘裕。節點啟動時會檢查並警告。
TRACK_CLOSE_M = 4.0
# ⚠️ 兩次調過（都是實測後）：0.2 → 0.4（2026-11-14）→ **0.7**（2026-11-15）。
#    倉庫的目標車 move_target.py 預設跑 0.4 m/s，倒車只要「不比它慢」理論上
#    就守得住距離 —— 但 0.4 時 Oliver 實際看到「目標開的比倒退的快」。
#    原因是倒車要先從 0 加速，而且 BackUp 是一段一段發的（每段之間有空檔），
#    平均速度達不到設定值。0.7 留餘裕。
#    2026-11-15 起這個值有兩個用途：跟隨時速度控制的**倒車上限**，以及
#    重找時 BackUp 的速度。
#    上限是車體本身的 1.0（model.sdf 的 max_linear_velocity，A200 原廠上限）。
#    代價：盲退變快。BackUp 有用區域 costmap 做 simulate_ahead_time 檢查，
#    但那張圖只有 static_layer（Husky 沒有後方感測器），所以只擋得住
#    地圖上畫的貨架和牆，擋不住臨時出現的東西。
TRACK_BACKUP_SPEED = 0.7    # 倒車速度（m/s）
TRACK_BACKUP_MAX_M = 2.0    # 一次最多退這麼遠
# --- 跟隨階段的速度控制（2026-11-15 從 Nav2 位置目標改過來）-------------------
#
# ⚠️ 為什麼非改不可：`NavigateToPose` 是「開到那個點然後停」，目標在動就必定
#    反覆衝刺。2026-11-15 量化過：真正在追的時候平均只有 **0.60 m/s**
#    （目標 0.5、車體上限 1.0），149 秒跟丟 7 次。
#    算得出來：站位點只在前方約 1.1 m，而 acc_lim_x 1.0 / decel_lim_x -1.5
#    之下跑 1.1 m 要 1.93 秒 → 平均 0.57 m/s。和實測完全吻合。
#
# 🔑 **無人機 2026-10-16 已經走過同一條路**（drone_api_node.py:151-200）：
#    它原本用 DO_REPOSITION，量到「距離比設定值多 1.25 m = 2.5 秒反應延遲」，
#    試過拉高更新率、縮小容差、調加速度與加加速度限制 —— **完全沒有改善**，
#    全部還原。那邊的結論是：
#        「問題不在調參，而在『用位置設定點追移動目標』這個架構」
#    所以這裡不要再試「把目標點拉遠一點」那類調整，那已經被證明是死路。
#
# 搜尋階段**維持 Nav2**（「開到航點」本來就該用位置控制，而且那段有避障）。
# 兩段分開正好和無人機一致。
#
# ⚠️ 和無人機唯一真正的差別：滑移轉向**不能側移**。無人機可以直接下
#    (vx, vy) 往任意方向飛；這裡只能下「前進速度 v + 轉向角速度 ω」，
#    而且方位誤差大的時候要把 v 壓低，先轉頭再走，否則會畫一個大弧。
TRACK_CMD_HZ = 10.0          # cmd_vel 的發送頻率。看門狗 2 秒逾時，10 Hz 餘裕很大
# ⚠️ 2.0 不是 0.8（2026-11-15 實測後調的）。
#    比例控制的平衡點是「誤差大到 kp×誤差 = 目標速度」才跟得住：
#        kp 0.8、目標 0.5 m/s → 要落後 0.625 m 才給得出 0.5 m/s
#    實測就是這樣：相距 3.5 m（站位 3.0）時速度只給 0.40，比目標還慢，
#    於是一路被拉開。kp 2.0 之下只要落後 0.25 m 就給得出 0.5 m/s，
#    落後 0.5 m 就直接滿速 1.0。
#    不怕衝過頭：v 有上限 1.0，而且太近時 e_d 變負的會自動倒車。
TRACK_KP_LIN = 2.0           # 距離誤差（m）→ 前進速度（m/s）
TRACK_KP_YAW = 1.5           # 方位誤差（rad）→ 角速度（rad/s）。抄無人機的值
TRACK_MAX_SPEED = 1.0        # 前進上限。車體本身就是 1.0（A200 原廠上限）
TRACK_MAX_YAWRATE = 1.0      # 角速度上限。和 Nav2 的 max_vel_theta 一致
TRACK_SPEED_DEADBAND_M = 0.3 # 距離誤差小於這個就不動，免得一直來回修
# 方位誤差超過這個就**只轉頭不前進**。90° 以上往前開等於朝反方向跑。
TRACK_YAW_ONLY_RAD = 1.2     # ≈ 69°

# --- 跟隨時不撞牆（2026-11-15，照抄無人機的禁飛方框）-------------------------
# ⚠️ 舊的擋牆（_blocked_ahead）有三個洞，Oliver 實測「轉彎幾乎每次都撞」：
#      ① 只檢查車中心線前方 1 秒的**一個點**。車寬 0.67 m，轉彎時車身側面
#         已經擦到貨架，那個點還在外面
#      ② 方框**沒有加上車身大小**（無人機那邊有 no_fly_margin，這裡沒有）
#      ③ 擋到時只把前進速度歸零，車頭還是對著貨架後面的目標 → 卡在貨架上
# 改成 drone_api_node.py 的 _clamp_no_fly 同一套：
#      方框加大 → 先夾「想往哪走」的世界座標速度，**只砍朝牆那一面的分量**，
#      剩下的就是沿著牆面滑行 → 再換成差速車的 v、ω。
# 和無人機唯一的差別：無人機可以直接往任何方向飛；差速車要先把車頭轉到
# 夾完的方向才走得過去（見 follow_world）。
#
# ⚠️ 這**不是**避障。方框是 warehouse_geometry.yaml 的貨架與外牆，目標車、
#    無人機、臨時放的東西都不在裡面 —— 和無人機那邊的限制一樣。
# ⚠️ 環線離貨架面 2.0 m，加大 0.8 m 之後走道還剩 1.2 m，環線本身不會被擋。
#    自己在積木裡改搜尋環線時，離貨架面要 > 0.8 m，否則跟隨會被擋在環線外。
TRACK_WALL_MARGIN_M = 0.8     # 車身外接半徑 0.6（0.99x0.67 的半對角）＋ 0.2；無人機也是 0.8
TRACK_WALL_LOOKAHEAD_S = 1.0  # 預判幾秒，和無人機一樣
# 照實際車頭方向走會被砍掉多少：剩下不到一半就不走、只轉頭。
# 差速車不能只走「一部分方向」—— 車頭斜斜對著牆時，與其慢慢擠過去，
# 不如先轉到沿牆的方向再走。
TRACK_WALL_MIN_FRAC = 0.5
# 🔴 差速車和無人機真正不一樣的地方：**車身是長方形，原地轉頭會掃到東西**。
#    2026-11-15 模擬實測：車中心離貨架角 0.52 m 時原地轉了 26°，車尾掃上貨架角。
#    無人機在這方面是圓的，所以那邊的作法沒有這一層。
#    車身外接半徑 = 半對角線 √(0.495² + 0.335²) = 0.598 m。中心離所有貨架/牆
#    都超過這個值，怎麼轉都碰不到；小於它就不准轉，先沿車身軸直直移開。
TRACK_ROT_CLEAR_M = 0.65      # 外接半徑 0.6 ＋ 0.05
# 已經在加大後的方框裡時，車頭要多接近「允許的方向」才准走。
# ⚠️ 不能是 1.0（＝完全不准有往裡的分量）—— 2026-11-15 單元測試踩到死結：
#    車頭靠 ω = kp × 誤差 轉過去，誤差是**漸近**收斂的，永遠停在 +0.01° 這種
#    值，帶著一點點往裡的分量 → 永遠 v=0 → 卡在貨架角旁邊動不了。
#    0.97 ≈ 車頭差 10° 以內就准走，往裡的漂移最多是速度的 17%，
#    而從餘裕區開出去只要幾十公分（漂移幾公分），轉頭保護那一層還在後面兜底。
TRACK_WALL_INSIDE_FRAC = 0.97
TRACK_ESCAPE_SPEED = 0.2      # 直直移開的速度（只移幾十公分，慢一點穩）

# --- 對頭相遇時倒車讓路（2026-11-15）-----------------------------------------
# ⚠️ 下午那版「退到環線邊界就停」把車停在**路口前 0.6 m**（實測 y=8.55，
#    路口 y=9）—— 剛好擋在目標的路上，目標車沒有碰撞箱，於是直接穿過去，
#    看起來像「沒有換成順向追的機制」。
#    原因：它檢查的是車後方 1.4 m 那個點有沒有超出環線外框 + 0.8 m。
#
# 正確的位置是**路口外側**：單線道上要讓目標從後面變成前面，只能讓它
# 從車旁邊過去，而唯一不在它路上的地方就是路口往外延伸的那一小段。
#
#        北牆 y=12（加大後 11.2）
#            [車]   ← 車中心停在約 y=10.5，車頭 y=10.0
#      ──────●──────── y=9 路口：目標在這裡轉彎（車頭最遠到 y≈9.3），從車前面過
#            │
#            ▲ 目標（對頭開過來）
#
# 不能改成「倒進下一段環線」：目標轉彎後又會朝車開過來，車永遠在它前面。
#
# 車中心最多離開環線外框這麼遠。實際停的位置還會被北/南牆的方框
# （加大後 ±11.2）再擋一次，四個路口外側到牆都有 3 m，兩道限制不會互相打架。
TRACK_POCKET_M = 1.8
# 只看 0.3 秒：這道限制要的是「停在外側」，前瞻太長會停太早又回到路上。
# 倒車 0.7 m/s × 0.3 s = 0.21 m，加上減速距離約 0.16 m，不會衝過頭。
TRACK_POCKET_LOOKAHEAD_S = 0.3

TRACK_SPIN_S = 25           # 轉一圈的時間上限（實測 0.8 rad/s，360° 約 8 秒）
# 兩步都試完之後，隔多久可以再試一輪。
# 為什麼要重試：目標可能悄悄繞到旁邊，轉一圈就看得到。
# 為什麼不是一直轉：那樣會變成原地打轉的機器人，而且目標真的走遠時沒有意義。
TRACK_RECOVER_RETRY_S = 20.0


def wrap_pi(a):
    """把角度收進 (-pi, pi]。算角度差一定要做，否則 179° 到 -179° 會被當成
    轉了 358° 而不是 2°。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def follow_cmd(dist, e_yaw, standoff, kp_lin, kp_yaw,
               max_fwd, max_back, max_yawrate, deadband):
    """跟隨的控制律（差速車）。回傳 (前進速度 v, 角速度 w)。

    抽成純函式是為了測得到 —— 正負號寫反在模擬裡要盯著畫面才看得出來，
    而且症狀（車往反方向開、原地打轉）很容易被當成別的問題。

    e_yaw：目標在車頭的左右幾弧度（左正右負，已經 wrap 到 ±pi）
    回傳的 v 可以是負的 = 倒車，而車頭還是對著目標（這就是「太近就退」）。
    """
    w = max(-max_yawrate, min(max_yawrate, kp_yaw * e_yaw))
    e_d = dist - standoff
    if abs(e_d) < deadband or abs(e_yaw) > TRACK_YAW_ONLY_RAD:
        return 0.0, w                       # 只轉頭不前進
    v = kp_lin * e_d * math.cos(e_yaw)
    lim = max_fwd if v > 0 else max_back
    return max(-lim, min(lim, v)), w


# =============================================================================
#  擋牆（2026-11-15）—— 下面三個函式照抄 drone_api_node.py 的
#  _box_entry_axis / _clamp_no_fly，只差在座標是 map (x, y) 而不是 NED (北, 東)。
#  判斷式本身和軸的順序無關，只要點、速度、方框用同一套軸就對。
# =============================================================================
def point_box_dist(p, b):
    """點到方框的距離（點在框裡回 0）。"""
    dx = max(b[0] - p[0], 0.0, p[0] - b[2])
    dy = max(b[1] - p[1], 0.0, p[1] - b[3])
    return math.hypot(dx, dy)


def inflate_boxes(boxes, margin):
    """方框各邊往外推 margin。之後就可以把車當成一個點來算。"""
    return [(b[0] - margin, b[1] - margin, b[2] + margin, b[3] + margin)
            for b in boxes]


def box_entry_axis(p, v, box, horizon):
    """掃掠線段 p → p + v·horizon 會不會進到 box 裡？會的話回「先碰到的那一軸」。

    照抄 drone_api_node.py:_box_entry_axis（標準的 AABB slab test）。
    每一軸算出「位於該軸區間內」的時間區間，交集還在就表示會穿進去，
    而 near 最大的那一軸就是會先碰上的那一面 —— 砍掉那一軸的速度，
    平行那一面的分量就留下來了。

    ⚠️ 測的是「整條線段」不是「終點」，速度快時才不會穿過薄的方框。
    ⚠️ p 已經在方框裡的情況這裡不處理，由 clamp_to_boxes 先判斷。
    """
    near, far, axis = -math.inf, math.inf, None
    for i in (0, 1):
        lo, hi = box[i], box[i + 2]
        if abs(v[i]) < 1e-9:
            if p[i] < lo or p[i] > hi:
                return None
            continue
        a = (lo - p[i]) / v[i]
        b = (hi - p[i]) / v[i]
        if a > b:
            a, b = b, a
        if a > near:
            near, axis = a, i
        far = min(far, b)
    if axis is None or near > far or near >= horizon or far <= 0.0:
        return None
    return axis


def clamp_to_boxes(p, v, boxes, horizon):
    """把速度夾成「照這個速度走 horizon 秒也進不了任何方框」。

    照抄 drone_api_node.py:_clamp_no_fly 的迴圈。回傳 ([vx, vy], 擋住的方框索引 or None)。
    只砍法線方向，不砍整個速度 —— 這樣才會貼著牆面滑行，而不是一靠近就停住。
    """
    v = [float(v[0]), float(v[1])]
    blocked = None
    # 最多三輪：一輪最多歸零一個軸，兩軸都歸零就進不了任何方框，一定收斂
    for _ in range(3):
        hit = None
        for k, box in enumerate(boxes):
            if all(box[i] <= p[i] <= box[i + 2] for i in (0, 1)):
                # 已經在**加大後**的方框裡（餘裕被吃掉了，實體還有一段距離）。
                # 不能照常夾，否則連「往外走」都會被砍，車就永遠出不來。
                for i in (0, 1):
                    out = (box[i] - p[i]
                           if (p[i] - box[i]) < (box[i + 2] - p[i])
                           else box[i + 2] - p[i])
                    if v[i] * out < 0.0:
                        v[i] = 0.0
                        hit = k
                continue
            axis = box_entry_axis(p, v, box, horizon)
            if axis is not None:
                v[axis] = 0.0
                hit = k
                break
        if hit is None:
            break
        blocked = hit
    return v, blocked


def clamp_to_area(p, v, area, horizon):
    """不准離開 area 這個矩形：只砍「往外而且 horizon 秒後會出界」的那一軸。

    已經在外面時，往裡走照樣放行（例如車剛從生成點開過來）。
    回傳 ([vx, vy], 有沒有砍)。area 是 None 就不限制。
    """
    v = [float(v[0]), float(v[1])]
    if area is None:
        return v, False
    cut = False
    for i in (0, 1):
        q = p[i] + v[i] * horizon
        if (v[i] > 0.0 and q > area[i + 2]) or (v[i] < 0.0 and q < area[i]):
            v[i] = 0.0
            cut = True
    return v, cut


def clamp_motion(p, v, walls, area, wall_h, area_h):
    """先夾範圍再夾牆。回傳 ([vx, vy], 'area' / 'wall' / None)。

    兩個都砍到時回 'area' —— 那是「退到路口外側」的情況，車尾同時靠近
    北/南牆是正常的，要顯示給使用者的是前者。
    """
    v, cut = clamp_to_area(p, v, area, area_h)
    v, k = clamp_to_boxes(p, v, walls, wall_h)
    if cut:
        return v, "area"
    return v, ("wall" if k is not None else None)


def follow_world(p, yaw, tgt, standoff, kp_lin, kp_yaw, max_fwd, max_back,
                 max_yawrate, deadband, walls=(), area=None,
                 wall_margin=TRACK_WALL_MARGIN_M, rot_clear=TRACK_ROT_CLEAR_M,
                 wall_h=TRACK_WALL_LOOKAHEAD_S, area_h=TRACK_POCKET_LOOKAHEAD_S):
    """跟隨控制律 ＋ 擋牆（差速車）。回傳 (v, w, why)。

    why：None / 'wall'（沿牆或被牆擋）/ 'area'（退到路口外側）/ 'close'（太靠近，先移開）
    walls 是**原始**方框（貨架與牆的實際邊界），加大由這裡做。

    依車中心離貨架/牆的距離分三層（2026-11-15 模擬實測後補的，見 TRACK_ROT_CLEAR_M）：
        > wall_margin（0.8）   沿牆滑行，車頭方向可以帶一點往裡的分量（見最後的檢查）
        rot_clear ~ wall_margin 可以轉頭；移動**不准有任何往裡的分量**
        < rot_clear（0.65）    **不准轉頭**（車尾會掃到），先沿車身軸直直移開

    🔑 什麼都沒擋到時，答案和 follow_cmd **完全一樣**（測試⑰釘住這件事）。
       擋牆只在快撞的時候介入，不改變平常跟隨的手感 —— 下午調好的
       kp 2.0、倒車 0.7 都不受影響。

    三步：
      ① 想往哪走：沿「車 → 目標」連線，大小用 follow_cmd 的距離律
         （太遠往目標走，太近往反方向退）
      ② 用無人機同一套判斷式夾這個世界座標速度：只砍朝牆那一面 → 沿牆滑
      ③ 換成差速車的 v、ω：
           往前：車頭轉向夾完的方向，v = 速率 × cos(車頭誤差)
           倒車：車頭**照樣對著目標**（相機要看得到），v = 夾完的速度在車身軸上的投影
      最後再用**實際的車頭方向**檢查一次（車頭可能還沒轉過來）：
      被砍掉超過一半就不走、只轉頭。
    """
    # ⓪ 太靠近：不准轉頭。⚠️ 必須在最前面 —— 「已就位」那條路徑 v=0 但 w≠0，
    #    2026-11-15 掃到貨架角的那一下就是從那裡轉的，放在後面會被它先 return 掉。
    if walls and rot_clear > 0.0:
        d0 = min(point_box_dist(p, b) for b in walls)
        if d0 < rot_clear:
            hd = (math.cos(yaw), math.sin(yaw))
            step = 0.1
            df = min(point_box_dist((p[0] + step * hd[0], p[1] + step * hd[1]), b)
                     for b in walls)
            db = min(point_box_dist((p[0] - step * hd[0], p[1] - step * hd[1]), b)
                     for b in walls)
            if max(df, db) <= d0 + 1e-6:
                return 0.0, 0.0, "close"     # 前後都不會更遠（不該發生）：不動
            v = TRACK_ESCAPE_SPEED if df > db else -TRACK_ESCAPE_SPEED
            return v, 0.0, "close"

    walls = inflate_boxes(walls, wall_margin)
    dx, dy = tgt[0] - p[0], tgt[1] - p[1]
    dist = math.hypot(dx, dy)
    e_t = wrap_pi(math.atan2(dy, dx) - yaw)
    v, w = follow_cmd(dist, e_t, standoff, kp_lin, kp_yaw,
                      max_fwd, max_back, max_yawrate, deadband)
    e_d = dist - standoff
    if abs(e_d) < deadband or dist < 1e-6:
        return v, w, None                    # 只轉頭（上面已確定轉頭不會掃到）

    # ① ②
    s = max(-max_back, min(max_fwd, kp_lin * e_d))
    want = (s * dx / dist, s * dy / dist)
    got, why = clamp_motion(p, want, walls, area, wall_h, area_h)

    # ③
    if why is not None:
        if s > 0.0:
            spd = math.hypot(got[0], got[1])
            if spd < 1e-3:
                v = 0.0                      # 整個被擋：原地等，車頭繼續對著目標
            else:
                e = wrap_pi(math.atan2(got[1], got[0]) - yaw)
                w = max(-max_yawrate, min(max_yawrate, kp_yaw * e))
                v = spd * math.cos(e) if abs(e) <= TRACK_YAW_ONLY_RAD else 0.0
        else:
            v = got[0] * math.cos(yaw) + got[1] * math.sin(yaw)
            v = max(-max_back, min(0.0, v))
            if abs(e_t) > TRACK_YAW_ONLY_RAD:
                v = 0.0

    # 實際車頭方向的最後檢查
    if v != 0.0:
        m = (v * math.cos(yaw), v * math.sin(yaw))
        m2, why2 = clamp_motion(p, m, walls, area, wall_h, area_h)
        if why2 is not None:
            frac = (m2[0] * m[0] + m2[1] * m[1]) / (v * v)
            # ⚠️ 已經在加大後的方框裡就**不放寬**：餘裕已經被吃掉了，
            #    再帶一點往裡的分量就會一路擠向實體（2026-11-15 實測：倒車時
            #    靠這個放寬斜斜退進去，中心離貨架角從 0.78 m 退到 0.52 m）。
            inside = any(all(b[i] <= p[i] <= b[i + 2] for i in (0, 1)) for b in walls)
            need = TRACK_WALL_INSIDE_FRAC if inside else TRACK_WALL_MIN_FRAC
            v = v * frac if frac >= need else 0.0
            why = why or why2
    return v, w, why


def _seg_dist(p, a, b):
    """點 p 到線段 ab 的距離，以及最近點在線段上的參數 t（0=a、1=b）。"""
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:                       # 退化成一個點
        return math.hypot(p[0] - a[0], p[1] - a[1]), 0.0
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy)), t


def nearest_leg(here, pts):
    """here 離哪一段環線最近 → 回傳**那一段的索引**（第 i 段 = pts[i] → pts[i+1]）。

    ⚠️ 看**線段**不是看航點。作法照抄 drone_nodered/scripts/move_target.py
       的同名函式（2026-10-15 在那邊改過一次，理由一樣）：
       只挑「最近的航點」的話，點已經在某條邊上時會往回倒退去踩那條邊的
       起點，看起來像「出場先走錯方向」。正確的行為是直接朝那條邊的終點走。
    """
    best_i, best_d = 0, float("inf")
    for k in range(len(pts)):
        d, _t = _seg_dist(here, pts[k], pts[(k + 1) % len(pts)])
        if d < best_d:
            best_d, best_i = d, k
    return best_i


def next_wp(leg, step, n):
    """走第 leg 段時，下一個該去的航點索引。

    step=+1 照列表順序走 → 下一個是這段的**終點** pts[leg+1]
    step=-1 反著走       → 下一個是這段的**起點** pts[leg]
    """
    return (leg + 1) % n if step > 0 else leg % n


class HuskyApi(Node):
    def __init__(self):
        super().__init__("husky_api_node")
        self.declare_parameter("namespace", "husky")
        self.declare_parameter("goto_timeout_s", 180.0)
        self.declare_parameter("geometry_file", "")
        # Nav2 被推進 /husky 命名空間（兩台同場時節點會撞名），
        # 所以 action 不是 /navigate_to_pose。單獨跑時可以改回來。
        self.declare_parameter("nav_action", "/husky/navigate_to_pose")
        # --- 追蹤的可調參數 ---------------------------------------------------
        self.declare_parameter("track_standoff_m", TRACK_STANDOFF_M)
        self.declare_parameter("track_duration_s", TRACK_DURATION_S)
        # 目標位置多新才拿來算站位。相機約 15 Hz，1.5 秒 ≒ 連續 20 幀都沒看到。
        self.declare_parameter("track_fresh_s", TRACK_FRESH_S)
        # ⚠️ 和上面是**兩個不同的門檻**，不要合成一個。
        #    1.5~5 秒之間是「原地等」，超過 5 秒才去跑搜尋環線。
        #    理由：目標靠太近（<2 m）時偵測率會掉，但它其實就在眼前。
        #    只有一個門檻的話，車會在目標就在正前方的時候掉頭跑去搜尋。
        self.declare_parameter("track_search_after_s", TRACK_SEARCH_AFTER_S)
        self.declare_parameter("track_goal_interval_s", TRACK_GOAL_INTERVAL_S)
        self.declare_parameter("track_goal_move_m", TRACK_GOAL_MOVE_M)
        self.declare_parameter("track_arrive_tol_m", TRACK_ARRIVE_TOL_M)
        self.declare_parameter("track_min_m", TRACK_MIN_M)
        self.declare_parameter("track_close_m", TRACK_CLOSE_M)
        self.declare_parameter("track_backup_speed", TRACK_BACKUP_SPEED)
        self.declare_parameter("track_kp_lin", TRACK_KP_LIN)
        self.declare_parameter("track_kp_yaw", TRACK_KP_YAW)
        self.declare_parameter("track_max_speed", TRACK_MAX_SPEED)
        self.declare_parameter("track_max_yawrate", TRACK_MAX_YAWRATE)
        self.declare_parameter("track_speed_deadband_m", TRACK_SPEED_DEADBAND_M)
        # 擋牆與倒車讓路（2026-11-15）。理由見常數那邊。
        self.declare_parameter("track_wall_margin_m", TRACK_WALL_MARGIN_M)
        self.declare_parameter("track_wall_lookahead_s", TRACK_WALL_LOOKAHEAD_S)
        self.declare_parameter("track_pocket_m", TRACK_POCKET_M)
        # Nav2 的復原行為。和 nav_action 一樣，被推進 /husky 命名空間了。
        self.declare_parameter("spin_action", "/husky/spin")
        self.declare_parameter("backup_action", "/husky/backup")
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

        # --- 追蹤的狀態 -------------------------------------------------------
        # phase：'' = 沒在追；'搜尋' = 看不到目標在跑環線；'跟隨' = 看得到
        # nav_busy：有沒有一個追蹤的子目標還在 Nav2 手上。
        #   ⚠️ 需要這個旗標的理由：追蹤是「一個任務、很多個 Nav2 目標」。
        #      Nav2 回報某個子目標完成時**不能**把整個追蹤任務判成結束，
        #      要讓它繼續下一個。_on_result 會看這個旗標決定怎麼走。
        self._track = {
            "active": False, "phase": "", "deadline": 0.0, "duration": 0.0,
            "standoff": TRACK_STANDOFF_M,
            "search": [], "search_i": 0,
            "target": None, "target_t": None,   # (x, y) map 與收到的 monotonic
            "goal": None, "goal_t": 0.0,        # 最後送給 Nav2 的搜尋航點（2026-11-15 起跟隨不送 Nav2）
            "nav_busy": False, "nav_fail": 0,
            "message": "",
            # 近距離重找：'' = 沒在做；'轉圈' / '倒車' = 正在做那一步。
            # tried 記這一輪跟丟已經試過哪些步驟，才不會原地轉到天荒地老。
            # 這一輪追蹤有沒有已經接上環線（接的那一次要挑起點，之後照順序走）
            "search_started": False,
            # 環線的繞行方向。+1 = 照列表順序、-1 = 反向。
            #
            # 🔑 這是整個「從後面追」的關鍵（2026-11-15，作法抄無人機的
            #    drone_api_node.py:1465-1477，2026-10-10 Oliver 提的）：
            #
            #    搜尋時刻意和目標**反向**繞 → 每一段都是對頭相遇，最快碰到
            #    （環線 47 m，反向最久 19 秒；同向要 31 秒）。
            #    但對頭的問題是「碰到之後怎麼辦」—— 原地轉 180° 會變成
            #    車在目標前面、而且目標還朝車尾開過來，會撞。
            #
            #    正確的做法（Oliver 2026-11-15 指出）：
            #      ① 對頭相遇後，車**保持車頭對著目標、一路倒車**退
            #         （速度控制倒車時車頭照樣對著目標 —— 見 follow_world）
            #      ② 退過環線的**路口**、停在路口外側讓路，目標在路口轉彎、
            #         從車前面過去（見 TRACK_POCKET_M）
            #      ③ 目標轉過路口的那一刻才翻成同向 —— 這時它已經在遠離了，
            #         追上去自然就在它後面
            #
            #    ⚠️ 只翻一次。翻完兩者同向，之後再跟丟（例如目標又轉過一個
            #       彎角）繼續同方向走就會追上；再翻一次反而變回對頭。
            # 跟隨時是不是已經接管速度（接管前要先把 Nav2 的目標取消掉，
            # 不然兩個來源會同時寫 /husky/cmd_vel）
            "vel_mode": False,
            "loop_step": 1, "loop_flipped": False,
            "wall_why": None,   # 擋牆目前是誰在擋（只在改變時記 log）
            # 目標上次落在環線的第幾段。段號變了 = 它轉過路口了。
            "target_leg": None,
            "recover": "", "recover_tried": set(), "recover_t": 0.0,
            # True = 這個動作是「跟丟了在重找」，看到目標就該取消。
            # False = 是「跟隨時太近在退」，看得到也要讓它退完，
            #         不然每一拍都會被取消，等於永遠退不了（2026-11-14 踩到）。
            "recover_from_lost": False,
            "last_dist": None,      # 最後一次「看得到」時離目標多遠
        }
        self._beh_handle = None     # 進行中的 Spin / BackUp

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
        self.spin_client = ActionClient(
            self, Spin, self.get_parameter("spin_action").value)
        self.backup_client = ActionClient(
            self, BackUp, self.get_parameter("backup_action").value)

        # 目標的 map 座標。由 drone_vision 的 point_to_frame_node 發
        # （相機 → 偵測 → 深度 → TF → map）。沒有視覺鏈時這個話題就不會有東西，
        # 追蹤會一直停在「搜尋」—— 那是正確行為，不是壞掉。
        self.create_subscription(PointStamped, f"/{ns}/api/target",
                                 self._on_target, 10)

        self.create_service(GoTo, f"/{ns}/api/goto", self.srv_goto)
        self.create_service(Track, f"/{ns}/api/track", self.srv_track)
        self.create_service(Trigger, f"/{ns}/api/stop", self.srv_stop)
        self.create_service(Trigger, f"/{ns}/api/state", self.srv_state)

        # 參數關係檢查：close_m 太小的話整段近距離重找是死的（見常數那邊的說明）。
        # 這種錯不會有任何症狀 —— 只會「重找好像從來沒作用過」，很難查。
        hold = max(float(self.get_parameter("track_min_m").value),
                   float(self.get_parameter("track_standoff_m").value)
                   + float(self.get_parameter("track_arrive_tol_m").value))
        if float(self.get_parameter("track_close_m").value) <= hold:
            self.get_logger().warn(
                f"track_close_m（{self.get_parameter('track_close_m').value}）"
                f"不大於車實際維持的距離 {hold:.1f} m —— "
                "近距離跟丟時的「轉圈／倒車重找」永遠不會觸發")
        # 倒車讓路要退得夠遠，目標轉彎時才不會穿過車：
        #   車頭（中心 + 半長 0.5）要超過目標轉彎時伸出去的 0.3 m，再留 0.4 m。
        if float(self.get_parameter("track_pocket_m").value) < 1.2:
            self.get_logger().warn(
                f"track_pocket_m（{self.get_parameter('track_pocket_m').value}）"
                "小於 1.2 m —— 對頭相遇倒車讓路時，車頭還在目標轉彎的路上，"
                "目標會直接穿過車")

        self.create_timer(1.0 / STATUS_HZ, self._tick)
        # 跟隨階段的速度控制。和狀態迴圈分開，因為兩者的節拍需求不同：
        # 狀態 5 Hz 夠用，速度指令要 10 Hz 才順。
        self.create_timer(1.0 / TRACK_CMD_HZ, self._follow_cmd_tick)
        self.get_logger().info(
            f"husky_api_node 起來了：/{ns}/api/{{goto,track,stop,state}}，"
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
        self._track_tick()
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
        # 追蹤的狀態。儀表板和積木靠 phase 看現在是在搜尋還是跟隨。
        # target 給介面畫「車認為目標在哪」用（和無人機的紅點同一個用途）。
        t = self._track
        d["track"] = {
            "active": t["active"], "phase": t["phase"],
            "standoff": round(t["standoff"], 2),
            "target": [round(t["target"][0], 2), round(t["target"][1], 2)]
                      if t["target"] else None,
            # 目標位置有多舊（秒）。None = 從來沒看到過。
            "target_age": (round(time.monotonic() - t["target_t"], 1)
                           if t["target_t"] else None),
            "goal": [round(t["goal"][0], 2), round(t["goal"][1], 2)]
                    if t["goal"] else None,
            # 正在做哪個復原動作（'' / 轉圈 / 倒車）。儀表板可以顯示。
            "recover": t["recover"],
            # 已經追了幾秒／總共要追幾秒（2026-11-15，儀表板顯示「42 / 180 秒」用）。
            # 從 deadline 倒推，不另外記開始時間 —— 兩個來源會不同步。
            "elapsed_s": (round(max(0.0, time.monotonic()
                                    - (t["deadline"] - t["duration"])))
                          if t["active"] else None),
            "duration_s": round(t["duration"]) if t["active"] else None,
            # 環線繞行方向：'反向'=和目標對頭（找得快）、'同向'=從後面追
            "loop": "同向" if t["loop_step"] < 0 else "反向",
            # True = 跟隨中、由速度控制在開；False = Nav2 在開（搜尋）
            "vel": t["vel_mode"],
            "message": t["message"],
        }
        d["scene"] = {"boxes": self._scene["boxes"]}
        if self._scene["home"] is not None:
            # 「返航」積木讀這個。拿不到就走失敗出口，不要亂猜一個 (0,0)。
            d["scene"]["home"] = list(self._scene["home"])
        # 「最近訊息」：儀表板會顯示這一行。優先顯示任務的訊息，
        # 沒有就退回導航的。無人機那邊也有同名欄位，儀表板共用同一段程式。
        # 優先顯示任務訊息；任務還沒開始（例如服務當場被拒絕）就顯示那個回覆。
        # ⚠️ 追蹤時優先顯示追蹤的階段訊息。
        #    不這樣的話整趟 60 秒那一欄都凍在「開始追蹤」，看起來像卡住了
        #    —— 無人機那邊踩過同一個坑（2026-10-12）。
        d["last_message"] = ((t["message"] if t["active"] else "")
                             or self._task.get("message")
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
            self._release_velocity()
            self._track.update(active=False, phase="", nav_busy=False)
            self._cancel_behavior()
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
            self._release_velocity()
            self._track.update(active=False, phase="", nav_busy=False,
                               message="看門狗：位置估計中斷")
            self._cancel_behavior()
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
        # 手動給目標 = 放棄追蹤。兩個都在開 Nav2 目標，同時跑會互相搶。
        self._track_cancel("被「導航到座標」接手")
        gx, gy = float(request.map_x), float(request.map_y)
        timeout = float(request.timeout) or self.goto_timeout
        self._begin_task(name, timeout)
        self._send_nav(gx, gy, float(request.yaw))
        return self._reply(response, True,
                           f"已送出導航目標 map=({gx:+.2f}, {gy:+.2f})，"
                           f"逾時 {timeout:.0f} 秒")

    def _send_nav(self, gx, gy, yaw):
        """送一個 Nav2 目標。**只碰導航狀態，不碰任務狀態。**

        為什麼要分開（2026-11-14）：追蹤是「一個任務、很多個 Nav2 目標」。
        如果送目標時順手開一個新任務，積木的 task.seq 會在追蹤途中一直跳，
        「等待完成」會以為上一個動作做完了 —— 整個協定就壞了。
        所以開任務是呼叫端的事，這裡只負責把目標丟出去。
        """
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(gx)
        goal.pose.pose.position.y = float(gy)
        if abs(yaw) > math.pi:
            # 「不指定終點朝向」。四元數還是要給合法值（w=1），
            # 給 NaN 會讓 Nav2 整串 TF 變 NaN。反正 yaw_goal_tolerance 是 3.14。
            goal.pose.pose.orientation.w = 1.0
        else:
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self._nav_seq += 1
        seq = self._nav_seq
        self._nav = {"state": "running", "goal": [round(gx, 3), round(gy, 3)],
                     "seq": seq, "distance_remaining": None, "message": ""}
        fut = self.nav_client.send_goal_async(goal, feedback_callback=self._on_fb)
        fut.add_done_callback(lambda f, q=seq: self._on_goal_response(f, q))
        return seq

    # ---------- 追蹤（2026-11-14）--------------------------------------------
    def _on_target(self, msg):
        """收到目標的 map 座標。

        ⚠️ 這個點有一個**固定的系統偏移**：深度量到的是目標的表面（車尾），
           不是車體中心。倉庫實測是偏近 0.28 m（README 的 H7 節有數據）。
           這裡刻意**不補**那個偏移 —— 它和目標長什麼樣、從哪個角度看都有關，
           補成固定值換個目標就錯。站位距離本來就有 ±0.5 m 的容許，吸收得掉。
        """
        if msg.header.frame_id not in ("map", ""):
            # 只收 map 的點。收到別的座標系就是上游接錯了，要講出來而不是默默算錯。
            self._track["message"] = f"目標的座標系是 {msg.header.frame_id}，不是 map"
            return
        self._track["target"] = (msg.point.x, msg.point.y)
        self._track["target_t"] = time.monotonic()

    def srv_track(self, request, response):
        name = "追蹤目標"
        r = self._ready()
        if not r["odom"]:
            return self._reply(response, False, "沒有位置估計（收不到 odom）")
        if not r["nav2"]:
            return self._reply(response, False,
                               f"Nav2 沒在跑（{self.get_parameter('nav_action').value} 不存在）")
        if not r["bridge"]:
            return self._reply(response, False,
                               "速度看門狗沒在跑 —— 沒有它 gz 不會自己停車，拒絕出車")

        standoff = float(request.distance) or float(
            self.get_parameter("track_standoff_m").value)
        duration = float(request.duration) or float(
            self.get_parameter("track_duration_s").value)
        sx = list(request.search_x)
        sy = list(request.search_y)
        if len(sx) != len(sy):
            return self._reply(response, False,
                               f"搜尋環線的 x 和 y 長度不一樣（{len(sx)} vs {len(sy)}）")
        # ⚠️ altitude 欄位對地面車沒有意義（Track.srv 是和無人機共用的格式）。
        #    收到非零值時講一聲，不要默默忽略 —— 使用者可能以為自己設定了什麼。
        if float(request.altitude):
            self.get_logger().info(
                f"忽略 altitude={request.altitude}：地面車沒有高度這個概念")

        self._track_cancel()             # 重複呼叫就是重新開始，不是疊加
        self._track.update(
            active=True, phase="搜尋", deadline=time.monotonic() + duration,
            duration=duration,
            standoff=standoff, search=list(zip(sx, sy)), search_i=0,
            search_started=False,
            goal=None, goal_t=0.0, nav_busy=False, nav_fail=0,
            recover="", recover_tried=set(), recover_t=0.0,
            recover_from_lost=False, last_dist=None,
            vel_mode=False, loop_step=1, loop_flipped=False, target_leg=None, wall_why=None,
            message="開始追蹤")
        # 任務的逾時給寬一點：正常結束是由 _track_tick 在 deadline 時判成功，
        # _check_task 這個保險只在追蹤邏輯本身卡住時才該觸發。
        self._begin_task(name, duration + 10.0)
        where = (f"，看不到時巡 {len(sx)} 個搜尋點"
                 if sx else "，沒有給搜尋環線（看不到就原地等）")
        return self._reply(response, True,
                           f"開始追蹤：站位 {standoff:.1f} m、{duration:.0f} 秒{where}")

    def _track_cancel(self, why=""):
        """停止追蹤。why 有值時會寫進任務訊息（被別的指令接手）。"""
        if not self._track["active"]:
            return
        self._release_velocity()
        self._track.update(active=False, phase="", nav_busy=False,
                           message=why or "已停止追蹤")
        self._cancel_behavior()
        self._cancel_nav()
        if why:
            self._end_task(False, why)

    def _cancel_behavior(self):
        h = self._beh_handle
        self._beh_handle = None
        self._track["recover"] = ""
        self._track["recover_from_lost"] = False
        if h is not None:
            try:
                h.cancel_goal_async()
            except Exception:                                    # noqa: BLE001
                pass

    def _send_behavior(self, kind, dist=0.0, from_lost=False):
        """送一個 Nav2 的復原行為（原地轉 / 倒車）。

        ⚠️ 一定要先取消導航目標：行為伺服器和控制器都會發 cmd_vel，
           兩個同時跑會互相打架。
        """
        self._cancel_nav()
        self._track["nav_busy"] = False
        if kind == "轉圈":
            client, goal = self.spin_client, Spin.Goal()
            goal.target_yaw = 2.0 * math.pi          # 轉一整圈
            goal.time_allowance = Duration(sec=TRACK_SPIN_S)
        else:
            client, goal = self.backup_client, BackUp.Goal()
            # ⚠️ target.x 給**正值**就是往後退（2026-11-14 實測：+1.0 → 沿車頭
            #    方向 -1.027 m，車頭完全沒轉）。不要自作聰明給負值。
            goal.target = Point(x=float(dist), y=0.0, z=0.0)
            goal.speed = float(self.get_parameter("track_backup_speed").value)
            goal.time_allowance = Duration(sec=int(dist / max(goal.speed, 0.05)) + 15)
        if not client.server_is_ready():
            self._track["message"] = f"{kind}：行為伺服器不在（Nav2 的 behavior_server）"
            self._track["recover_tried"].add(kind)
            return False
        self._track["recover"] = kind
        self._track["recover_from_lost"] = from_lost
        fut = client.send_goal_async(goal)
        fut.add_done_callback(self._on_behavior_response)
        return True

    def _on_behavior_response(self, fut):
        try:
            h = fut.result()
        except Exception as e:                                   # noqa: BLE001
            self._track["message"] = f"復原行為送出失敗：{e}"
            self._on_behavior_done()
            return
        if not h.accepted:
            self._track["message"] = f"{self._track['recover']}：被拒絕"
            self._on_behavior_done()
            return
        self._beh_handle = h
        h.get_result_async().add_done_callback(lambda _f: self._on_behavior_done())

    def _on_behavior_done(self):
        """一步做完了（成功或失敗都一樣）—— 記起來，讓下一拍換下一步。"""
        kind = self._track["recover"]
        self._beh_handle = None
        self._track["recover"] = ""
        if kind:
            self._track["recover_tried"].add(kind)

    def _standoff_goal(self, tx, ty, hx, hy, standoff):
        """算站位點：目標與車連線上、離目標 standoff 的那個點，車頭朝向目標。"""
        dx, dy = hx - tx, hy - ty
        d = math.hypot(dx, dy)
        if d < 1e-3:                      # 幾乎重疊，方向沒有意義
            return None, None, None, d
        gx, gy = tx + dx / d * standoff, ty + dy / d * standoff
        return gx, gy, math.atan2(ty - gy, tx - gx), d

    def _track_tick(self):
        """每次發狀態時推進追蹤。整個追蹤邏輯只有這一個入口。"""
        t = self._track
        if not t["active"]:
            return
        now = time.monotonic()
        if now >= t["deadline"]:
            self._release_velocity()
            t.update(active=False, phase="", nav_busy=False, message="追蹤時間到")
            self._cancel_behavior()
            self._cancel_nav()
            self._stop_wheels()
            self._end_task(True, "追蹤時間到")
            return
        if t["nav_fail"] >= TRACK_MAX_NAV_FAIL:
            self._release_velocity()
            t.update(active=False, phase="", nav_busy=False,
                     message=f"連續 {TRACK_MAX_NAV_FAIL} 次導航失敗")
            self._cancel_behavior()
            self._cancel_nav()
            self._stop_wheels()
            self._end_task(False, f"追蹤失敗：連續 {TRACK_MAX_NAV_FAIL} 次導航失敗")
            return

        age = (now - t["target_t"]) if t["target_t"] else 1e9
        fresh = float(self.get_parameter("track_fresh_s").value)
        search_after = float(self.get_parameter("track_search_after_s").value)

        if age <= fresh:
            # 看得到了 —— 重新找目標的動作可以停了（例如轉到一半就看到）。
            # ⚠️ 但「跟隨時太近在倒車」不能停：它是看得到的情況下才發的，
            #    這裡一併取消的話每一拍都會被取消，等於永遠退不了。
            if t["recover"] and t["recover_from_lost"]:
                self._cancel_behavior()
            t["recover_tried"] = set()
            t["phase"] = "跟隨"
            self._track_follow(now)
        elif t["recover"]:
            # 復原動作進行中。它可能比 search_after 還久（轉一圈約 8 秒），
            # 所以要在這裡攔住，不然會被下面的分支切去搜尋。
            t["phase"] = "重找"
            t["message"] = f"{t['recover']}重找目標…"
        elif age <= search_after:
            # 剛剛還看得到 —— 原地等，不要急著掉頭去搜尋。
            # 這一段存在的理由見 track_search_after_s 的參數說明。
            t["phase"] = "跟隨"
            t["message"] = f"目標暫時看不到（{age:.1f} 秒）—— 原地等"
        elif self._track_recover():
            t["phase"] = "重找"
        else:
            t["phase"] = "搜尋"
            self._track_search(now)

    def _track_recover(self):
        """近距離跟丟的重找。回傳 True = 這一拍由重找接管。

        只有「最後看到時很近」才做。遠距離跟丟（目標繞到貨架後面）做這些
        沒有意義，直接跑搜尋環線比較快。

        兩步，各自解決不同的幾何：
            轉圈  目標從旁邊開走或開到車後面 —— 前視相機永遠看不到那裡
            倒車  目標停在正前方 1 m 不動 —— 每個角度都太近，轉圈沒用
        先轉再退，因為原地轉比盲退安全（Husky 沒有後方感測器）。
        """
        t = self._track
        if t["last_dist"] is None or t["last_dist"] >= float(
                self.get_parameter("track_close_m").value):
            return False
        now = time.monotonic()
        # 兩步都試完之後隔一段時間再來一輪 —— 目標可能悄悄繞到旁邊。
        if t["recover_tried"] and now - t["recover_t"] > TRACK_RECOVER_RETRY_S:
            t["recover_tried"] = set()
        if not t["recover_tried"]:
            t["recover_t"] = now
        self._release_velocity()       # 重找要發 Spin/BackUp，速度控制先放手
        if "轉圈" not in t["recover_tried"]:
            t["message"] = "近距離跟丟 —— 原地轉一圈找"
            return self._send_behavior("轉圈", from_lost=True)
        if "倒車" not in t["recover_tried"]:
            back = min(max(t["standoff"] - t["last_dist"], 0.5), TRACK_BACKUP_MAX_M)
            t["message"] = f"轉一圈還是沒看到 —— 後退 {back:.1f} m 再找"
            return self._send_behavior("倒車", back, from_lost=True)
        return False            # 兩步都試過了 → 交給搜尋環線

    def _check_corner(self, tx, ty):
        """目標轉過環線路口了嗎？是的話把繞行方向翻成同向（只翻一次）。

        為什麼用「第幾段」而不是估目標的速度方向：段號只要比對一個整數，
        不受偵測雜訊影響；而速度方向要靠連續位置去估，會被「深度量到的是
        車尾表面、偏移方向隨視角轉」那個假動作騙（README 的 H7 有數據）。
        """
        t = self._track
        if t["loop_flipped"] or len(t["search"]) < 2:
            return
        leg = nearest_leg((tx, ty), t["search"])
        if t["target_leg"] is None:
            t["target_leg"] = leg
            return
        if leg == t["target_leg"]:
            return
        # 段號變了 = 目標轉過路口，現在正在遠離 → 這一刻才翻方向
        t["target_leg"] = leg
        t["loop_step"] = -t["loop_step"]
        t["loop_flipped"] = True
        # 翻了之後，下一次要上環線時起點要重算（方向變了，接法也不同）
        t["search_started"] = False
        self.get_logger().info(
            f"[追蹤] 目標轉過路口（環線第 {leg + 1} 段）—— 繞行方向翻成同向，"
            "之後從後面追")

    def _track_follow(self, now):
        """跟隨：接管速度控制。

        這裡只負責「切到速度模式 + 更新訊息」，真正每 10 Hz 送 cmd_vel 的是
        _follow_cmd_tick。分開的理由見 TRACK_CMD_HZ 那段。
        """
        t = self._track
        with self._lock:
            o = self._odom
        if not o:
            return
        tx, ty = t["target"]
        dist = math.hypot(tx - o[0], ty - o[1])
        t["last_dist"] = dist          # 跟丟時要靠它決定做不做近距離重找
        self._check_corner(tx, ty)

        if not t["vel_mode"]:
            # 接管之前先把別人放掉 —— Nav2 的控制器和行為伺服器都會寫
            # /husky/cmd_vel，三個來源同時寫會互相蓋掉。
            self._cancel_behavior()
            self._cancel_nav()
            t["nav_busy"] = False
            t["vel_mode"] = True
            self.get_logger().info("[追蹤] 鎖定目標，改用速度控制跟隨")

    def _release_velocity(self, why=""):
        """放掉速度控制，把輪子停住，讓 Nav2 可以接手。"""
        t = self._track
        if not t["vel_mode"]:
            return
        t["vel_mode"] = False
        self._stop_wheels()
        if why:
            t["message"] = why

    def _follow_cmd_tick(self):
        """跟隨階段每 10 Hz 送一次速度。只有 vel_mode 時才動。

        控制律（差速車版）：
            方位誤差 → 角速度 ω
            距離誤差 → 前進速度 v（太近就是負的，直接倒車，車頭還是對著目標）
            方位誤差大的時候把 v 壓到 0，先轉頭再走 —— 不然會畫一個大弧。

        ⚠️ 這裡取代了原本「太近就發 BackUp 行為」那段：速度控制本來就能給
           負的 v，而且是連續的，不會有「退一段、停、再退一段」的空檔。
           BackUp 只留給「跟丟之後的重找」。
        """
        t = self._track
        if not (t["active"] and t["vel_mode"]):
            return
        if t["target"] is None or t["target_t"] is None:
            return
        # ⚠️ 目標暫時看不到時**不要直接停死**（2026-11-15 實測修）。
        #    偵測每閃一下（1~2 秒）車就完全停住、看到了再衝到滿速 ——
        #    Oliver 回報的「頓一下頓一下」就是這個。
        #    改成繼續朝「最後看到的位置」開：那個位置只有一兩秒舊，而且車在
        #    目標後面，朝它開是安全的；車愈靠近、距離誤差愈小，速度自然收斂到 0，
        #    不需要特別煞車。
        #    超過 search_after（預設 5 秒）就真的停 —— 那時 _track_tick 會接手
        #    去做重找或搜尋。
        age = time.monotonic() - t["target_t"]
        if age > float(self.get_parameter("track_search_after_s").value):
            self._stop_wheels()
            return
        with self._lock:
            o = self._odom
        if not o:
            return

        hx, hy, yaw = o[0], o[1], o[2]
        tx, ty = t["target"]
        dist = math.hypot(tx - hx, ty - hy)
        e_d = dist - t["standoff"]

        dead = float(self.get_parameter("track_speed_deadband_m").value)
        v, w, why = follow_world(
            (hx, hy), yaw, (tx, ty), t["standoff"],
            float(self.get_parameter("track_kp_lin").value),
            float(self.get_parameter("track_kp_yaw").value),
            float(self.get_parameter("track_max_speed").value),
            float(self.get_parameter("track_backup_speed").value),
            float(self.get_parameter("track_max_yawrate").value),
            dead, walls=self._scene.get("boxes") or [], area=self._pocket_area(),
            wall_margin=float(self.get_parameter("track_wall_margin_m").value),
            wall_h=float(self.get_parameter("track_wall_lookahead_s").value))

        if why == "close":
            t["message"] = (f"太靠近貨架或牆 —— 先直直移開，不轉頭"
                            f"（離目標 {dist:.1f} m）")
        # ⚠️ 用 < 0.01 不用 == 0：被夾到只剩 -0.003 這種值時，訊息會變成
        #    「沿著貨架或牆走（速度 -0.00 m/s）」，看的人會以為它在動。
        elif why == "area" and abs(v) < 0.01:
            t["message"] = f"退到路口外側了 —— 停著讓目標轉彎（離目標 {dist:.1f} m）"
        elif why == "wall" and abs(v) < 0.01:
            t["message"] = f"前面是貨架或牆 —— 先轉頭（離目標 {dist:.1f} m）"
        elif why == "wall":
            t["message"] = (f"沿著貨架或牆走（離目標 {dist:.1f} m，"
                            f"速度 {v:.2f} m/s）")
        elif abs(e_d) < dead:
            t["message"] = f"已就位（離目標 {dist:.1f} m）"
        elif v < 0:
            t["message"] = f"目標靠到 {dist:.1f} m —— 倒車保持距離"
        else:
            t["message"] = f"跟隨中：離目標 {dist:.1f} m（速度 {v:.2f} m/s）"

        if why != t.get("wall_why"):
            if why:
                self.get_logger().info(
                    f"[追蹤] 擋牆介入（{ {'area': '路口外側', 'close': '太靠近，先移開'}.get(why, '貨架或牆') }）："
                    f"車 ({hx:+.2f},{hy:+.2f}) v={v:+.2f} w={w:+.2f}")
            t["wall_why"] = why

        m = Twist()
        m.linear.x = v
        m.angular.z = w
        self.pub_cmd.publish(m)

    def _pocket_area(self):
        """倒車讓路的範圍：搜尋環線的外接矩形往外放寬 track_pocket_m。

        沒給環線時不限制（使用者自己決定要去哪）。
        """
        pts = self._track.get("search") or []
        if len(pts) < 2:
            return None
        m = float(self.get_parameter("track_pocket_m").value)
        xs = [q[0] for q in pts]
        ys = [q[1] for q in pts]
        return (min(xs) - m, min(ys) - m, max(xs) + m, max(ys) + m)

    def _track_search(self, now):
        t = self._track
        self._release_velocity()       # 搜尋由 Nav2 開，速度控制要先放手
        if t["recover"]:
            self._cancel_behavior()
        if not t["search"]:
            if t["last_dist"] is not None and t["last_dist"] < float(
                    self.get_parameter("track_close_m").value):
                # 這個狀況最容易被誤會成「卡住了」。最常見的成因是車退到
                # 牆邊退不動了，而目標就停在正前方一兩公尺 —— 相機近處
                # 看不到（可用範圍 2~8 m）。講清楚比只說「原地等」有用。
                t["message"] = (f"近距離跟丟（最後在 {t['last_dist']:.1f} m），"
                                "轉圈和後退都試過了，也沒有搜尋環線 —— 原地等")
            else:
                t["message"] = "看不到目標，也沒有給搜尋環線 —— 原地等"
            return
        if t["nav_busy"]:
            return                        # 還在往上一個搜尋點走

        # 第一次上環線時，從「離最後看到目標的位置最近的那一段」接進去，
        # 不要總是從第 1 點開始（2026-11-15）。
        #
        # 為什麼：環線的方向和目標繞行的方向是同一個（見積木的預設值），
        # 所以從目標最後出現的地方接上去，等車開到那裡時目標已經往前走了
        # —— 車自然接在它**後面**。從第 1 點開始的話，可能要繞大半圈才追到，
        # 更糟的是有機會在對向的那一段和它迎面錯身而過。
        #
        # 沒看過目標（一開始就看不到）就還是從第 1 點開始。
        if not t["search_started"]:
            t["search_started"] = True
            if t["target"] is not None:
                t["search_i"] = next_wp(nearest_leg(t["target"], t["search"]),
                                        t["loop_step"], len(t["search"]))
                t["message"] = f"從離目標最近的那一段接上環線（第 {t['search_i'] + 1} 點）"

        gx, gy = t["search"][t["search_i"] % len(t["search"])]
        t.update(goal=(gx, gy), goal_t=now, nav_busy=True)
        t["message"] = (f"搜尋中（{'同向' if t['loop_step'] < 0 else '反向'}）："
                        f"往第 {t['search_i'] % len(t['search']) + 1}/"
                        f"{len(t['search'])} 個搜尋點 ({gx:+.1f}, {gy:+.1f})")
        # 搜尋點不指定終點朝向：到了就往下一個走，朝向沒有意義。
        self._send_nav(gx, gy, 999.0)

    def _track_on_nav_done(self, ok):
        """追蹤的子目標結束了。**不**結束整個追蹤任務。"""
        t = self._track
        t["nav_busy"] = False
        if t["phase"] == "搜尋":
            # 到了就換下一個點。到不了也往前跳，不要卡在同一個點重試到死。
            # ⚠️ 加的是 loop_step 不是 1 —— 翻成同向之後要往回走這個列表。
            t["search_i"] = (t["search_i"] + t["loop_step"]) % len(t["search"])
            if not ok:
                t["nav_fail"] += 1
            return
        t["nav_fail"] = 0 if ok else t["nav_fail"] + 1

    def srv_stop(self, request, response):
        """停車：取消 Nav2 的目標，並主動送零速度。"""
        had = (self._nav_handle is not None or self._track["active"]
               or self._task["state"] == "running")
        self._release_velocity()
        self._track.update(active=False, phase="", nav_busy=False,
                           message="被停止指令取消")
        self._cancel_behavior()
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
            if self._track["active"]:
                # 追蹤的子目標被拒 —— 記一次失敗，讓下一拍重算。
                # 常見原因是站位點剛好落在貨架裡（目標貼著貨架時會這樣）。
                self._track["message"] = "Nav2 拒絕了站位點（可能在障礙物裡）"
                self._track_on_nav_done(False)
                return
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
        #
        # ⚠️ 追蹤時這裡**不能**結束任務：一次追蹤會送很多個 Nav2 目標，
        #    每個子目標完成都結束任務的話，積木的 task.seq 會在半路就跳成
        #    done，「等待完成」會以為追蹤做完了（實際上才剛開始）。
        if self._track["active"]:
            self._nav.update(
                state={4: "done", 5: "canceled"}.get(status, "failed"),
                message={4: "子目標到達", 5: "子目標取消"}.get(
                    status, f"子目標失敗（status={status}）"))
            # 5=CANCELED 是我們自己換目標造成的，不算失敗
            if status != 5:
                self._track_on_nav_done(status == 4)
            else:
                self._track["nav_busy"] = False
            return
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
        node._track.update(active=False)
        node._stop_wheels()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
