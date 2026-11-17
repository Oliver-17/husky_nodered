#!/usr/bin/env python3
# =============================================================================
#  t_track_geometry.py — 追蹤的站位點幾何（不需要 ROS、不需要 Gazebo）
#
#  只驗 _standoff_goal 這個純函式。它決定「車該停在哪、車頭朝哪」，
#  算錯的症狀是車開到目標的另一邊、或是背對目標 —— 兩種都不會報錯，
#  在模擬裡要盯著畫面才看得出來，所以值得一支秒級的測試擋在前面。
#
#  ⚠️ 兩道重送閘（站位點移動 > 0.5 m、距離上次 > 2 秒）和「太近不後退」
#     的判斷不在這裡 —— 它們要讀節點參數，得有真的 node。那些由
#     test/sim/t_husky_track.py 在模擬裡驗。
# =============================================================================
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "scripts"))

from husky_api_node import HuskyApi                              # noqa: E402

# 不建 node，直接把方法當純函式用（它完全沒碰 self）
sg = HuskyApi._standoff_goal
fails = []


def check(what, got, want, tol=1e-6):
    if abs(got - want) > tol:
        fails.append(f"{what}：算出來 {got:.4f}，應該是 {want:.4f}")


# ① 目標在正前方（車在 (0,0) 朝 +x，目標在 (10,0)），站位 3 m
#    → 站位點在 (7,0)：**離目標 3 m**，在目標和車之間。
#    ⚠️ 站位距離是從**目標**算起，不是從車算起。
#       （寫這支測試時第一版就把它寫成 3.0 —— 錯的是測試不是程式。）
gx, gy, yaw, d = sg(None, 10.0, 0.0, 0.0, 0.0, 3.0)
check("① 站位點 x", gx, 7.0)
check("① 站位點 y", gy, 0.0)
check("① 車頭朝向（應朝 +x 看向目標）", yaw, 0.0)
check("① 回報的距離", d, 10.0)

# ② 目標在正左方（車在原點，目標在 (0,8)）
gx, gy, yaw, d = sg(None, 0.0, 8.0, 0.0, 0.0, 3.0)
check("② 站位點 x", gx, 0.0)
check("② 站位點 y", gy, 5.0)
check("② 車頭朝向（應朝 +y）", yaw, math.pi / 2)

# ③ 斜的：目標在 (6,8)（距離 10），站位 3
#    → 站位點 = 目標 + 單位向量(目標→車) x 3 = (6,8) + (-0.6,-0.8)x3 = (4.2,5.6)
gx, gy, yaw, d = sg(None, 6.0, 8.0, 0.0, 0.0, 3.0)
check("③ 站位點 x", gx, 4.2)
check("③ 站位點 y", gy, 5.6)
check("③ 回報的距離", d, 10.0)
check("③ 車頭朝向", yaw, math.atan2(8.0, 6.0))

# ④ 站位點一定在「目標→車」的連線上，而且離目標剛好 standoff
#    —— 掃一圈角度和距離，這是整個函式的定義
for deg in range(0, 360, 17):
    for dist in (1.0, 3.0, 5.0, 12.0):
        hx, hy = 2.0, -1.0                       # 車擺在一個不是原點的地方
        tx = hx + dist * math.cos(math.radians(deg))
        ty = hy + dist * math.sin(math.radians(deg))
        gx, gy, yaw, d = sg(None, tx, ty, hx, hy, 3.0)
        check(f"④ deg={deg} dist={dist} 離目標的距離",
              math.hypot(gx - tx, gy - ty), 3.0, 1e-9)
        check(f"④ deg={deg} dist={dist} 回報距離", d, dist, 1e-9)
        # 車頭要朝向目標
        check(f"④ deg={deg} dist={dist} 車頭朝向",
              math.atan2(math.sin(math.atan2(ty - gy, tx - gx) - yaw),
                         math.cos(math.atan2(ty - gy, tx - gx) - yaw)), 0.0, 1e-9)

# ⑤ 車比站位距離更近時，站位點會跑到**車的後面**。
#
#    這就是為什麼太近時**不能用導航目標去退** —— 規劃器會把車掉頭開過去，
#    而終點朝向不被檢查（yaw_goal_tolerance 是 3.14），結果是背對目標。
#    實際的退是用 Nav2 的 BackUp 行為（直線倒車、不轉向）做的，不走這個函式。
#    這裡把前提釘住，免得以後有人「修好」這個函式，讓那個決定失去依據。
gx, gy, yaw, d = sg(None, 10.0, 0.0, 8.0, 0.0, 3.0)   # 車離目標只有 2 m
check("⑤ 距離", d, 2.0)
check("⑤ 站位點 x", gx, 7.0)
if gx >= 8.0:
    fails.append(f"⑤ 車在 x=8、目標在 x=10、站位 3 → 站位點應該在車**後面**"
                 f"（x=7，比車更遠離目標），實際 {gx}")

# ⑥ 幾乎重疊時要回 None，不能回 NaN
gx, gy, yaw, d = sg(None, 1.0, 1.0, 1.0, 1.0, 3.0)
if gx is not None:
    fails.append(f"⑥ 目標和車重疊時應該回 None（方向沒有意義），實際 {gx}")

# =============================================================================
#  環線的繞行方向（2026-11-15）
#
#  追蹤的策略是「反向找 → 對頭相遇 → 倒車退到路口 → 目標轉彎後翻成同向追」。
#  方向差一個正負號，症狀是「車往反方向繞，愈繞愈遠」，而且在模擬裡要盯著
#  畫面好幾分鐘才看得出來 —— 所以用一支秒級的測試釘住。
# =============================================================================
from husky_api_node import nearest_leg, next_wp                  # noqa: E402

# 倉庫的搜尋環線（和車反向）
LOOP = [(5.6, -9.0), (5.6, 9.0), (0.0, 9.0), (0.0, -9.0)]
N = len(LOOP)

# ⑦ nearest_leg 回傳的是**段號**，不是航點號
#    第 0 段 = (5.6,-9)→(5.6,9)、第 1 段 = (5.6,9)→(0,9)、…
cases = [
    ((5.6, 0.0), 0, "東邊那條長走道的中間"),
    ((2.8, 9.0), 1, "北邊那條短邊的中間"),
    ((0.0, 0.0), 2, "西邊那條長走道的中間"),
    ((2.8, -9.0), 3, "南邊那條短邊的中間"),
]
for pt, want, what in cases:
    got = nearest_leg(pt, LOOP)
    if got != want:
        fails.append(f"⑦ {what} {pt} 應該在第 {want} 段，實際第 {got} 段")

# ⑦b 點剛好在航點上時不可以往回倒退（這就是當初 move_target 改成看線段的理由）
#     站在 (5.6,-9)（第 0 段的起點）、照列表順序走 → 下一個該去第 0 段的終點
if next_wp(nearest_leg((5.6, -9.0), LOOP), +1, N) not in (0, 1):
    fails.append("⑦b 站在航點 0 上、正向走，下一個目標不合理")

# ⑧ next_wp：方向決定「下一個航點是這段的終點還是起點」
for leg in range(N):
    if next_wp(leg, +1, N) != (leg + 1) % N:
        fails.append(f"⑧ 正向走第 {leg} 段，下一個航點應該是 {(leg + 1) % N}")
    if next_wp(leg, -1, N) != leg:
        fails.append(f"⑧ 反向走第 {leg} 段，下一個航點應該是 {leg}")

# ⑨ 一路走完一圈：正向是 0→1→2→3→0，反向是 0→3→2→1→0
for step, want in ((+1, [1, 2, 3, 0]), (-1, [3, 2, 1, 0])):
    i, seq = 0, []
    for _ in range(N):
        i = (i + step) % N
        seq.append(i)
    if seq != want:
        fails.append(f"⑨ step={step:+d} 走一圈的順序是 {seq}，應該是 {want}")

# ⑩ 真正要擋的那件事：翻方向之後，車必須往**目標去的方向**走。
#    目標照 target_loop 跑：(0,-9)→(0,9)→(5.6,9)→(5.6,-9)
#    也就是在西邊走道往北、北邊往東、東邊往南、南邊往西。
#    搜尋環線是反向的，所以「同向」= step 為 -1。
#    驗法：目標在西邊走道 (0,0) 往北，翻方向後車的下一個航點應該是 (0,9)。
leg = nearest_leg((0.0, 0.0), LOOP)              # 第 2 段 (0,9)→(0,-9)
wp_same = next_wp(leg, -1, N)                    # 同向
if LOOP[wp_same] != (0.0, 9.0):
    fails.append(f"⑩ 目標在西邊走道往北，翻成同向後下一個航點應該是 (0,9)，"
                 f"實際 {LOOP[wp_same]} —— 方向反了，車會往南愈跑愈遠")
wp_opp = next_wp(leg, +1, N)                     # 反向（搜尋用）
if LOOP[wp_opp] != (0.0, -9.0):
    fails.append(f"⑩ 反向時下一個航點應該是 (0,-9)，實際 {LOOP[wp_opp]}")

# =============================================================================
#  跟隨的控制律（2026-11-15）
#
#  正負號寫反在模擬裡要盯著畫面才看得出來，而且症狀（車往反方向開、原地
#  打轉、該退的時候往前衝）很容易被當成別的問題。所以在這裡釘死。
# =============================================================================
from husky_api_node import follow_cmd                             # noqa: E402

G = dict(standoff=3.0, kp_lin=0.8, kp_yaw=1.5, max_fwd=1.0,
         max_back=0.7, max_yawrate=1.0, deadband=0.3)

# ⑪ 太遠且正對目標 → 往前（v > 0）
v, w = follow_cmd(8.0, 0.0, **G)
if v <= 0:
    fails.append(f"⑪ 離目標 8 m、正對著，應該往前，實際 v={v}")
if abs(w) > 1e-9:
    fails.append(f"⑪ 正對著不該轉，實際 w={w}")
if abs(v - 1.0) > 1e-9:
    fails.append(f"⑪ 誤差 5 m × kp 0.8 = 4.0，應該被上限 1.0 夾住，實際 {v}")

# ⑫ 太近且正對目標 → **倒車**（v < 0），而且被倒車上限夾住
v, w = follow_cmd(1.0, 0.0, **G)
if v >= 0:
    fails.append(f"⑫ 離目標只有 1 m，應該倒車（v<0），實際 v={v}")
if abs(v + 0.7) > 1e-9:
    fails.append(f"⑫ 誤差 -2 m × 0.8 = -1.6，應該被倒車上限 -0.7 夾住，實際 {v}")

# ⑬ 剛好在站位距離附近 → 不動（死區）
for d in (2.8, 3.0, 3.2):
    v, _ = follow_cmd(d, 0.0, **G)
    if v != 0.0:
        fails.append(f"⑬ 離目標 {d} m 在死區內，應該不動，實際 v={v}")

# ⑭ 目標在左邊 → 往左轉（w > 0）；在右邊 → 往右轉（w < 0）
v, w = follow_cmd(8.0, 0.5, **G)
if w <= 0:
    fails.append(f"⑭ 目標在左（e_yaw=+0.5），應該左轉 w>0，實際 {w}")
v, w = follow_cmd(8.0, -0.5, **G)
if w >= 0:
    fails.append(f"⑭ 目標在右（e_yaw=-0.5），應該右轉 w<0，實際 {w}")

# ⑮ 方位誤差很大 → 只轉頭不前進（不然會畫一個大弧）
v, w = follow_cmd(8.0, 2.5, **G)       # 目標幾乎在正後方
if v != 0.0:
    fails.append(f"⑮ 目標在身後（e_yaw=2.5 rad ≈ 143°），應該只轉頭，實際 v={v}")
if abs(w - 1.0) > 1e-9:
    fails.append(f"⑮ 角速度應該被上限 1.0 夾住，實際 {w}")

# ⑯ 角速度上限：誤差再大也不能超過
for e in (0.5, 1.0, 3.0, -3.0):
    _, w = follow_cmd(8.0, e, **G)
    if abs(w) > G["max_yawrate"] + 1e-9:
        fails.append(f"⑯ e_yaw={e} 時 w={w} 超過上限 {G['max_yawrate']}")

# =============================================================================
#  擋牆與倒車讓路（2026-11-15）
#
#  Oliver 回報兩件事：①對頭相遇倒車後停在原地，目標直接穿過去
#                    ②轉彎時直接送速度會撞牆，幾乎每次
#  ⑰~⑲ 驗函式本身，⑳㉑ 用運動學小模擬重現這兩個情境，
#  而且各有一個**舊做法的對照組**，證明這幾支測試抓得到那兩個毛病。
# =============================================================================
from husky_api_node import (follow_world, inflate_boxes, clamp_to_boxes,  # noqa: E402
                            clamp_to_area, point_box_dist, TRACK_WALL_MARGIN_M,
                            TRACK_POCKET_M, TRACK_ROT_CLEAR_M)

GW = dict(G, kp_lin=2.0)                   # 現在實際用的 kp
SHELF2 = (2.0, -7.0, 3.6, 7.0)             # warehouse_geometry.yaml 的第 2 排貨架
NORTH = (-20.0, 12.0, 20.0, 14.0)
SOUTH = (-20.0, -14.0, 20.0, -12.0)
RAW = [SHELF2, (-3.6, -7.0, -2.0, 7.0), (7.6, -7.0, 9.2, 7.0), NORTH, SOUTH]
WALLS = inflate_boxes(RAW, TRACK_WALL_MARGIN_M)
LOOP_PTS = [(0.0, -9.0), (0.0, 9.0), (5.6, 9.0), (5.6, -9.0)]   # 目標車的繞行順序


def area_of(pocket):
    xs = [q[0] for q in LOOP_PTS]
    ys = [q[1] for q in LOOP_PTS]
    return (min(xs) - pocket, min(ys) - pocket, max(xs) + pocket, max(ys) + pocket)


# ⑰ 什麼都沒擋到時，和原本的控制律**完全一樣**（不改變平常跟隨的手感）
for (px, py, yaw, tx, ty) in [(0, 0, 0.0, 8, 0), (0, 0, 0.0, 1, 0), (0, 0, 0.0, 3, 0),
                              (0, 0, 0.3, 5, 2), (0, 0, -1.0, 4, -3), (0, 0, 2.8, 6, 0.5)]:
    d = math.hypot(tx - px, ty - py)
    e = math.atan2(ty - py, tx - px) - yaw
    e = (e + math.pi) % (2 * math.pi) - math.pi
    v0, w0 = follow_cmd(d, e, **GW)
    v1, w1, why = follow_world((px, py), yaw, (tx, ty), walls=[], area=None, **GW)
    if abs(v0 - v1) > 1e-9 or abs(w0 - w1) > 1e-9 or why is not None:
        fails.append(f"⑰ 沒有牆時應該和 follow_cmd 一樣：({px},{py})→({tx},{ty}) "
                     f"舊 ({v0:.3f},{w0:.3f}) 新 ({v1:.3f},{w1:.3f}) why={why}")

# ⑱ 正對貨架面衝過去 → 只砍法線分量，沿面的分量留著（＝貼著貨架滑）
v, k = clamp_to_boxes((0.8, 0.0), (0.8, 0.6), WALLS, 1.0)
if k is None or abs(v[0]) > 1e-9 or abs(v[1] - 0.6) > 1e-9:
    fails.append(f"⑱ 斜著衝向貨架西面，應該砍掉 x、留下 y=0.6，實際 {v} 方框 {k}")
#    已經在加大後的方框裡 → 往外可以，往裡不行
v, k = clamp_to_boxes((1.5, 0.0), (-0.5, 0.0), WALLS, 1.0)
if abs(v[0] + 0.5) > 1e-9:
    fails.append(f"⑱ 在框內往外走應該放行，實際 {v}")
v, k = clamp_to_boxes((1.5, 0.0), (0.5, 0.0), WALLS, 1.0)
if abs(v[0]) > 1e-9:
    fails.append(f"⑱ 在框內往更裡面走應該被砍，實際 {v}")

# ⑲ 範圍：往外且會出界才砍；已經在外面時往裡照樣放行
A = area_of(TRACK_POCKET_M)
v, cut = clamp_to_area((0.0, 10.7), (0.0, 0.7), A, 0.3)
if not cut or v[1] != 0.0:
    fails.append(f"⑲ 退到 y=10.7 還往北，應該被擋，實際 {v}")
v, cut = clamp_to_area((-14.0, 9.0), (1.0, 0.0), A, 0.3)
if cut:
    fails.append("⑲ 從生成點往環線開（往裡）不該被擋")


# ---------- 運動學小模擬 ----------------------------------------------------
HALF_L, HALF_W = 0.495, 0.335


def body_clear(cx, cy, yaw, boxes):
    """車身矩形到方框的最近距離（和 test/sim/watch_walls.py 同一套算法）。"""
    c, s_ = math.cos(yaw), math.sin(yaw)
    corners = [(cx + a * HALF_L * c - b * HALF_W * s_, cy + a * HALF_L * s_ + b * HALF_W * c)
               for a in (-1, 1) for b in (-1, 1)]
    best = 1e9
    for bx in boxes:
        for qx, qy in corners:
            dx = max(bx[0] - qx, 0.0, qx - bx[2])
            dy = max(bx[1] - qy, 0.0, qy - bx[3])
            dd = (math.hypot(dx, dy) if (dx or dy)
                  else -min(qx - bx[0], bx[2] - qx, qy - bx[1], bx[3] - qy))
            best = min(best, dd)
        for qx in (bx[0], bx[2]):
            for qy in (bx[1], bx[3]):
                lx = (qx - cx) * c + (qy - cy) * s_
                ly = -(qx - cx) * s_ + (qy - cy) * c
                best = min(best, math.hypot(max(abs(lx) - HALF_L, 0), max(abs(ly) - HALF_W, 0)))
    return best


def target_on_loop(s0, t, speed=0.5):
    """目標車沿 LOOP_PTS 走：起點是第 0 段上距離 s0 的位置。"""
    dist = s0 + speed * t
    segs = [(LOOP_PTS[i], LOOP_PTS[(i + 1) % 4]) for i in range(4)]
    total = sum(math.dist(a, b) for a, b in segs)
    dist %= total
    for a, b in segs:
        L = math.dist(a, b)
        if dist <= L:
            f = dist / L
            return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))
        dist -= L
    return LOOP_PTS[0]


def simulate(x, y, yaw, target_fn, secs, ctrl, dt=0.05):
    """獨輪車模型，含加減速上限（和 warehouse_nav2_husky.yaml 一致：
    acc 1.0、decel 1.5、角加速度 3.2）。回傳每一步的 (x, y, yaw, 目標)。"""
    v = w = 0.0
    out = []
    for i in range(int(secs / dt)):
        tg = target_fn(i * dt)
        vc, wc = ctrl((x, y), yaw, tg)
        dv = vc - v
        lim = (1.0 if abs(vc) > abs(v) else 1.5) * dt
        v += max(-lim, min(lim, dv))
        w += max(-3.2 * dt, min(3.2 * dt, wc - w))
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += w * dt
        out.append((x, y, yaw, tg))
    return out


def new_ctrl(area):
    def f(p, yaw, tg):
        v, w, _ = follow_world(p, yaw, tg, walls=RAW, area=area, **GW)
        return v, w
    return f


def old_ctrl(p, yaw, tg):
    d = math.hypot(tg[0] - p[0], tg[1] - p[1])
    e = math.atan2(tg[1] - p[1], tg[0] - p[0]) - yaw
    return follow_cmd(d, (e + math.pi) % (2 * math.pi) - math.pi, **GW)


# ⑳ 轉彎撞牆（問題②）：目標在貨架另一側的角落，直線追會切過貨架角
#    車在 (0, 8.5) 朝東，目標停在 (5.6, 6.0)：連線在 x=3.6 時 y=6.89 → 穿過貨架東北角
fixed = lambda t: (5.6, 6.0)                                        # noqa: E731
run_new = simulate(0.0, 8.5, 0.0, fixed, 20.0, new_ctrl(area_of(TRACK_POCKET_M)))
run_old = simulate(0.0, 8.5, 0.0, fixed, 20.0, old_ctrl)
c_new = min(body_clear(x, y, yw, RAW) for x, y, yw, _ in run_new)
c_old = min(body_clear(x, y, yw, RAW) for x, y, yw, _ in run_old)
end = run_new[-1]
d_end = math.hypot(end[3][0] - end[0], end[3][1] - end[1])
if c_old > 0.0:
    fails.append(f"⑳ 對照組（舊控制律）應該撞進貨架，結果最近 {c_old:.2f} m —— "
                 "這個情境沒有重現問題，測試本身無效")
if c_new <= 0.05:
    fails.append(f"⑳ 新控制律撞到了：車身離貨架/牆最近 {c_new:.2f} m")
if d_end > GW["standoff"] + 0.6:
    fails.append(f"⑳ 繞過貨架角之後應該到站位附近，結束時離目標 {d_end:.2f} m "
                 "（被擋住沒繞過去？）")

# ㉑ 對頭相遇倒車讓路（問題①）：
#    目標在西邊走道 (0,3) 往北開，車在 (0,6) 朝南對著它。路口 (0,9) 目標右轉往東。
#    正確：車退過路口、停在外側，目標從車前面過去，車再從後面追。
tgt = lambda t: target_on_loop(12.0, t)                             # noqa: E731  (0,3)
run = simulate(0.0, 6.0, -math.pi / 2, tgt, 40.0, new_ctrl(area_of(TRACK_POCKET_M)))
sep_min = min(math.hypot(tg[0] - x, tg[1] - y) for x, y, _, tg in run)
clr = min(body_clear(x, y, yw, RAW) for x, y, yw, _ in run)
end = run[-1]
d_end = math.hypot(end[3][0] - end[0], end[3][1] - end[1])
y_max = max(y for _, y, _, _ in run)
if sep_min < 0.8:
    fails.append(f"㉑ 目標穿過地面車了（中心最近 {sep_min:.2f} m < 0.8）")
if y_max < 9.8:
    fails.append(f"㉑ 車應該退過路口 y=9、停在外側，實際最北只到 y={y_max:.2f}")
if clr <= 0.05:
    fails.append(f"㉑ 退到路口外側時撞牆了（車身離牆 {clr:.2f} m）")
if d_end > GW["standoff"] + 1.0:
    fails.append(f"㉑ 讓路之後應該從後面跟上，40 秒時離目標 {d_end:.2f} m")
#    對照組：不讓退出環線（pocket=0，等同下午那版的效果）→ 車停在路上，目標穿過去
run0 = simulate(0.0, 6.0, -math.pi / 2, tgt, 40.0, new_ctrl(area_of(0.0)))
sep0 = min(math.hypot(tg[0] - x, tg[1] - y) for x, y, _, tg in run0)
if sep0 >= 0.8:
    fails.append(f"㉑ 對照組（不准退出環線）應該被穿過，結果最近 {sep0:.2f} m —— "
                 "這個情境沒有重現問題，測試本身無效")
# ㉒ 已經在加大後的方框裡（餘裕被吃掉）→ 倒車**不准**帶任何往裡的分量
#    2026-11-15 模擬實測的那一刻：車 (-1.30,-6.85) 朝 50°，目標在車頭前 1.9 m，
#    舊版靠「被砍不到一半就照走」斜斜退進去，中心離貨架角 0.78 → 0.52 m。
SHELF1 = (-3.6, -7.0, -2.0, 7.0)
yaw = math.radians(50)
p = (-1.30, -6.85)
tg = (p[0] + 1.9 * math.cos(yaw), p[1] + 1.9 * math.sin(yaw))
v, w, why = follow_world(p, yaw, tg, walls=RAW + [SHELF1], area=None, **GW)
step = (p[0] + v * math.cos(yaw) * 0.1, p[1] + v * math.sin(yaw) * 0.1)
if point_box_dist(step, SHELF1) < point_box_dist(p, SHELF1) - 1e-9:
    fails.append(f"㉒ 已在餘裕區裡，倒車不該再靠近貨架：v={v:+.2f} why={why}")

# ㉓ 中心離貨架角比外接半徑還近 → **不准轉頭**，而且要沿車身軸往遠離的方向移
#    （同一次實測：中心 0.52 m 時原地轉了 26°，車尾掃上貨架角）
p, yaw = (-1.49, -7.06), math.radians(49)
d0 = point_box_dist(p, SHELF1)
assert d0 < TRACK_ROT_CLEAR_M, "測試前提錯了"
for tg in [(0.0, -3.7), (3.0, -7.0), (-1.0, -10.0)]:     # 目標在哪都一樣
    v, w, why = follow_world(p, yaw, tg, walls=RAW + [SHELF1], area=None, **GW)
    nxt = (p[0] + v * math.cos(yaw) * 0.1, p[1] + v * math.sin(yaw) * 0.1)
    if w != 0.0:
        fails.append(f"㉓ 中心離貨架 {d0:.2f} m（< {TRACK_ROT_CLEAR_M}），不該轉頭，實際 w={w}")
    if v == 0.0 or point_box_dist(nxt, SHELF1) <= d0:
        fails.append(f"㉓ 應該沿車身軸直直移開，實際 v={v}，移完離貨架 "
                     f"{point_box_dist(nxt, SHELF1):.3f}（原本 {d0:.3f}）")

# ㉔ 把那一段整個重跑：路口 (0,-9) 讓路完，車卡在 x=-2.8 那排貨架的東南角旁，
#    目標從 (0,-8.23) 往北開走。車要脫出來跟上，而且全程不碰貨架。
RAW1 = RAW + [SHELF1]
tgt2 = lambda t: target_on_loop(0.77, t)                            # noqa: E731
h0 = math.atan2(-8.23 + 7.05, 0.0 + 1.22)


def ctrl_with(rot):
    def f(p, yaw, tg):
        v, w, _ = follow_world(p, yaw, tg, walls=RAW1, area=area_of(TRACK_POCKET_M),
                               rot_clear=rot, **GW)
        return v, w
    return f


run = simulate(-1.22, -7.05, h0, tgt2, 25.0, ctrl_with(TRACK_ROT_CLEAR_M))
c24 = min(body_clear(x, y, yw, RAW1) for x, y, yw, _ in run)
d24 = math.hypot(run[-1][3][0] - run[-1][0], run[-1][3][1] - run[-1][1])
if c24 <= 0.05:
    fails.append(f"㉔ 從貨架角旁脫出時撞到了：車身離貨架最近 {c24:.2f} m")
if d24 > GW["standoff"] + 1.0:
    fails.append(f"㉔ 25 秒後應該跟上目標，實際離目標 {d24:.2f} m")
run_nr = simulate(-1.22, -7.05, h0, tgt2, 25.0, ctrl_with(0.0))
c24_nr = min(body_clear(x, y, yw, RAW1) for x, y, yw, _ in run_nr)

# =============================================================================
#  跟隨中／已就位來回跳（2026-11-16，Oliver 回報）
#
#  不只是文字在跳，車也在頓：舊的不動區邊緣從 0 跳到 kp×0.3 = 0.6 m/s，
#  目標 0.5 m/s 時平衡點（落後 0.25 m）落在不動區裡 →「停、衝、停」。
#  ㉕ 驗速度在不動區邊緣連續；㉖ 用一維小模擬跟 0.5 m/s 的目標，
#  有對照組（舊的跳躍式）證明這支抓得到；㉗ 驗訊息防抖。
# =============================================================================
from husky_api_node import dist_speed, hold_kind                 # noqa: E402

# ㉕ 不動區邊緣連續：剛出不動區時速度要接近 0，不是 0.6
for e in (0.3001, -0.3001):
    s25 = dist_speed(e, 2.0, 0.3)
    if abs(s25) > 0.01:
        fails.append(f"㉕ 誤差 {e} m 剛出不動區，速度應該接近 0，實際 {s25:.3f}")
if dist_speed(0.29, 2.0, 0.3) != 0.0:
    fails.append("㉕ 不動區裡速度應該是 0")
if dist_speed(1.3, 2.0, 0.3) <= 0 or dist_speed(-1.3, 2.0, 0.3) >= 0:
    fails.append("㉕ 正負號錯了：太遠要往前（正）、太近要倒車（負）")


def follow_1d(spd, law, secs=40.0, dt=0.05):
    """一維：目標以 spd 往前走，車在後面跟，10 Hz 下指令、含加減速上限。
    回傳 10 秒之後每一拍的 (指令, 實際距離)。"""
    x = v = vc = 0.0
    tx = 4.0
    out = []
    for i in range(int(secs / dt)):
        tx += spd * dt
        if i % 2 == 0:
            vc = law(tx - x)
        lim = (1.0 if abs(vc) > abs(v) else 1.5) * dt
        v += max(-lim, min(lim, vc - v))
        x += v * dt
        if i * dt > 10.0 and i % 2 == 0:
            out.append((vc, tx - x))
    return out


def law_new(d):
    v, _, _ = follow_world((0.0, 0.0), 0.0, (d, 0.0), walls=[], area=None, **GW)
    return v


def law_old(d):                       # 舊版：不動區外直接 kp × 誤差
    e = d - GW["standoff"]
    return 0.0 if abs(e) < GW["deadband"] else max(-0.7, min(1.0, GW["kp_lin"] * e))


def stop_go(run):
    return sum(1 for a, b in zip(run, run[1:]) if (a[0] == 0.0) != (b[0] == 0.0))


# ㉖ 跟 0.5 m/s（倉庫目標車的速度）和 0.4 m/s（move_target 預設）
for spd in (0.5, 0.4):
    r_new, r_old = follow_1d(spd, law_new), follow_1d(spd, law_old)
    n_new, n_old = stop_go(r_new), stop_go(r_old)
    jump = max(abs(a[0] - b[0]) for a, b in zip(r_new, r_new[1:]))
    if n_new > 0:
        fails.append(f"㉖ 目標 {spd} m/s：30 秒內停／走切換 {n_new} 次，應該 0 次")
    if jump > 0.05:
        fails.append(f"㉖ 目標 {spd} m/s：速度指令一拍跳 {jump:.2f} m/s，應該是平順的")
    if n_old < 10:
        fails.append(f"㉖ 對照組（舊的跳躍式）只切換 {n_old} 次 —— 這支測試抓不到原本的毛病了")
    gap = sum(a[1] for a in r_new) / len(r_new)
    if not 3.3 < gap < 3.8:
        fails.append(f"㉖ 目標 {spd} m/s：平均距離 {gap:.2f} m，應該在 3.3~3.8")
    if spd == 0.5:
        n26, o26, g26 = n_new, n_old, gap

# ㉗ 訊息防抖（假時鐘）
st = {}
seq = []
for i in range(30):                   # 每 0.3 秒來回切一次 → 一直維持「跟隨」
    seq.append(hold_kind(st, "following" if (i // 3) % 2 == 0 else "settled",
                         i * 0.1, 1.0))
if set(seq) != {"following"}:
    fails.append(f"㉗ 每 0.3 秒來回切時應該一直顯示跟隨，實際出現 {sorted(set(seq))}")
st = {}
hold_kind(st, "following", 0.0, 1.0)
got = [hold_kind(st, "settled", 0.1 * i, 1.0) for i in range(1, 13)]
if got[8] != "following" or got[-1] != "settled":
    fails.append(f"㉗ 持續 1 秒才該換成已就位：0.9 秒時 {got[8]}、1.2 秒時 {got[-1]}")
if hold_kind(st, "close", 1.3, 1.0, immediate=True) != "close":
    fails.append("㉗ 擋牆／太靠近要馬上顯示，不能被防抖延後")

# =============================================================================
#  沿目標的足跡跟（2026-11-16，Oliver 回報「到轉角一定轉到快撞牆、一直調整、
#  目標不見；不見後也沒去它最後出現的位置，而是去上個經過的轉角」）
#  ㉘~㉚ 驗純函式；㉛ 用轉角小模擬（含相機視角與貨架擋視線）比新舊。
# =============================================================================
from husky_api_node import vel_update                              # noqa: E402
from husky_api_node import (trail_add, trail_plan, trail_heading,       # noqa: E402
                            leg_and_dir, nearest_leg, next_wp, wrap_pi)

# ㉘ 足跡：每 0.3 m 一點、總長有上限、跳太遠就重新開始
tr = []
for i in range(200):
    trail_add(tr, (0.0, i * 0.05))                 # 走 10 m，每步 5 cm
gaps = [math.dist(a, b) for a, b in zip(tr, tr[1:])]
if min(gaps) < 0.3 - 1e-9:
    fails.append(f"㉘ 足跡點的間距應該 ≥ 0.3 m，實際最小 {min(gaps):.3f}")
for i in range(200, 800):
    trail_add(tr, (0.0, i * 0.05))                 # 再走 30 m → 應該被砍到 25 m 以內
if sum(math.dist(a, b) for a, b in zip(tr, tr[1:])) > 25.0 + 1e-6:
    fails.append("㉘ 足跡總長應該被限制在 25 m 以內")
if not trail_add(tr, (10.0, 10.0)) or len(tr) != 1:
    fails.append("㉘ 新的點離尾巴太遠（> 2.5 m）時足跡要重新開始")

# ㉙ trail_plan：在足跡上 → 瞄前方、距離沿路量；不在足跡上 / 在尾巴前面 → None
L = [(0.0, y * 0.3) for y in range(31)] + [(x * 0.3, 9.0) for x in range(1, 11)]
tg = (3.0 + 0.3, 9.0)
pl = trail_plan((0.0, 6.0), L, tg)
if pl is None:
    fails.append("㉙ 車在足跡上（目標後面），應該沿足跡跟")
else:
    aim, gap = pl
    if abs(aim[0]) > 1e-6 or abs(aim[1] - 7.5) > 1e-6:
        fails.append(f"㉙ 瞄準點應該在足跡上前方 1.5 m 的 (0, 7.5)，實際 {aim}")
    if abs(gap - (3.0 + 3.0 + 0.3)) > 1e-6:
        fails.append(f"㉙ 沿路距離應該是 3 + 3 + 0.3 = 6.3（不是直線 4.4），實際 {gap:.2f}")
if trail_plan((-1.5, 6.0), L, tg) is not None:
    fails.append("㉙ 車離足跡 1.5 m，不該沿足跡跟（交回直線追）")
if trail_plan((4.5, 9.0), L, tg) is not None:
    fails.append("㉙ 車在足跡尾巴前面（對頭相遇），不該沿足跡跟")
hd = trail_heading(L)
if hd is None or abs(hd) > 1e-6:
    fails.append(f"㉙ 足跡最後往東走，方向應該是 0，實際 {hd}")

# ㉚ 跟丟後的搜尋起點：目標往北走、在 (0,9) 轉往東、最後出現在 (0.6, 9)
#    （搜尋環線用積木預設的順序，也就是和目標反向的那一串）
SEARCH = [(5.6, -9.0), (5.6, 9.0), (0.0, 9.0), (0.0, -9.0)]
leg, step = leg_and_dir((0.6, 9.0), 0.0, SEARCH)
nxt = SEARCH[next_wp(leg, step, 4)]
if nxt != (5.6, 9.0):
    fails.append(f"㉚ 目標往東走，應該先去它前面的轉角 (5.6, 9)，實際去 {nxt}")
old_nxt = SEARCH[next_wp(nearest_leg((0.6, 9.0), SEARCH), 1, 4)]   # 舊：沒翻方向時
if old_nxt != (0.0, 9.0):
    fails.append(f"㉚ 對照組：舊做法應該會去剛經過的轉角 (0, 9)，實際 {old_nxt}")
# 剛好在轉角上、量到的點偏向南邊那一段時，方向還是要分得出來
leg, step = leg_and_dir((0.15, 8.75), 0.0, SEARCH)
if SEARCH[next_wp(leg, step, 4)] != (5.6, 9.0):
    fails.append("㉚ 目標在轉角上、量到的點偏南：還是應該往東邊的轉角去")


# ㉛ 轉角小模擬：目標從 (0,2) 往北走、在 (0,9) 轉東、在 (5.6,9) 轉南，車在後面 3.5 m
#    相機模型：前視 ±35°（horizontal_fov 1.204）、1.5~8 m、貨架會擋視線；
#    量到的是目標表面（偏向車 0.28 m）＋ 0.1 m 雜訊。
def _seg_hits(p, q, b):
    t0, t1 = 0.0, 1.0
    for i in (0, 1):
        d = q[i] - p[i]
        if abs(d) < 1e-12:
            if p[i] < b[i] or p[i] > b[i + 2]:
                return False
        else:
            a1, a2 = (b[i] - p[i]) / d, (b[i + 2] - p[i]) / d
            t0, t1 = max(t0, min(a1, a2)), min(t1, max(a1, a2))
            if t0 > t1:
                return False
    return True


def _visible(p, yaw, tg):
    d = math.dist(p, tg)
    if not 1.5 <= d <= 8.0:
        return False
    if abs(wrap_pi(math.atan2(tg[1] - p[1], tg[0] - p[0]) - yaw)) > 0.6:
        return False
    return not any(_seg_hits(p, tg, b) for b in RAW[:3])


JERK = [0.0]                 # corner_run 累積的 |Δω|（車頭轉速變化，看會不會抖）


def corner_run(new, seed, secs=45.0, dt=0.05, safety="rules", fov=False):
    """回傳 (擋牆介入秒數, 看不到的秒數, 車身離貨架最近, 最後離目標多遠)。
    new=True 照節點的新流程：看得到 → 沿足跡跟；看不到 → 沿足跡開到尾巴、轉向；
    new=False 是舊的：朝最後看到的位置直線追 5 秒。"""
    import random
    rnd = random.Random(seed)
    x, y, yaw, v, w = 0.0, -1.5, math.pi / 2, 0.0, 0.0
    last, last_t, filt, trail, chase = None, -1e9, None, [], None
    vst, vel = {}, (0.0, 0.0)
    wall = blind = 0.0
    clear = 9.0
    vc = wc = 0.0
    for i in range(int(secs / dt)):
        t = i * dt
        tg = target_on_loop(11.0, t)
        if i % 2 == 0:
            if _visible((x, y), yaw, tg):
                ux, uy = x - tg[0], y - tg[1]
                n = math.hypot(ux, uy)
                last = (tg[0] + 0.28 * ux / n + rnd.gauss(0, 0.1),
                        tg[1] + 0.28 * uy / n + rnd.gauss(0, 0.1))
                last_t = t
                a = 1 - math.exp(-0.1 / 0.3)
                filt = last if filt is None or math.dist(filt, last) > 1.5 else \
                    (filt[0] + a * (last[0] - filt[0]), filt[1] + a * (last[1] - filt[1]))
                trail_add(trail, filt)
                vel = vel_update(vst, filt, t)
                chase = None
            else:
                blind += 0.1
            age, why = t - last_t, None
            vc = wc = 0.0
            if last is not None and not new and age <= 5.0:
                vc, wc, why = follow_world((x, y), yaw, last, walls=RAW, **dict(GW, safety=safety))
            elif last is not None and new and age <= 1.5:
                fv = (filt, vel) if fov else None
                pl = trail_plan((x, y), trail, filt)
                if pl and pl[1] >= GW["standoff"] - GW["deadband"]:
                    vc, wc, why = follow_world((x, y), yaw, filt, walls=RAW,
                                               aim=pl[0], gap=pl[1], fov=fv,
                                               **dict(GW, safety=safety))
                else:
                    vc, wc, why = follow_world((x, y), yaw, last, walls=RAW, fov=fv,
                                               **dict(GW, safety=safety))
            elif last is not None and new:
                if chase is None:
                    chase = "go" if trail_plan((x, y), trail, trail[-1], on_m=2.0) else "done"
                if chase == "go":
                    pl = trail_plan((x, y), trail, trail[-1], on_m=2.0)
                    if pl is None:
                        chase = "turn"
                    else:
                        vc, wc, why = follow_world((x, y), yaw, trail[-1],
                                                   walls=RAW, aim=pl[0], gap=pl[1],
                                                   **dict(GW, standoff=0.0, safety=safety))
                if chase == "turn":
                    h = trail_heading(trail)
                    look = (x + 2 * math.cos(h), y + 2 * math.sin(h))
                    vc, wc, why = follow_world((x, y), yaw, look, walls=RAW,
                                               **dict(GW, standoff=2.0, safety=safety))
                    if abs(wrap_pi(h - yaw)) < 0.25:
                        chase = "done"
            if why:
                wall += 0.1
        lim = (1.0 if abs(vc) > abs(v) else 1.5) * dt
        v += max(-lim, min(lim, vc - v))
        w_prev = w
        w += max(-3.2 * dt, min(3.2 * dt, wc - w))
        JERK[0] += abs(w - w_prev)
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += w * dt
        clear = min(clear, body_clear(x, y, yaw, RAW))
    return wall, blind, clear, math.dist((x, y), target_on_loop(11.0, secs))


r31n = [corner_run(True, sd) for sd in (1, 2, 3)]
r31o = [corner_run(False, sd) for sd in (1, 2, 3)]
r31c = [corner_run(True, sd, safety="cbf") for sd in (1, 2, 3)]
for sd, (wl, bl, cl, de) in zip((1, 2, 3), r31n):
    if wl > 0.5:
        fails.append(f"㉛ 新（種子 {sd}）：擋牆介入 {wl:.1f} 秒，沿足跡跟應該幾乎不碰牆")
    if cl < 1.0:
        fails.append(f"㉛ 新（種子 {sd}）：車身離貨架最近 {cl:.2f} m，應該 ≥ 1.0")
    if de > 5.0:
        fails.append(f"㉛ 新（種子 {sd}）：繞過兩個轉角後離目標 {de:.1f} m，跟丟了")
if not any(de > 6.0 or cl < 0.6 for _wl, _bl, cl, de in r31o):
    fails.append("㉛ 對照組（舊的直線追）沒有出現貼牆或跟丟 —— 這支測試抓不到原本的毛病了")

# =============================================================================
#  CBF 安全濾波（2026-11-17）
#
#  ㉜~㊱ 驗函式本身（C1）；㊲ 拿同一批情境和手寫規則並排比（C2）。
#  CBF 是用來**取代**三層手寫規則的，所以每一個當初逼出規則的情境
#  （⑳ 切貨架角、㉑ 對頭讓路、㉓㉔ 貨架角旁轉頭、㉛ 轉角）它都要過。
# =============================================================================
from husky_api_node import (cbf_filter, cbf_constraints, cbf_solve,          # noqa: E402
                            CBF_W_YAW, CBF_CIRCLE_OFF_M, CBF_CIRCLE_R_M)
import random                                                               # noqa: E402

GC = dict(GW, safety="cbf")

# ㉜ 什麼都沒擋到時，和原本的控制律完全一樣（同⑰的情境）
for (px, py, yaw, tx, ty) in [(0, 0, 0.0, 8, 0), (0, 0, 0.0, 1, 0), (0, 0, 0.0, 3, 0),
                              (0, 0, 0.3, 5, 2), (0, 0, -1.0, 4, -3), (0, 0, 2.8, 6, 0.5)]:
    d = math.hypot(tx - px, ty - py)
    e = wrap_pi(math.atan2(ty - py, tx - px) - yaw)
    v0, w0 = follow_cmd(d, e, **GW)
    v1, w1, why = follow_world((px, py), yaw, (tx, ty), walls=[], area=None, **GC)
    if abs(v0 - v1) > 1e-9 or abs(w0 - w1) > 1e-9 or why is not None:
        fails.append(f"㉜ CBF 沒有牆時應該和 follow_cmd 一樣：({px},{py})→({tx},{ty}) "
                     f"({v0:.3f},{w0:.3f}) vs ({v1:.3f},{w1:.3f}) why={why}")

# ㉝ 求解器的答案真的是「可行裡最近的」：和把 (v, ω) 平面切 241x241 格的暴力搜尋比
#    暴力搜尋只會比真正的最佳解差（格點不一定落在最佳點上），
#    所以要求：解可行，而且代價 ≤ 暴力搜尋找到的最佳代價（多 1e-3 容許）。
rnd = random.Random(7)
n33 = bad33 = 0
while n33 < 60:
    p = (rnd.uniform(-1.5, 7.0), rnd.uniform(-10.5, 10.5))
    yaw = rnd.uniform(-math.pi, math.pi)
    cons = cbf_constraints(p, yaw, RAW, area_of(TRACK_POCKET_M))
    if not cons:
        continue
    v0, w0 = rnd.uniform(-0.7, 1.0), rnd.uniform(-1.0, 1.0)
    v, w, okk = cbf_solve(v0, w0, cons, 1.0, 0.7, 1.0)
    if not okk:
        continue
    n33 += 1
    cost = (v - v0) ** 2 + CBF_W_YAW * (w - w0) ** 2
    feas = all(av * v + aw * w >= b - 1e-6 for av, aw, b, _ in cons)
    grid_best = math.inf
    for i in range(241):
        gv = -0.7 + 1.7 * i / 240
        for j in range(241):
            gw = -1.0 + 2.0 * j / 240
            if all(av * gv + aw * gw >= b for av, aw, b, _ in cons):
                grid_best = min(grid_best, (gv - v0) ** 2 + CBF_W_YAW * (gw - w0) ** 2)
    if not feas or cost > grid_best + 1e-3:
        bad33 += 1
        fails.append(f"㉝ 求解器不是最佳：p={p} 代價 {cost:.4f}，格點最佳 {grid_best:.4f}，"
                     f"可行={feas}")

# ㉞ 只有一條限制起作用時 ＝ null space 投影：修正量和那條限制的法向量平行
#    （在 z=(v, √w_yaw·ω) 座標裡），和它垂直的分量完全不動。
p, yaw = (1.2, 0.0), 0.5                  # 車頭斜斜對著貨架西面（x=2.0）
cons = cbf_constraints(p, yaw, RAW)
v0, w0 = 0.8, 0.1
v, w, _ = cbf_solve(v0, w0, cons, 1.0, 0.7, 1.0)
k = math.sqrt(CBF_W_YAW)
act = [(av, aw / k) for av, aw, b, _ in cons if abs(av * v + aw * w - b) < 1e-6]
dz = (v - v0, k * (w - w0))
if len(act) != 1:
    fails.append(f"㉞ 這個情境應該剛好一條限制起作用，實際 {len(act)} 條")
else:
    a = act[0]
    cross = a[0] * dz[1] - a[1] * dz[0]
    if abs(cross) > 1e-9 or math.hypot(*dz) < 1e-3:
        fails.append(f"㉞ 修正量應該和限制的法向量平行（只改違規的分量），"
                     f"外積 {cross:.2e}，修正量 {dz}")

# ㉟ 隨機放車、隨機目標，跑 150 次 × 8 秒：車身從來不碰貨架或牆
#    （模擬含加減速上限，和 P 控制器、取樣頻率都不理想 —— 量的是「實際上」夠不夠安全）
def cbf_ctrl(area=None, alpha=None):
    def f(p, yaw, tg):
        v, w, _ = follow_world(p, yaw, tg, walls=RAW, area=area, cbf_alpha=alpha, **GC)
        return v, w
    return f


rnd = random.Random(11)
c35 = 9.0
for _ in range(150):
    while True:
        x0, y0 = rnd.uniform(-1.5, 7.0), rnd.uniform(-11.0, 11.0)
        yaw0 = rnd.uniform(-math.pi, math.pi)
        if body_clear(x0, y0, yaw0, RAW) > 0.3:
            break
    tg = (rnd.uniform(-1.5, 7.0), rnd.uniform(-11.0, 11.0))
    run = simulate(x0, y0, yaw0, lambda t, tg=tg: tg, 8.0, cbf_ctrl())
    c35 = min(c35, min(body_clear(x, y, yw, RAW) for x, y, yw, _ in run))
if c35 <= 0.0:
    fails.append(f"㉟ 隨機情境裡車身碰到貨架或牆了（最近 {c35:.3f} m）")

# ㊱ 貨架角旁原地轉頭（㉓ 的情境：中心離貨架角 0.52 m）。CBF 沒有「不准轉頭」那條規則，
#    靠後圓的限制自己擋住 —— 車尾不能掃到貨架。對照組：拿掉 CBF 照轉，會掃到。
p36, yaw36 = (-1.49, -7.06), math.radians(49)
c36, c36_raw = 9.0, 9.0
for tg in [(0.0, -3.7), (3.0, -7.0), (-1.0, -10.0)]:
    run = simulate(p36[0], p36[1], yaw36, lambda t, tg=tg: tg, 4.0,
                   lambda p, yaw, g: follow_world(p, yaw, g, walls=RAW1, **GC)[:2])
    c36 = min(c36, min(body_clear(x, y, yw, RAW1) for x, y, yw, _ in run))
    run = simulate(p36[0], p36[1], yaw36, lambda t, tg=tg: tg, 4.0, old_ctrl)
    c36_raw = min(c36_raw, min(body_clear(x, y, yw, RAW1) for x, y, yw, _ in run))
if c36 <= 0.0:
    fails.append(f"㊱ CBF 在貨架角旁轉頭掃到了（最近 {c36:.3f} m）")
if c36_raw > 0.0:
    fails.append(f"㊱ 對照組（不擋）應該掃到貨架，結果最近 {c36_raw:.2f} m —— 情境無效")

# ㊲ C2：當初逼出手寫規則的每一個情境，換成 CBF 一樣要過（門檻和 rules 版相同）
def cbf_area_ctrl(walls, area):
    def f(p, yaw, tg):
        return follow_world(p, yaw, tg, walls=walls, area=area, **GC)[:2]
    return f


A37 = area_of(TRACK_POCKET_M)
run = simulate(0.0, 8.5, 0.0, fixed, 20.0, cbf_area_ctrl(RAW, A37))           # ⑳
c37_20 = min(body_clear(x, y, yw, RAW) for x, y, yw, _ in run)
d37_20 = math.dist(run[-1][:2], run[-1][3])
if c37_20 <= 0.05 or d37_20 > GW["standoff"] + 0.6:
    fails.append(f"㊲⑳ CBF 繞貨架角：最近 {c37_20:.2f} m，最後離目標 {d37_20:.2f} m")
run = simulate(0.0, 6.0, -math.pi / 2, tgt, 40.0, cbf_area_ctrl(RAW, A37))     # ㉑
s37 = min(math.dist((x, y), g) for x, y, _, g in run)
y37 = max(y for _, y, _, _ in run)
c37_21 = min(body_clear(x, y, yw, RAW) for x, y, yw, _ in run)
d37_21 = math.dist(run[-1][:2], run[-1][3])
if s37 < 0.8 or y37 < 9.8 or c37_21 <= 0.05 or d37_21 > GW["standoff"] + 1.0:
    fails.append(f"㊲㉑ CBF 對頭讓路：和目標最近 {s37:.2f}、最北 {y37:.2f}、"
                 f"離牆 {c37_21:.2f}、40 秒時離目標 {d37_21:.2f}")
run = simulate(-1.22, -7.05, h0, tgt2, 25.0, cbf_area_ctrl(RAW1, A37))       # ㉔
c37_24 = min(body_clear(x, y, yw, RAW1) for x, y, yw, _ in run)
d37_24 = math.dist(run[-1][:2], run[-1][3])
if c37_24 <= 0.05 or d37_24 > GW["standoff"] + 1.0:
    fails.append(f"㊲㉔ CBF 貨架角旁脫出：最近 {c37_24:.2f} m，25 秒時離目標 {d37_24:.2f} m")
for sd, (wl, bl, cl, de) in zip((1, 2, 3), r31c):                            # ㉛
    if cl < 1.0 or de > 5.0:
        fails.append(f"㊲㉛ CBF 轉角（種子 {sd}）：離貨架最近 {cl:.2f} m，最後離目標 {de:.1f} m")

# ㊳ 2026-11-17 SITL 抓到的：讓路後車在貨架角旁（中心離角 0.78 m，比邊界 0.8 近 2 cm），
#    目標從車旁 1.1 m 經過往北走。「離貨架遠一點」和「不准再靠近目標」衝突 → 無解，
#    舊的脫離模式挑「動作最小」＝不轉頭，車頭朝 −24° 停了 6 秒，目標走出畫面、跟丟。
#    要求：3 秒內車頭轉到目標在相機視野內（±35°），15 秒後跟上，全程不碰貨架。
def cbf_avoid_ctrl(walls):
    def f(p, yaw, tg):
        return follow_world(p, yaw, tg, walls=walls, avoid=tg, **GC)[:2]
    return f


tgt38 = lambda t: target_on_loop(1.9, t)                             # noqa: E731  (0,-7.1)


def blind_run(run):
    """目標在相機視野（±35°、8 m 內）外最長連續多久。"""
    best = cur = 0.0
    for x, y, yw, g in run:
        out = (abs(wrap_pi(math.atan2(g[1] - y, g[0] - x) - yw)) > 0.6
               or math.dist((x, y), g) > 8.0)
        cur = cur + 0.05 if out else 0.0
        best = max(best, cur)
    return best


run = simulate(-1.21, -7.06, math.radians(-24), tgt38, 15.0, cbf_avoid_ctrl(RAW1))
b38 = blind_run(run)
c38 = min(body_clear(x, y, yw, RAW1) for x, y, yw, _ in run)
d38 = math.dist(run[-1][:2], run[-1][3])
if b38 > 1.5:
    fails.append(f"㊳ 目標從旁經過後，在視野外連續 {b38:.1f} 秒（應該轉頭跟著，≤ 1.5）")
if c38 <= 0.0 or d38 > GW["standoff"] + 1.0:
    fails.append(f"㊳ 貨架角旁跟上目標：車身最近 {c38:.2f} m，15 秒後離目標 {d38:.2f} m")

#    對照組：把濾波器換回修正前（無解時不放掉「不撞目標」、脫離時挑動作最小）
import husky_api_node as HA                                                 # noqa: E402


def _old_filter(p, yaw, v0, w0, walls=(), area=None, avoid=None, max_fwd=1.0, max_back=0.7,
                max_yawrate=1.0, alpha=2.0, margin=0.2, w_yaw=0.2, body=None, fov=None):
    cons = HA.cbf_constraints(p, yaw, walls, area, avoid, alpha, margin, body)
    v, w, okk = HA.cbf_solve(v0, w0, cons, max_fwd, max_back, max_yawrate, w_yaw)
    if okk:
        return v, w, None
    best = None
    for i in range(21):
        vv = -max_back + (max_fwd + max_back) * i / 20
        for j in range(21):
            ww = -max_yawrate + 2 * max_yawrate * j / 20
            worst = min((av * vv + aw * ww - bb) / max(math.hypot(av, aw), 1e-9)
                        for av, aw, bb, _ in cons)
            key = (round(worst, 6), -abs(ww), -abs(vv))
            if best is None or key > best[0]:
                best = (key, vv, ww)
    return best[1], best[2], "close"


_keep = HA.cbf_filter
HA.cbf_filter = _old_filter
try:
    run = simulate(-1.21, -7.06, math.radians(-24), tgt38, 15.0, cbf_avoid_ctrl(RAW1))
finally:
    HA.cbf_filter = _keep
b38_old = blind_run(run)
if b38_old <= 1.5:
    fails.append(f"㊳ 對照組（修正前）應該讓目標離開視野很久，結果只有 {b38_old:.1f} 秒 —— "
                 "這個情境沒有重現 SITL 的問題，測試本身無效")

# =============================================================================
#  C4 視野限制（2026-11-17）：目標要留在相機畫面內（±25°）
# =============================================================================
from husky_api_node import fov_constraint, CBF_FOV_MAX_RAD, CBF_FOV_ALPHA  # noqa: E402

# ㊴ 推導驗證：限制式算出的 ḣ ＝ 往前推一小步實際量到的 h 變化
#    （車照獨輪車模型走、目標照自己的速度走）。式子寫錯正負號或漏一項，這裡一定抓得到。
rnd = random.Random(39)
bad39 = 0
for _ in range(300):
    p = (rnd.uniform(-5, 5), rnd.uniform(-5, 5))
    yaw = rnd.uniform(-math.pi, math.pi)
    ang = yaw + rnd.uniform(-1.4, 1.4)
    rr = rnd.uniform(1.0, 8.0)
    g = (p[0] + rr * math.cos(ang), p[1] + rr * math.sin(ang))
    vt = (rnd.uniform(-1, 1), rnd.uniform(-1, 1))
    v, w = rnd.uniform(-0.7, 1.0), rnd.uniform(-1.0, 1.0)
    av, aw, b = fov_constraint(p, yaw, g, vt)

    def hof(p_, yaw_, g_):
        return math.cos(wrap_pi(math.atan2(g_[1] - p_[1], g_[0] - p_[0]) - yaw_)) \
            - math.cos(CBF_FOV_MAX_RAD)
    h0 = hof(p, yaw, g)
    dt = 1e-6
    h1 = hof((p[0] + v * math.cos(yaw) * dt, p[1] + v * math.sin(yaw) * dt), yaw + w * dt,
             (g[0] + vt[0] * dt, g[1] + vt[1] * dt))
    num = (h1 - h0) / dt
    pred = av * v + aw * w - (b + CBF_FOV_ALPHA * h0)
    if abs(num - pred) > 1e-4:
        bad39 += 1
if bad39:
    fails.append(f"㊴ 視野限制的 ḣ 和數值微分對不上：300 個裡 {bad39} 個")

# ㊵ 目標在正前方、什麼都沒擋到 → 加了視野限制，指令完全不變
v1 = follow_world((0, 0), 0.0, (6, 0), walls=[], **GC)
v2 = follow_world((0, 0), 0.0, (6, 0), walls=[], fov=((6, 0), (0.0, 0.5)), **GC)
if abs(v1[0] - v2[0]) > 1e-9 or abs(v1[1] - v2[1]) > 1e-9:
    fails.append(f"㊵ 目標在正前方時視野限制不該介入：{v1[:2]} → {v2[:2]}")

# ㊶ 目標在左前方 22°、正往左橫著走 0.5 m/s（轉角的情況）→ 車要更用力往左轉
g41 = (4 * math.cos(math.radians(22)), 4 * math.sin(math.radians(22)))
w_no = follow_world((0, 0), 0.0, g41, walls=[], aim=(1.5, 0.0), gap=4.0, **GC)[1]
w_fov = follow_world((0, 0), 0.0, g41, walls=[], aim=(1.5, 0.0), gap=4.0,
                     fov=(g41, (0.0, 0.5)), **GC)[1]
#    CBF 只做「剛好夠」的修正：ω 要大到讓 ḣ ≥ −α·h（目標不再往畫面外跑得太快）
v41 = follow_world((0, 0), 0.0, g41, walls=[], aim=(1.5, 0.0), gap=4.0,
                   fov=(g41, (0.0, 0.5)), **GC)[0]
a1, a2, b41 = fov_constraint((0, 0), 0.0, g41, (0.0, 0.5))
if not (w_fov > w_no + 0.05 and a1 * v41 + a2 * w_fov >= b41 - 1e-6):
    fails.append(f"㊶ 目標快出畫面時應該提早往它那邊轉：沒限制 ω={w_no:.2f}，有限制 ω={w_fov:.2f}")

# ㊷ 和安全衝突時視野讓步：㉓ 的位置（中心離貨架角 0.52 m，禁止轉頭），
#    目標在右邊 30° → 視野想要轉頭，但轉頭會掃到貨架 → ω 必須是 0
p42, yaw42 = (-1.49, -7.06), math.radians(49)
g42 = (p42[0] + 4 * math.cos(yaw42 - 0.5), p42[1] + 4 * math.sin(yaw42 - 0.5))
v42, w42, _ = follow_world(p42, yaw42, g42, walls=RAW1, fov=(g42, (0.3, 0.0)), **GC)
if abs(w42) > 1e-9:
    fails.append(f"㊷ 貨架角旁不准轉頭，視野限制不能蓋過它：ω={w42:.3f}")

# ㊸ 速度估計：目標 0.5 m/s 往北、位置加 0.1 m 雜訊並先平滑（和節點一樣），
#    3 秒後估出來的速度誤差 < 0.15 m/s
rnd = random.Random(43)
vst, filt, t = {}, None, 0.0
errs43 = []
for i in range(60):
    t = i * 0.1
    meas = (rnd.gauss(0, 0.1), 0.5 * t + rnd.gauss(0, 0.1))
    a = 1 - math.exp(-0.1 / 0.3)
    filt = meas if filt is None else (filt[0] + a * (meas[0] - filt[0]), filt[1] + a * (meas[1] - filt[1]))
    vel = vel_update(vst, filt, t)
    if t >= 4.0:
        errs43.append(math.hypot(vel[0], vel[1] - 0.5))
#    單一瞬間會跳（位置雜訊差分後約 ±0.18 m/s），看最後 2 秒的平均誤差
err43 = sum(errs43) / len(errs43)
if err43 > 0.15:
    fails.append(f"㊸ 目標速度估計最後 2 秒平均誤差 {err43:.2f} m/s（實際 (0, 0.5)）")

# ㊹ C4-2 轉角對照：同一個轉角模擬，CBF vs CBF＋視野。看不到的秒數要明顯變少，
#    而且一樣不碰貨架、一樣跟得上
JERK[0] = 0.0
_ = [corner_run(True, sd, safety="cbf") for sd in (1, 2, 3)]
jerk_c = JERK[0]
JERK[0] = 0.0
r31f = [corner_run(True, sd, safety="cbf", fov=True) for sd in (1, 2, 3)]
jerk_f = JERK[0]
if jerk_f > jerk_c * 2.0:
    fails.append(f"㊹ 加視野限制後車頭轉速變化量變成 {jerk_f / jerk_c:.1f} 倍（速度估計的雜訊讓車頭抖）")
for sd, (wl, bl, cl, de), (_wl0, bl0, _cl0, _de0) in zip((1, 2, 3), r31f, r31c):
    if cl < 0.3 or de > 5.0:
        fails.append(f"㊹ 加視野限制後（種子 {sd}）：離貨架最近 {cl:.2f} m，最後離目標 {de:.1f} m")
blind_c = sum(b for _, b, _, _ in r31c)
blind_f = sum(b for _, b, _, _ in r31f)
if blind_f > blind_c * 0.8:
    fails.append(f"㊹ 視野限制沒有讓看不到的時間明顯變少：{blind_c:.1f} → {blind_f:.1f} 秒")

print(f"   ㊴ 視野限制推導 vs 數值微分：300 個裡 {bad39} 個不合　㊸ 速度估計誤差 {err43:.2f} m/s")
print(f"   ㊶ 目標快出畫面：ω {w_no:.2f} → {w_fov:.2f}　"
      f"㊹ 車頭轉速變化量 {jerk_c:.1f} → {jerk_f:.1f} rad/s（3 個種子加總）")
print("   ㊹ 轉角 看不到秒數／離貨架最近／最後距離：")
print("      CBF      " + "  ".join(f"{b:.1f}/{c:.2f}/{d:.1f}" for _, b, c, d in r31c))
print("      CBF＋視野 " + "  ".join(f"{b:.1f}/{c:.2f}/{d:.1f}" for _, b, c, d in r31f))
print(f"   ㊳ SITL 卡住那一刻：目標在視野外最長 {b38:.1f} 秒（修正前 {b38_old:.1f} 秒），"
      f"車身最近 {c38:.2f} m，15 秒後 {d38:.2f} m")
print(f"   ㊲ CBF 重跑舊情境：⑳ 最近 {c37_20:.2f} m／站位 {d37_20:.2f} m　"
      f"㉑ 最北 {y37:.2f}、和目標最近 {s37:.2f} m　㉔ 最近 {c37_24:.2f} m／{d37_24:.2f} m")
print("      ㉛ CBF " + "  ".join(f"{a:.1f}/{b:.1f}/{c:.2f}/{d:.1f}" for a, b, c, d in r31c))
print(f"   ㉝ 求解器 vs 暴力搜尋：{n33} 個情境，{bad33} 個不是最佳")
print(f"   ㉟ 隨機 150 次：車身離貨架/牆最近 {c35:.3f} m")
print(f"   ㊱ 貨架角旁轉頭：CBF 最近 {c36:.3f} m（不擋時 {c36_raw:.2f} m）")
print(f"   ⑳ 繞貨架角：新 最近 {c_new:.2f} m／舊 {c_old:.2f} m（負的＝撞進去）")
print(f"   ㉔ 貨架角旁脫出：最近 {c24:.2f} m（拿掉「不准轉頭」時 {c24_nr:.2f} m），"
      f"25 秒時離目標 {d24:.2f} m")
print(f"   ㉖ 跟 0.5 m/s 目標：停／走切換 新 {n26} 次／舊 {o26} 次，平均距離 {g26:.2f} m")
print("   ㉛ 轉角（3 個種子）擋牆秒數／看不到秒數／離貨架最近／最後距離：")
print("      新 " + "  ".join(f"{a:.1f}/{b:.1f}/{c:.2f}/{d:.1f}" for a, b, c, d in r31n))
print("      舊 " + "  ".join(f"{a:.1f}/{b:.1f}/{c:.2f}/{d:.1f}" for a, b, c, d in r31o))
print(f"   ㉑ 對頭讓路：最北退到 y={y_max:.2f}，和目標最近 {sep_min:.2f} m"
      f"（不准退出環線時 {sep0:.2f} m），車身離牆最近 {clr:.2f} m，"
      f"40 秒時離目標 {d_end:.2f} m")

if fails:
    print("✗ 站位點幾何沒過：")
    for f in fails:
        print("   " + f)
    sys.exit(1)
print("✓ 站位幾何／環線方向／跟隨控制律／擋牆與讓路全部通過")
