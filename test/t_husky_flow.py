#!/usr/bin/env python3
# =============================================================================
#  t_husky_flow.py — 產生出來的 husky_basic.json 結構對不對
#
#  跑法（不需要 ROS、不需要 Node-RED）：
#      python3 ~/ros2_ws/src/husky_nodered/test/t_husky_flow.py
#
#  為什麼要有這支：
#      流程 JSON 壞掉時 Node-RED 的表現是「安靜地少一塊」—— 指向不存在的
#      subflow 會變成紅色壞節點、接到不存在的節點只是那條線沒作用，
#      兩種都不會讓部署失敗。所以要在產生的當下就檢查。
#
#  特別重要的是 ④：兩套流程會併進**同一個** Node-RED，id 撞號的話後載入的
#  會覆蓋前一個 —— 症狀是「無人機的某個積木突然變成地面車的」。
# =============================================================================
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
FLOW = os.path.join(PKG, "nodered", "flows", "husky_basic.json")
DRONE_FLOW = os.path.join(PKG, os.pardir, "drone_nodered", "nodered", "flows",
                          "drone_basic.json")
MAP_SRC_ID = "d0c0ffee0000020f"
UI_PAGE = "d0c0ffeeui000003"          # 無人機建立的儀表板頁面
fails = []

if not os.path.exists(FLOW):
    print(f"✗ 找不到 {FLOW} —— 先跑 nodered/make_husky_basic.py")
    sys.exit(1)
nodes = json.load(open(FLOW, encoding="utf-8"))
byid = {n["id"]: n for n in nodes}

# ① id 不重複
ids = [n["id"] for n in nodes]
dup = {i for i in ids if ids.count(i) > 1}
if dup:
    fails.append(f"① 有重複的 id：{sorted(dup)[:4]}")

# ② 每個 subflow 實例都指向這個檔案裡真的有的定義
subflow_ids = {n["id"] for n in nodes if n["type"] == "subflow"}
for n in nodes:
    t = n.get("type", "")
    if t.startswith("subflow:") and t.split(":", 1)[1] not in subflow_ids:
        fails.append(f"② {n.get('name')} 指向不存在的積木 {t}")

# ③ 每條線都接到存在的節點
for n in nodes:
    for port in n.get("wires", []) or []:
        for tgt in (port if isinstance(port, list) else [port]):
            if isinstance(tgt, str) and tgt not in byid:
                fails.append(f"③ {n.get('name') or n['id']} 接到不存在的節點 {tgt}")

# ④ 不能和無人機的流程撞 id（兩套會併進同一個 Node-RED）
if os.path.exists(DRONE_FLOW):
    drone = json.load(open(DRONE_FLOW, encoding="utf-8"))
    dids = {n["id"] for n in drone}
    # nodered_ros2 的兩個通用積木是**故意**共用同一個 id（同一份定義），
    # 合併時會對上，不算撞號。
    shared_ok = {"f1a0c0de00000001", "f1a0c0de00000002"}
    shared_ok |= {n["id"] for n in drone
                  if n.get("z") in shared_ok or n.get("id", "").startswith("f1a0c0de")}
    clash = (set(ids) & dids) - shared_ok
    if clash:
        fails.append(f"④ 和無人機的流程撞 id：{sorted(clash)[:4]}")
else:
    fails.append("④ 找不到無人機的流程，沒辦法檢查 id 撞號")

# ⑤ 四個積木都在，而且 id 是凍結的那幾個
# 名稱刻意加「車：」前綴 —— merge_flows.py 有一條「名稱相同就當成同一個東西」
# 的比對（為了清掉匯入時產生的副本），和無人機的積木同名會被誤判。
want = {"d0c0ffeeb0b10000": "車：導航到座標", "d0c0ffeeb0b20000": "車：停止",
        "d0c0ffeeb0b30000": "車：狀態查詢", "d0c0ffeeb0b90000": "車：返航"}
for i, name in want.items():
    n = byid.get(i)
    if not n:
        fails.append(f"⑤ 少了積木 {name}（{i}）")
    elif n.get("name") != name:
        fails.append(f"⑤ {i} 應該是「{name}」，實際是「{n.get('name')}」")
    elif len(n.get("out", [])) != 2:
        fails.append(f"⑤ 積木「{name}」不是兩個出口")

# ⑥ 「等任務完成」必須讀 husky_status，不能讀到無人機的
for n in nodes:
    if n.get("name") == "等任務完成":
        f = n.get("func", "")
        if "husky_status" not in f:
            fails.append("⑥ 等任務完成沒有讀 husky_status")
        if "drone_status" in f:
            fails.append("⑥ 等任務完成讀到無人機的 drone_status（複製時沒改乾淨）")

# ⑦ 只能有一張地圖：地面車不自己畫，改用 link 把資料送到無人機那張
#    （2026-11-07 之前這裡是複製一份 widget，變成兩張圖）
if any(n["type"] == "ui-template" for n in nodes):
    fails.append("⑦ 地面車的流程裡還有 ui-template —— 地圖應該只有一張")
lo = next((n for n in nodes if n["type"] == "link out"), None)
if not lo:
    fails.append("⑦ 沒有 link out —— 地圖資料送不到無人機那張圖")
elif "d0c0ffeelink0002" not in (lo.get("links") or []):
    fails.append(f"⑦ link out 指向 {lo.get('links')}，應該是無人機儀表板的 "
                 "link in（d0c0ffeelink0002）")
elif os.path.exists(DRONE_FLOW):
    li = next((n for n in drone if n["id"] == "d0c0ffeelink0002"), None)
    if not li:
        fails.append("⑦ 無人機的流程裡找不到對應的 link in —— 這條線會安靜地斷掉")
    elif li.get("type") != "link in":
        fails.append(f"⑦ d0c0ffeelink0002 的型別是 {li.get('type')}，不是 link in")
    elif "d0c0ffee0000020f" not in [w for ws in li.get("wires", []) for w in ws]:
        fails.append("⑦ link in 沒有接到地圖 widget")

# ⑧b 每個分頁／積木都要有 generated-by 標記，更新流程時才認得出是產生的
for n in nodes:
    if n["type"] in ("tab", "subflow") and not n["id"].startswith("f1a0c0de"):
        if "generated-by: husky_nodered" not in (n.get("info") or ""):
            fails.append(f"⑧b {n.get('label') or n.get('name')} 沒有 generated-by 標記")

# ⑧ 兩個分區（狀態在右欄、指令在「飛行」下面）都要掛在無人機建立的那一頁上，
#    而且每個元件都要屬於其中之一。
groups = {n["id"]: n for n in nodes if n["type"] == "ui-group"}
if len(groups) != 2:
    fails.append(f"⑧ 應該有兩個分區（狀態、指令），實際 {len(groups)} 個")
for gid, g in groups.items():
    if g.get("page") != UI_PAGE:
        fails.append(f"⑧ 分區「{g.get('name')}」掛錯頁面：{g.get('page')}")
for n in nodes:
    if n.get("type", "").startswith("ui-") and n["type"] != "ui-group":
        if n.get("group") and n["group"] not in groups:
            fails.append(f"⑧ {n.get('name')} 掛在不存在的分區 {n['group']}")
# ⑧c 版面：三欄那一列（無人機 w4 + 地圖 w4 + 地面車 w4）剛好 12 欄，
#     指令那一塊 w6 要和無人機的「飛行」同寬才排得整齊。
st = next((g for g in groups.values() if g["order"] == 3), None)
cmd = next((g for g in groups.values() if g["order"] == 6), None)
if not st or st["width"] != 4:
    fails.append(f"⑧c 狀態分區應該是 order=3、width=4，實際 {st}")
if not cmd or cmd["width"] != 6:
    fails.append(f"⑧c 指令分區應該是 order=6、width=6，實際 {cmd}")

if fails:
    print(f"✗ 流程結構檢查沒過（{len(fails)} 項）：")
    for f in fails:
        print("   " + f)
    sys.exit(1)
n_sub = len(subflow_ids)
print(f"✓ 流程結構檢查通過（{len(nodes)} 個節點、{n_sub} 個積木、"
      "id 不撞號、線都接得到、地圖是複製的、等待完成讀 husky_status）")
