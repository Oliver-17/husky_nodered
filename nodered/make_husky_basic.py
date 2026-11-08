#!/usr/bin/env python3
# =============================================================================
#  make_husky_basic.py — 產生地面車（Husky）的 Node-RED 流程
#
#  用法：
#      python3 ~/ros2_ws/src/husky_nodered/nodered/make_husky_basic.py
#      python3 ~/ros2_ws/src/husky_nodered/nodered/make_husky_basic.py --force
#
#  產出：nodered/flows/husky_basic.json
#  ⚠️ 這支**不會**自動併進正在跑的 Node-RED。要部署是另一個動作：
#         python3 ~/ros2_ws/src/drone_nodered/nodered/merge_flows.py
#     合併前後都跑一次 drone_nodered/test/t_merge_keep.py。
#
#  ── 為什麼可以共用無人機那一套 ───────────────────────────────────────────
#  這支刻意「借用」drone_nodered 的產生器模組：
#      load_subflows()  nodered_ros2 的兩個通用積木（呼叫服務／訂閱話題）
#      CALL / SUB       那兩個積木的 subflow id
#      UI_PAGE          儀表板頁面的 id（地面車的分區掛在同一頁上）
#      WAIT_CODE        「等任務完成」的判定邏輯
#  只有 WAIT_CODE 需要改一個字：它讀的全域變數從 drone_status 換成
#  husky_status。這正是整件事要證明的 —— 換載具不用改積木框架，
#  nodered_ros2 一行都沒動。
#
#  ── 積木識別碼是對外介面 ─────────────────────────────────────────────────
#  使用者把積木拖到自己的分頁後，那個實例是用 id 指向定義的。id 一改，
#  他分頁上的積木就變成壞節點。所以下面有一張凍結表，而且**不能和無人機
#  的 d0c0ffeeac* 撞號**（兩套流程會併在同一個 Node-RED 裡）。
# =============================================================================
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DRONE_NODERED = os.path.join(HERE, "..", "..", "drone_nodered", "nodered")
sys.path.insert(0, DRONE_NODERED)
import make_drone_basic as M                                     # noqa: E402

OUT = os.path.join(HERE, "flows", "husky_basic.json")
DRONE_FLOW = os.path.join(DRONE_NODERED, "flows", "drone_basic.json")

NS = "husky"
CATEGORY = "地面車"                      # 左側面板的分類（無人機那組是「無人機」）
TAB = "d0c0ffeeb0ffff01"                 # 分頁：地面車
GROUP = "d0c0ffeeb0grp001"               # 右欄：地面車的狀態
GROUP_CMD = "d0c0ffeeb0grp002"           # 「飛行」下面那一塊：地面車的指令
MAP_LINK_IN = "d0c0ffeelink0002"         # 無人機儀表板上的 link in（地圖資料）

# ⚠️ 這個標記決定了「更新流程時哪些東西要換掉」。merge_flows.py 用
#    generated-by: <套件>_nodered 的 regex 比對 —— 沒有標記的話，這個分頁
#    會被當成使用者自建的而永遠不更新；標記寫成 drone_nodered 則是說謊。
MARK = "generated-by: husky_nodered\n\n"


def hid(tag, n=0):
    """積木（subflow）的 id。b? 是地面車的家族，不會和無人機的 a? 撞號。"""
    return f"d0c0ffeeb0{tag}{n:04x}"


def nid(i):
    """一般節點的 id。前綴 b1 區隔，避免和 drone_basic.json 的 d0c0ffee???????? 撞。"""
    return f"d0c0ffeeb1{i:06x}"


FROZEN_IDS = {
    hid("b1"): "api/goto",
    hid("b2"): "api/stop",
    hid("b3"): "api/state",
    hid("b9"): "返航（組合積木，沒有對應服務）",
}

# 等待完成的判定邏輯直接沿用無人機的，只換全域變數的名字。
# ⚠️ 不要把這段複製一份改 —— 兩邊的判定必須一模一樣，否則同一個積木
#    框架服務兩種載具這件事就不成立了（t_status_contract.py 在守這條）。
WAIT_CODE = M.WAIT_CODE.replace("drone_status", "husky_status")

# 動作積木：(tag, 顯示名, 服務, 逾時秒, env 欄位, 組參數的 JS)
ACTIONS = [
    ("b1", "車：導航到座標", "goto", 180,
     [("MAP_X", "num", "0.0", "目標 x（map 座標，東為正）"),
      ("MAP_Y", "num", "0.0", "目標 y（map 座標，北為正）"),
      ("YAW", "num", "999", "終點車頭朝向（弧度；999 = 不指定）")],
     'msg.payload = { map_x: Number(env.get("MAP_X")),\n'
     '                map_y: Number(env.get("MAP_Y")),\n'
     '                yaw: Number(env.get("YAW")), timeout: 0 };'),
    ("b2", "車：停止", "stop", 10, [], 'msg.payload = {};'),
    ("b3", "車：狀態查詢", "state", 10, [], 'msg.payload = {};'),
]


def check_frozen():
    got = {hid(t): a for t, _, a, _, _, _ in ACTIONS}
    for i, a in got.items():
        assert i in FROZEN_IDS, f"積木 id {i}（{a}）不在凍結表裡"
    assert not (set(FROZEN_IDS) & {f"d0c0ffeeac{t}0000" for t in
                                   ("a1", "a2", "a3", "a4", "a5", "a6", "a7", "a8", "a9")}), \
        "地面車的積木 id 和無人機撞號了"


# =============================================================================
#  動作積木
# =============================================================================
def action_subflows():
    out = []
    for tag, label, action, timeout, envs, build in ACTIONS:
        sid, fn_in, inst, fn_wait = hid(tag), hid(tag, 1), hid(tag, 2), hid(tag, 3)
        env_def = [{"name": n, "type": t, "value": v,
                    "ui": {"label": {"zh-TW": z}, "type": "input",
                           "opts": {"types": [t]}}} for n, t, v, z in envs]
        env_def.append({"name": "TIMEOUT", "type": "num", "value": str(timeout),
                        "ui": {"label": {"zh-TW": "逾時（秒）"}, "type": "input",
                               "opts": {"types": ["num"]}}})
        out += [
            {"id": sid, "type": "subflow", "name": label,
             "info": (MARK + f"送出「{label}」給地面車，並等它真的做完。\n\n"
                      "上面出口＝完成、下面出口＝失敗（逾時或被拒絕）。\n"
                      f"對應服務 /{NS}/api/{action}。"),
             "category": CATEGORY, "color": "#E6C8A0",
             "in": [{"x": 40, "y": 80, "wires": [{"id": fn_in}]}],
             "out": [{"x": 560, "y": 40, "wires": [{"id": fn_wait, "port": 0}]},
                     {"x": 560, "y": 120, "wires": [{"id": fn_wait, "port": 1}]}],
             "outputLabels": ["完成", "失敗"], "env": env_def},
            {"id": fn_in, "type": "function", "z": sid, "name": "組參數",
             "func": ("const incoming = (msg.payload && typeof msg.payload === 'object')\n"
                      "    ? msg.payload : {};\n"
                      + build +
                      "\n// 傳進來的值可以覆蓋，但只限這個動作本來就有的欄位\n"
                      "for (const k of Object.keys(msg.payload)) {\n"
                      "    if (k in incoming) { msg.payload[k] = incoming[k]; }\n"
                      "}\n"
                      "// 記下送指令前的任務編號，等待時才分得出新舊任務\n"
                      'msg._seq_before = ((global.get("husky_status") || {}).task || {}).seq;\n'
                      "return msg;"),
             "outputs": 1, "timeout": 0, "noerr": 0, "initialize": "", "finalize": "",
             "libs": [], "x": 160, "y": 80, "wires": [[inst]]},
            {"id": inst, "type": f"subflow:{M.CALL}", "z": sid,
             "name": f"/{NS}/api/{action}",
             "env": [{"name": "SERVICE", "type": "str", "value": f"/{NS}/api/{action}"},
                     {"name": "TIMEOUT", "type": "num", "value": "10"}],
             "x": 320, "y": 80, "wires": [[fn_wait]]},
            {"id": fn_wait, "type": "function", "z": sid, "name": "等任務完成",
             "func": WAIT_CODE, "outputs": 2, "timeout": 0, "noerr": 0,
             "initialize": "", "finalize": "", "libs": [], "x": 460, "y": 80,
             "wires": [[], []]},
        ]
    return out


def return_home_subflow():
    """返航＝從狀態讀回家座標，再呼叫「導航到座標」。沒有對應的服務。

    地面車的返航比無人機少一步：不用降落。
    規格書 P-2「飛控端只提供單一動作模組、組合留在編排層」在地面車照樣成立。
    """
    sid, fetch, goto = hid("b9"), hid("b9", 1), hid("b9", 2)
    code = (
        '// 回家＝車子的 spawn 位置，由 husky_api_node 從 warehouse_geometry.yaml\n'
        '// 的 husky_spawn 讀出來放進狀態的 scene.home。\n'
        '// ⚠️ 不要退回 (0,0) —— map 的原點是倉庫正中央，不是車子的家。\n'
        '//    拿不到就走失敗出口，寧可不動也不要開到別的地方。\n'
        'const s = global.get("husky_status") || {};\n'
        'const home = (s.scene || {}).home;\n'
        'if (!Array.isArray(home) || typeof home[0] !== "number"\n'
        '        || typeof home[1] !== "number") {\n'
        '    node.status({ fill: "red", shape: "ring", text: "不知道家在哪" });\n'
        '    return [null, { payload: { 失敗: "不知道家在哪：api/status 裡沒有 '
        'scene.home" } }];\n'
        '}\n'
        '// 只送「導航到座標」本來就有的欄位 —— 整包 assign 會塞進服務不認得的欄位\n'
        'return [{ payload: { map_x: home[0], map_y: home[1], yaw: 999 } }, null];')
    return [
        {"id": sid, "type": "subflow", "name": "車：返航",
         "info": (MARK + "把車開回出發點。\n\n"
                  "**沒有對應的服務** —— 它就是「讀 scene.home ＋ 導航到座標」。\n"
                  "家的座標來自 api/status 的 scene.home（husky_api_node 從\n"
                  "warehouse_geometry.yaml 的 husky_spawn 讀的）。"),
         "category": CATEGORY, "color": "#E6C8A0",
         "in": [{"x": 40, "y": 80, "wires": [{"id": fetch}]}],
         "out": [{"x": 620, "y": 40, "wires": [{"id": goto, "port": 0}]},
                 {"x": 620, "y": 140, "wires": [{"id": fetch, "port": 1},
                                                {"id": goto, "port": 1}]}],
         "outputLabels": ["完成", "失敗"], "env": []},
        {"id": fetch, "type": "function", "z": sid, "name": "取回家座標",
         "func": code, "outputs": 2, "timeout": 0, "noerr": 0, "initialize": "",
         "finalize": "", "libs": [], "x": 180, "y": 80, "wires": [[goto], []]},
        {"id": goto, "type": f"subflow:{hid('b1')}", "z": sid, "name": "導航回家",
         # 逾時寫死：巢狀 subflow 的 env 參照行為不好掌握，填錯會變成永不逾時
         # （無人機的返航積木踩過同一個坑，見 make_drone_basic.py）。
         "env": [{"name": "TIMEOUT", "type": "num", "value": "240"}],
         "x": 420, "y": 80, "wires": [[], []]},
    ]


# =============================================================================
#  分頁：訂閱狀態 + 儀表板
# =============================================================================
STATUS_CODE = r"""// 收到 /husky/api/status（JSON 字串）→ 存進全域，再拆成畫面要顯示的東西
//
// 全域 husky_status 是給積木用的：每個動作積木的「等任務完成」都讀它的
// task.seq / task.state。無人機那邊同一個位置叫 drone_status。
let s;
try { s = JSON.parse(msg.payload.data); } catch (e) { return null; }
global.set("husky_status", s);

// --- 地圖 payload：欄位名和無人機完全一樣，所以可以直接餵同一個 widget ---
// ⚠️ 不要在這裡塞 tx/ty（視覺目標）或 loop（搜尋環線）：地面車沒有感知鏈，
//    塞了反而會在地圖上畫出不存在的東西。
// 兩台載具畫在同一張地圖上（2026-11-07）。那張 widget 在無人機的儀表板
// 分頁上，而 Node-RED 的線不能跨分頁，所以：
//   ① 各自把自己那半存進全域（無人機 map_air、地面車 map_ugv）
//   ② 送出去時合併成完整的一份，再用 link 送到那張圖
// ⚠️ 不能只送自己那半：widget 的 watch 每次都會重設兩台的位置，
//    只送一半會把另一台從地圖上抹掉。
const ugv = {};
if (typeof s.map_x === "number" && typeof s.map_y === "number") {
    ugv.ux = s.map_x;
    ugv.uy = s.map_y;
    ugv.uh = s.map_heading_deg || 0;
}
global.set("map_ugv", ugv);
const 地圖 = Object.assign({ boxes: (s.scene && s.scene.boxes) || [] },
                          global.get("map_air") || {}, ugv);

// --- 就緒燈 ---------------------------------------------------------------
// ⚠️ 不能沿用無人機那段：它看的是 ready.px4 / ready.position，而地面車是
//    ready.platform / ready.odom，沿用的話會永遠顯示「未就緒（等 PX4）」。
//    地圖可以共用，這一段不行（t_map_widget_husky.js 把這個差異釘住了）。
const r = s.ready || {};
const 缺 = [];
if (!r.platform) { 缺.push("平台"); }
if (!r.odom) { 缺.push("位置估計"); }
if (!r.nav2) { 缺.push("導航"); }
if (!r.bridge) { 缺.push("速度看門狗"); }
const 就緒 = 缺.length ? ("⏳ 未就緒（等 " + 缺.join("、") + "）") : "✅ 就緒";
// 系統狀態整合在中欄那一行（無人機的 adapter 負責組合），所以這裡不自己顯示，
// 只把自己那半寫進全域。⚠️ 兩邊的 key 要對上：無人機讀的是 sys_ugv。
global.set("sys_ugv", 就緒);

// --- 位置與動作 -----------------------------------------------------------
// 格式和無人機那一欄一致（兩欄左右對照才看得出差異）。
// ⚠️ 不放「就緒」—— 中間那一欄的系統狀態已經整合了兩台，放兩次會讓人
//    懷疑哪一個才算數。高度只有無人機有，所以這裡少那一段。
const 位置 = (typeof s.map_x === "number")
    ? `map (${s.map_x.toFixed(2)}, ${s.map_y.toFixed(2)})　朝向 ${
        Math.round(s.map_heading_deg || 0)}°　${(s.speed_mps || 0).toFixed(2)} m/s`
    : "位置未知";
const t = s.task || {};
let 動作 = "待命";
if (t.state === "running") {
    const n = s.nav || {};
    動作 = (n.state === "running" && typeof n.distance_remaining === "number")
        ? `${t.name}　剩 ${n.distance_remaining.toFixed(1)} m`
        : (t.name || "執行中");
} else if (t.state === "done") {
    動作 = `✅ ${t.name}：${t.message || "完成"}`;
} else if (t.state === "failed") {
    動作 = `❌ ${t.name}：${t.message || "失敗"}`;
}
// 服務當場被拒絕時不會開任務（task 還是 idle），所以上面那幾條都走不到。
// 把 last_message 接進來，否則使用者按了按鈕只會看到「待命」，
// 完全看不出被拒絕的理由（2026-11-07 的症狀）。
// ⚠️ 這就是為什麼不需要另一行「最近回覆」—— 一行講完。
if (t.state === "idle" && s.last_message) { 動作 = s.last_message; }

return [{ payload: Number(s.speed_mps) || 0 }, { payload: 位置 },
        { payload: 動作 }, { payload: 地圖 }];
"""


def ui_groups():
    """兩個分區，位置和無人機那邊對稱：

        第一列  無人機(order1,w4) │ 地圖(order2,w4) │ 地面車狀態(order3,w4)
        第二列  飛行(order4,w6)   │ 任務(order5,w6)
        第三列  地面車指令(order6,w6) │ 模擬(order7,w6)
        第四列  緊急(order8,w6)

    ⚠️ order 和 width 要和 drone_basic.json 裡那四個分區一起看 ——
       12 欄排滿就折行，改一邊會讓另一邊跑位。
    """
    def g(gid, name, order, width, height=1):
        return {"id": gid, "type": "ui-group", "name": name, "page": M.UI_PAGE,
                "width": width, "height": height, "order": order,
                "showTitle": True, "className": "", "visible": True,
                "disabled": False, "groupType": "default"}
    # ⚠️ 2026-11-08：分區的 height **沒有作用** —— Dashboard 2.0 的分區高度
    #    是由內容撐出來的（設 9 之後三欄還是不齊）。要等高得調**元件**的高度，
    #    見下面 ui_gauge / ui_text 的 height。三欄的元件高度要一起看：
    #        無人機   儀表 6 ＋ 文字 1 ＋ 文字 1
    #        地圖     地圖 7 ＋ 文字 1
    #        地面車   儀表 6 ＋ 文字 1 ＋ 文字 1
    return [g(GROUP, "地面車（Husky）", 3, 4),
            g(GROUP_CMD, "地面車指令", 6, 6)]


def ui_text(i, label, order, y, x=900, layout="row-spread", name=None):
    return {"id": nid(i), "type": "ui-text", "z": TAB, "group": GROUP, "order": order,
            "width": 0, "height": 1, "name": name or label, "label": label,
            "format": "{{msg.payload}}", "layout": layout, "style": False,
            "font": "", "fontSize": 16, "color": "#717171", "wrapText": False,
            "className": "", "x": x, "y": y, "wires": []}


def ui_button(i, label, order, target, y, x=320, bg="", group=None):
    return {"id": nid(i), "type": "ui-button", "z": TAB, "group": group or GROUP_CMD, "name": label,
            "label": label, "order": order, "width": 0, "height": 0,
            "emulateClick": False, "tooltip": "", "color": "", "bgcolor": bg,
            "className": "", "icon": "", "iconPosition": "left",
            "payload": "", "payloadType": "str", "topic": label, "topicType": "str",
            "buttonColor": bg, "textColor": "", "iconColor": "",
            "enableClick": True, "enablePointerdown": False, "pointerdownPayload": "",
            "pointerdownPayloadType": "str", "enablePointerup": False,
            "pointerupPayload": "", "pointerupPayloadType": "str",
            "x": x, "y": y, "wires": [[target] if target else []]}


def ui_number(i, label, order, topic, y, x=180, target=None, lo=-18.0, hi=18.0):
    return {"id": nid(i), "type": "ui-number-input", "z": TAB, "group": GROUP_CMD,
            "name": label, "label": label, "order": order, "width": 0, "height": 0,
            "topic": topic, "topicType": "str", "min": lo, "max": hi, "step": 0.1,
            "tooltip": "", "delay": 300, "passthru": True, "sendOnDelay": True,
            "sendOnBlur": True, "sendOnEnter": True, "className": "",
            "clearable": False, "icon": "", "iconPosition": "left",
            "iconInnerPosition": "inside", "spinner": True,
            "x": x, "y": y, "wires": [[target] if target else []]}


def ui_gauge(i, label, order, y, x=760):
    """速度儀表。上限照 A200 的 1.0 m/s —— 和無人機那支（高度 0~6 m）不同，
    所以不能直接共用 make_drone_basic 的 ui_gauge。"""
    return {"id": nid(i), "type": "ui-gauge", "z": TAB, "name": label, "group": GROUP,
            "order": order, "value": "payload", "valueType": "msg",
            # height 6：和無人機那兩個儀表一樣高，三欄才會切齊
            "width": 0, "height": 6, "gtype": "gauge-half", "gstyle": "needle",
            "title": label, "alwaysShowTitle": True, "floatingTitlePosition": "inside",
            "units": "m/s", "icon": "", "prefix": "", "suffix": "",
            "segments": [{"from": 0, "color": "#5cd65c"}, {"from": 0.8, "color": "#ff8c00"}],
            "min": 0, "max": 1.0, "sizeThickness": 16, "sizeGap": 0,
            "sizeKeyThickness": 8, "styleRounded": True, "styleGlow": False,
            "className": "", "x": x, "y": y, "wires": []}


def map_link():
    """把合併好的地圖資料送到無人機儀表板上那張**唯一**的地圖。

    ⚠️ 2026-11-07 之前這裡是複製一份 widget 過來，變成兩張地圖。
       Oliver 要的是一張圖畫兩台，所以改成 link out。
       對應的 link in 在 drone_basic.json 的儀表板分頁（手工加的，
       id 寫在 MAP_LINK_IN）—— 那個 id 改掉的話這條線會安靜地斷掉。
    """
    return {"id": nid(0x200), "type": "link out", "z": TAB,
            "name": "地圖資料 → 無人機儀表板", "mode": "link",
            "links": [MAP_LINK_IN], "x": 920, "y": 120, "wires": []}


def tab_nodes():
    sub, split = nid(1), nid(2)
    b_goto, b_home, b_stop, b_state = (nid(0x10), nid(0x11), nid(0x12), nid(0x13))
    i_goto, i_home, i_stop, i_state = (nid(0x20), nid(0x21), nid(0x22), nid(0x23))
    out_msg = nid(0x30)
    xy = nid(0x40)
    return [
        {"id": TAB, "type": "tab", "label": "地面車（Husky）", "disabled": False,
         "info": (MARK + "Husky A200 的控制。由 husky_nodered/nodered/make_husky_basic.py 產生。\n\n"
                  "四個積木：導航到座標／停止／狀態查詢／返航（組合）。\n"
                  "服務在 /husky/api/* ，狀態在 /husky/api/status。\n\n"
                  "⚠️ 這個分頁是產生出來的，手改會在下次更新流程時被覆蓋。\n"
                  "要組自己的任務請另開分頁，merge_flows.py 不會動使用者自建的分頁。")},
        {"id": nid(3), "type": "comment", "z": TAB,
         "name": "這個分頁是產生出來的 —— 要改請改 make_husky_basic.py", "info": "",
         "x": 300, "y": 40, "wires": []},

        # --- 狀態訂閱 ---------------------------------------------------------
        {"id": sub, "type": f"subflow:{M.SUB}", "z": TAB, "name": "/husky/api/status",
         "env": [{"name": "TOPIC", "type": "str", "value": f"/{NS}/api/status"},
                 {"name": "TYPE", "type": "str", "value": "std_msgs/msg/String"},
                 {"name": "THROTTLE_MS", "type": "num", "value": "0"}],
         "x": 160, "y": 120, "wires": [[split]]},
        {"id": split, "type": "function", "z": TAB, "name": "存狀態並拆成畫面欄位",
         "func": STATUS_CODE, "outputs": 4, "timeout": 0, "noerr": 0,
         "initialize": "", "finalize": "", "libs": [], "x": 460, "y": 120,
         "wires": [[nid(0x100)], [nid(0x101)], [nid(0x102)], [nid(0x200)]]},
        ui_gauge(0x100, "速度", 1, 60, x=760),
        ui_text(0x101, "狀態", 2, 120, x=760),
        ui_text(0x102, "任務", 3, 180, x=760),
        map_link(),

        # --- 目標座標輸入 -----------------------------------------------------
        ui_number(0x50, "目標 x（東為正）", 1, "map_x", 260, target=xy),
        ui_number(0x51, "目標 y（北為正）", 2, "map_y", 320, target=xy),
        {"id": xy, "type": "function", "z": TAB, "name": "記住目標座標",
         "func": ('// 存進共用區，按「導航到目標」時才拿出來用\n'
                  'const g = global.get("husky_goal") || { map_x: 0, map_y: 0 };\n'
                  'g[msg.topic] = Number(msg.payload);\n'
                  'global.set("husky_goal", g);\nreturn null;'),
         "outputs": 1, "timeout": 0, "noerr": 0, "initialize": "", "finalize": "",
         "libs": [], "x": 460, "y": 290, "wires": [[]]},

        # --- 按鈕 → 積木 ------------------------------------------------------
        ui_button(0x10, "導航到目標", 3, nid(0x15), 380),
        {"id": nid(0x15), "type": "function", "z": TAB, "name": "帶入目標座標",
         "func": ('// 畫面上輸入的座標優先；沒輸入過就用倉庫東南角當示範點\n'
                  'const g = global.get("husky_goal") || {};\n'
                  'msg.payload = { map_x: Number(g.map_x ?? 5.6),\n'
                  '                map_y: Number(g.map_y ?? -9.0), yaw: 999 };\n'
                  'return msg;'),
         "outputs": 1, "timeout": 0, "noerr": 0, "initialize": "", "finalize": "",
         "libs": [], "x": 500, "y": 380, "wires": [[i_goto]]},
        {"id": i_goto, "type": f"subflow:{hid('b1')}", "z": TAB, "name": "導航到座標",
         "env": [], "x": 700, "y": 380, "wires": [[out_msg], [out_msg]]},

        ui_button(0x11, "返航", 4, i_home, 440),
        {"id": i_home, "type": f"subflow:{hid('b9')}", "z": TAB, "name": "返航",
         "env": [], "x": 700, "y": 440, "wires": [[out_msg], [out_msg]]},

        ui_button(0x12, "停止", 5, i_stop, 500, bg="#d93025"),
        {"id": i_stop, "type": f"subflow:{hid('b2')}", "z": TAB, "name": "停止",
         "env": [], "x": 700, "y": 500, "wires": [[out_msg], [out_msg]]},

        ui_button(0x13, "狀態查詢", 6, i_state, 560),
        {"id": i_state, "type": f"subflow:{hid('b3')}", "z": TAB, "name": "狀態查詢",
         "env": [], "x": 700, "y": 560, "wires": [[out_msg], [out_msg]]},

        {"id": out_msg, "type": "debug", "z": TAB, "name": "積木的回覆",
         "active": True, "tosidebar": True, "console": False, "tostatus": False,
         "complete": "payload", "targetType": "msg", "statusVal": "",
         "statusType": "auto", "x": 900, "y": 470, "wires": []},
    ]


def shared_blocks():
    """nodered_ros2 的兩個通用積木：**從無人機那份流程原封不動複製**。

    ⚠️ 不能用 M.load_subflows() 重新產生：無人機的產生器會在 info 裡加上
       generated-by 標記，兩份流程裡的同一個 id 就會有不同內容。兩份要併進
       同一個 Node-RED，同 id 不同內容 = 後載入的蓋掉前一個，而且不會報錯。
       （2026-11-07 install_flows.sh 的合併檢查當場抓到。）
       這兩個積木本來就該是同一份定義 —— 共用才是對的，複製才會分叉。
    """
    with open(DRONE_FLOW, encoding="utf-8") as f:
        drone = json.load(f)
    want = {M.CALL, M.SUB}
    out = [n for n in drone if n["id"] in want or n.get("z") in want]
    got = {n["id"] for n in out if n["type"] == "subflow"}
    assert got == want, f"無人機那份流程裡找不到通用積木：{want - got}"
    return out


def build():
    check_frozen()
    nodes = shared_blocks()
    nodes += action_subflows()
    nodes += return_home_subflow()
    nodes += ui_groups()
    nodes += tab_nodes()
    return nodes


def main():
    ap = argparse.ArgumentParser(description="產生地面車的 Node-RED 流程")
    ap.add_argument("--force", action="store_true",
                    help="覆蓋已存在的 husky_basic.json")
    args = ap.parse_args()
    nodes = build()
    ids = [n["id"] for n in nodes]
    assert len(ids) == len(set(ids)), \
        f"有重複的 id：{[i for i in ids if ids.count(i) > 1][:3]}"
    if os.path.exists(OUT) and not args.force:
        print(f"{OUT} 已經存在。確定要覆蓋就加 --force\n"
              "（如果手改過那個檔案，先確認改動已經搬回這支產生器）")
        return 1
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as f:
        json.dump(nodes, f, ensure_ascii=False, indent=2)
        f.write("\n")
    subs = [n["name"] for n in nodes if n["type"] == "subflow"]
    print(f"寫出 {OUT}")
    print(f"  節點 {len(nodes)} 個，積木 {len(subs)} 個：{'、'.join(subs)}")
    print(f"  儀表板分區「地面車（Husky）」掛在既有的頁面 {M.UI_PAGE}")
    print("\n要裝進 Node-RED：python3 ~/ros2_ws/src/drone_nodered/nodered/merge_flows.py")
    print("⚠️ 新套件 vehicle_nodered_interfaces 要讓 rosbridge 認得 —— "
          "重新 source 並重啟 rosbridge，否則呼叫會失敗（Unable to import）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
