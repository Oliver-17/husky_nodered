# husky_nodered

把 Low-Code 任務編排工具接到**第二種載具**：Clearpath Husky A200（地面無人載具）。
**只做模擬**，不涉及實體車輛。

這個套件存在的目的是驗證規格書 G-4／P-1 的主張：**換載具不用改 Node-RED、
不用改 `nodered_ros2`**。所以操作層、編排層、橋接層完全沿用無人機那一套，
只換掉決策層以下。

## 和無人機的差別（為什麼不是照抄）

| | 無人機 x500 | Husky A200 |
|---|---|---|
| 可控自由度 | 4（前後、左右、上下、機頭各自獨立） | 2（前進 v、轉向 ω） |
| 能不能側移 | 能 | **不能**，差速車，機頭就是移動方向 |
| 速度橋 | 需要 `cmd_vel_to_px4_node` | **不需要**，平台直接吃 `cmd_vel` |
| 起飛／降落／高度 | 有 | 沒有這些概念 |
| 終點朝向 | 不在乎（相機朝下） | 有意義（車頭） |

## 進度

| 階段 | 內容 | 狀態 |
|---|---|---|
| H0 | `ros_gz_bridge`（Harmonic）雙向驗證 | ✅ 2026-11-07 |
| H1 | Husky 進倉庫場景、`cmd_vel` 開得動 | ✅ 2026-11-07 |
| H2 | Nav2 給差速車（footprint、RotationShim、滑移修正、真值里程計） | ✅ 2026-11-07 |
| H3 | `husky_api_node`（3 個服務＋狀態話題）、`vehicle_nodered_interfaces` | ✅ 2026-11-07 |
| H4 | Node-RED 端四個積木（導航到座標、停止、狀態查詢、返航） | ✅ 2026-11-07 |
| H5 | 和無人機同場共存 | ✅ 2026-11-07（模擬） |

⬜ 還沒做：儀表板的「開模擬」按鈕仍然只起無人機那半（`drone-warehouse.service`
走的是 `warehouse_all.launch.py`）。要讓按鈕一鍵起兩台，得把那個服務改成
`both_all.launch.py`。

⚠️ H2 原本規劃要讓區域 costmap 吃 `obstacle_layer`，但最後選用的原廠 A200
模型**沒有光達**，所以和無人機一樣只有 `static_layer` —— 看得到地圖上畫的
貨架，看不到臨時出現的障礙物。

## H2 的驗收數據（2026-11-07，**以 Gazebo 真值量測**）

| 目標 | 結果 | 誤差 | 路徑 | 離貨架最近 | 耗時 |
|---|---|---|---|---|---|
| map (−14, 0)　西側南下 | SUCCEEDED | 0.38 m | 8.7 m | 4.36 m | 17 s |
| map (5.6, −9)　要繞過四排貨架 | SUCCEEDED | 0.39 m | 24.4 m | 1.07 m | 30 s |
| map (−14, 9)　回家 | SUCCEEDED | 0.39 m | 32.2 m | 0.85 m | 49 s |

起點處 odom 報的位置與真值一致（−14.00, +9.00）。平均 0.5–0.7 m/s。

誤差都落在 0.38～0.39 m，是因為 `xy_goal_tolerance` 就是 0.40 —— 車子一進
容許範圍就停。要更準就得把容許值調小，但 0.25 試過會在終點抽動（見下）。

### 這份數據改過兩次，前兩版都是錯的

1. **第一版**用 `/husky/odom` 當量尺，而那支 odom 正在灌水（滑移轉向）。
   **用待測系統自己的輸出當證據是不行的**，現在一律用 `gz model -m husky -p`。
2. **第二版**量法對了，但參數設錯：`xy_goal_tolerance: 0.25` 配
   `yaw_goal_tolerance: 0.26`，車子跑完 32.7 m 只要 51 秒，卻在終點
   原地抽動（yaw 在 +67°↔+91° 來回）166 秒，最後逾時失敗。
   改成 0.40 / 3.14 並拿掉 RotateToGoal 之後，同一趟 49 秒完成。

## H3 的驗收數據（2026-11-07）

| 檢查 | 結果 |
|---|---|
| 四項就緒條件（platform／odom／nav2／bridge） | 全滿足，23 秒 |
| `scene.home` 讀得到 | [−14.0, 9.0] |
| `api/state` 回報位置 vs 真值 | (−14.00, +9.00) vs (−14.00, +9.00) |
| `api/goto` → task.seq 0→1、state running→done | 通過 |
| 真的到達（真值） | 落點 (+5.23, −8.93)、誤差 0.38 m、38 s |
| 導航途中 `api/stop` | 停止後 3 秒位移 **0.000 m** |
| 被停止時 task.state | `failed`：被停止指令取消 |

喊停時在 (+3.51, −9.10)、停在 (+2.40, −8.95)：中間 1.1 m 是服務呼叫往返
加上減速距離，停車不是瞬間的。

## H5 的驗收數據（2026-11-07，兩台同場）

```
ros2 launch husky_nodered both_all.launch.py
```

| 檢查 | 結果 |
|---|---|
| 兩台的 `api/status` | 94 秒時都上線 |
| TF 座標系 | `base_link`／`husky/base_link`／`husky/odom`／`odom` 四個並存 |
| 兩套 Nav2 | `/navigate_to_pose` 與 `/husky/navigate_to_pose` |
| **同時下任務** | 無人機起飛到 2.96 m、地面車到 (+5.22, −8.93) 誤差 0.39 m，50 秒內都完成 |
| CPU | 最高 3.2 / 平均 2.2（16 核） |

設計重點：世界檔只有一份（`drone_nodered/gz/worlds/warehouse.sdf`，含 Husky），
Gazebo 由 PX4 那邊開，Husky 這邊 `start_gz:=false`；兩套 Nav2 的 configure
錯開 35 秒（同時 configure 會卡在 Configuring map_server 且不會重試）。

## H4 的驗收（2026-11-07，Oliver 親自在儀表板上按）

流程由 `nodered/make_husky_basic.py` 產生，用
`drone_nodered/scripts/install_flows.sh <husky_basic.json>` 和無人機那份
**一起**裝（分開裝會互相刪除，見下）。

| 檢查 | 結果 |
|---|---|
| 分頁 | 無人機控制／系統／儀表板／**地面車（Husky）**／自訂任務（使用者的保留） |
| 積木 | ROS 2 兩個通用 ＋ 無人機 10 ＋ 地面車 4，左側面板分三類 |
| 儀表板 | 就緒燈、位置、動作、最近回覆、地圖、座標輸入、四顆按鈕 |
| 端到端 | 按「導航到目標」→ 車子繞過貨架到東南角；停止／返航／狀態查詢都正常 |
| **`nodered_ros2` 的改動** | **0 行** ← G-4／P-1 的實證 |

### 部署時踩到的四個雷（都已修掉並留下防線）

1. **`merge_flows.py` 的標記寫死 `generated-by: drone_nodered`** —— 拿地面車的
   流程單獨合併會把無人機的 14 個分頁與積木整批刪掉。改成 regex 認得任何
   `<套件>_nodered`。
2. **積木同名** —— 地面車的「導航到座標」和無人機同名，而 merge 有「名稱相同
   就當成同一個」的比對。改名成「車：…」。
3. **共用積木的定義必須逐字相同** —— `nodered_ros2` 的兩個通用積木如果一份有
   `generated-by` 標記、一份沒有，併進同一個 Node-RED 時後載入的會蓋掉前一個
   而且不報錯。改成從無人機那份原封不動複製；`install_flows.sh` 併檔時會檢查。
4. **服務層要常駐，不能跟著場景走** —— `husky_api_node` 原本放在場景的 launch，
   沒開模擬時儀表板整塊是空的（地圖沒資料、按鈕沒反應、三個症狀都不報錯）。
   改成掛在底座 `nodered_all.launch.py`，和 `drone_api_node` 一樣。
5. **被拒絕的理由要進狀態** —— 服務回 `success=false` 時只回給呼叫端的話，
   儀表板什麼都不會變，使用者看到的是「按了沒反應」。現在寫進
   `status.last_message`，儀表板有「最近回覆」那一行。

## 跑起來

```bash
# 開畫面
ros2 launch husky_nodered husky_sim.launch.py
# 無頭
ros2 launch husky_nodered husky_sim.launch.py gui:=false

# 模擬 + TF + Nav2 一起（H2）
ros2 launch husky_nodered husky_all.launch.py
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 5.6, y: -9.0}, orientation: {w: 1.0}}}}'

# 走服務層（H3，Node-RED 積木之後就是呼叫這些）
ros2 service call /husky/api/goto vehicle_nodered_interfaces/srv/GoTo \
  '{map_x: 5.6, map_y: -9.0, yaw: 999.0, timeout: 0.0}'
ros2 service call /husky/api/stop  std_srvs/srv/Trigger
ros2 service call /husky/api/state std_srvs/srv/Trigger
ros2 topic echo /husky/api/status --full-length

# 開車（H2 之後這件事由 Nav2 做）
ros2 topic pub -r 10 /husky/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'
ros2 topic echo /husky/odom
```

話題對照（gz 那邊的名字寫在 `gz/models/husky/model.sdf` 的 DiffDrive 裡）：

| ROS | 方向 | gz |
|---|---|---|
| `/husky/cmd_vel` | ROS → gz | `/model/husky/cmd_vel` |
| `/husky/odom` | gz → ROS | `/model/husky/odom` |
| `/tf` | gz → ROS | `/model/husky/tf` |

## 車體幾何的唯一來源

原廠模型自己帶的數字，**不要去查別的規格表**：

```
輪距 0.5709 m    輪半徑 0.17775 m    車體約 0.99 x 0.67 m
模型原點離地 0.14275 m（輪心 0.035 + 輪半徑）
速度上限 1.0 m/s   扭力 35 N·m
```

這些值有四個消費者（DiffDrive、輪子的 collision、`warehouse_geometry.yaml`、
世界檔的 pose z），`test/t_husky_model.py` 會把它們綁在一起檢查。

## 三個雷

1. **DiffDrive 的 `<topic>` 是絕對路徑**，不會自動加 model 的 namespace。
   只寫 `cmd_vel` 會開在全域 `/cmd_vel`，bridge 等在 `/model/husky/...`
   會完全收不到訊息 —— 而且話題宣告得出來，所以很難查。
2. **gz 的 DiffDrive 沒有指令逾時**：送過一筆 `cmd_vel` 之後，沒人再送它也會
   永遠沿用最後那一筆，車子會一路開到撞牆、輪子在牆邊空轉。所以 Nav2 和
   bridge 中間一定要有 `scripts/cmd_vel_watchdog.py`（0.5 秒沒指令就送零），
   對應無人機那邊 `cmd_vel_to_px4_node` 的 `cmd_timeout_s`。

3. **四輪滑移轉向，輪式 odom 的轉角會灌水 46%**：實測下 0.5 rad/s，
   真值只轉到 0.69 倍，odom 卻報 1.44 倍。所以
   ① DiffDrive 用「等效輪距」0.834（＝實際 0.5709 × 1.46），達成率 0.69 → 0.98
   ② 定位改用 OdometryPublisher 的**真值**里程計，輪式那份改名 `odom_wheel` 留著比較
   ③ 真值里程計報的是**世界座標**，所以 `map → odom` 是單位變換，不是 spawn 偏移
   實機要用的話這三點都不成立，必須換成 AMCL 或輪式＋IMU 的 EKF。

4. **世界檔是產生出來的**，見 `gz/worlds/README.md`。跑 `gen_warehouse.py`
   記得加 `--target hatchback`。

## 模型出處

`gz/models/husky/` 來自 Gazebo Fuel 的
[Clearpath Robotics Husky A200](https://app.gazebosim.org/OpenRobotics/fuel/models/Clearpath%20Robotics%20Husky%20A200)
（OpenRobotics 發布，CC0 1.0）。改了三處，都記在 `model.sdf` 的檔頭。
