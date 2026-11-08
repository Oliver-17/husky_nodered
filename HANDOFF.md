# 交接：地面車（Husky）接入 Low-Code 工具

**最後更新：2026-11-08**　寫這份是為了讓新的對話不用讀舊對話就能接手。

---

## 一句話

無人機和地面車同場、各自能被 Node-RED 的積木和儀表板按鈕驅動，
**`nodered_ros2` 一行都沒改** —— 規格書 G-4／P-1 的實證。

## 現在是什麼狀態

| 階段 | 內容 | 狀態 |
|---|---|---|
| H0 | `ros_gz_bridge`（Harmonic）雙向 | ✅ |
| H1 | Husky 進倉庫場景、`cmd_vel` 開得動 | ✅ |
| H2 | Nav2 給差速車 | ✅ |
| H3 | `husky_api_node` ＋ `vehicle_nodered_interfaces` | ✅ |
| H4 | Node-RED 四個積木 ＋ 儀表板 | ✅ |
| H5 | 兩台同場 | ✅（模擬） |

驗收數字都在 `README.md`，**不要**在這裡重複。

## 怎麼跑起來

```bash
systemctl --user start drone-nodered.service     # 底座：rosbridge + 兩台的服務層 + Node-RED
# 儀表板按「開模擬」，或：
systemctl --user start drone-warehouse.service   # 場景：Gazebo + PX4 + 兩套 Nav2
```

⚠️ **全部就緒要等約 95 秒**（地面車的 Nav2 排在第 70 秒才起，兩套同時
configure 會卡在 Configuring map_server 而且不會重試）。沒等到就說「未就緒」
是正常的，不是壞掉。

測試：

```bash
bash test/run_all.sh                 # 4 支，不用開模擬，幾秒
python3 test/sim/t_nav_e2e.py        # 地面車單獨跑，約 3 分鐘，要 Gazebo
python3 test/sim/t_both_vehicles.py  # 兩台同場，約 3 分鐘，要 Gazebo + PX4
```

## 還沒做的事（接手就從這裡開始）

1. **任務要能同時驅動兩台**（Oliver 2026-11-08 提的）。積木層的條件已經具備
   ——兩台共用同一套 `task.seq`／`task.state` 協定、同一個「等待完成」邏輯。
   要decide 的是語意：一顆按鈕按下去，兩台是**並行**還是**依序**？失敗時
   另一台要不要停？建議先在「自訂任務」分頁手動串一個並行的版本試跑，
   確認行為之後再決定要不要做成儀表板按鈕。
2. **任務完成百分比**。只有導航（Nav2 的 `distance_remaining` ÷ 接下目標當刻的
   距離）和追蹤（已追秒數 ÷ 設定秒數）算得出真的進度；起飛、停止、狀態查詢
   沒有過程。結論是「有進度就顯示，沒有就顯示狀態」，但還沒實作 ——
   兩個 api node 都要記下「接下目標當刻的距離」。
3. **commit 與 repo**。見下。
4. **文件**。`Low-Code 應用示範與成果報告.odt` 的第四章（地面車自主巡檢）
   本來就留白，現在有東西可以填了。

## 版控狀態 ⚠️

| 套件 | 狀態 |
|---|---|
| `drone_nodered` | **13 個檔案未提交** |
| `drone_nodered_interfaces` | **3 個未提交**（`GoTo.srv`／`Track.srv` 搬走了） |
| `husky_nodered` | **還不是 git repo** |
| `vehicle_nodered_interfaces` | **還不是 git repo** |
| `nodered_ros2`／`drone_control`／`drone_vision` | 乾淨（這就是 G-4 的證據） |

Oliver 2026-11-07 的決定：先不開新 repo，等兩個新套件要開的時候一起處理。
⚠️ 兩個既有 repo 現在推上去的話，單獨 clone 會編不起來（缺
`vehicle_nodered_interfaces`）。

## 接手前必讀的三個檔

1. `README.md` —— 階段驗收數字、五個踩過的雷
2. `config/warehouse_nav2_husky.yaml` 的檔頭 —— 和無人機的 Nav2 差在哪、為什麼
3. `gz/models/husky/model.sdf` 的檔頭 —— 車體幾何的唯一來源、滑移轉向的修正

## 兩條不要再踩的規則

- **量測不要用待測系統自己的輸出。** 位置一律用 `gz model -m husky -p` 的真值。
  2026-11-07 用 `/husky/odom` 量出「誤差 0.24 m」的漂亮數字，實際上車子卡在
  31 公尺外的牆角。
- **儀表板的地圖 widget 是手工做的，產生器做不出來。** 不要用
  `make_drone_basic.py --force` 重建無人機的流程，那會把地圖和一個儀表刪掉。
  要改它就直接改 `drone_nodered/nodered/flows/drone_basic.json`，
  改完跑 `drone_nodered/test/t_map_widget.js`。
