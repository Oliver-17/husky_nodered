# 這個目錄現在是空的 —— 世界檔不在這裡

2026-11-07 起，倉庫世界檔**只有一份**，在 `drone_nodered/gz/worlds/warehouse.sdf`，
而且裡面已經包含地面車 Husky。

## 為什麼不放一份「含 Husky」的在這裡

`drone_nodered/scripts/start_empty_sitl.sh`（PX4 SITL 的啟動腳本）把世界檔名
綁死了三樣東西：

1. 檔案位置只找 `drone_nodered/gz/worlds`
2. `PX4_GZ_WORLD` 必須等於世界檔裡的 `<world name>`
3. 還要有同名的 `config/<world>_geometry.yaml`

多一份世界檔就要同步這三處，而且「打開模擬兩台都在」本來就是需求，
所以一份就夠。

## 產生方式

```bash
python3 src/drone_nodered/scripts/gen_warehouse.py --target hatchback
#   → drone_nodered/gz/worlds/warehouse.sdf（倉庫 + 目標車 + Husky）
python3 src/drone_nodered/scripts/gen_warehouse.py --target hatchback --no-husky
#   → 同一個檔案，但沒有地面車
```

Husky 的**模型**仍然在這個套件（`gz/models/husky`），由
`start_empty_sitl.sh` 把它加進 `GZ_SIM_RESOURCE_PATH`。
