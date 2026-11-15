#!/usr/bin/env bash
# 地面車的回歸測試。都不需要 Gazebo，t_status_contract 需要 ROS 2 環境。
#     bash ~/ros2_ws/src/husky_nodered/test/run_all.sh
HERE="$(cd "$(dirname "$0")" && pwd)"
pass=0; fail=0; failed=()

run() {
    local name="$1"; shift
    printf '\n\033[1m=== %s ===\033[0m\n' "$name"
    if "$@"; then pass=$((pass+1)); else fail=$((fail+1)); failed+=("$name"); fi
}

run "Husky 模型一致性（t_husky_model.py）"   python3 "$HERE/t_husky_model.py"
run "追蹤站位幾何（t_track_geometry.py）"     python3 "$HERE/t_track_geometry.py"
run "狀態契約（t_status_contract.py）"        python3 "$HERE/t_status_contract.py"
run "流程結構（t_husky_flow.py）"             python3 "$HERE/t_husky_flow.py"
run "地圖 widget 共用（t_map_widget_husky.js）" node "$HERE/t_map_widget_husky.js"

printf '\n'
if [ $fail -eq 0 ]; then
    printf '\033[32m✓ 全部通過（%d 支）\033[0m\n' "$pass"
else
    printf '\033[31m✗ %d 支失敗：\033[0m\n' "$fail"
    for f in "${failed[@]}"; do echo "    - $f"; done
    exit 1
fi
