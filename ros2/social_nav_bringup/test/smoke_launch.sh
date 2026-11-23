#!/usr/bin/env bash
# ヘッダレスのスモーク起動。静的 TF(map->odom->base_link)だけで全 lifecycle ノードが active になるか判定する。
# 使い方: source install/setup.bash && smoke_launch.sh   環境変数: TIMEOUT(秒, 既定 60), LAUNCH_ARGS
# 終了コード: 0 = 全ノード active かつログに致命的エラーなし / 1 = active に届かない / 2 = ログにロードエラー
set -u
TIMEOUT="${TIMEOUT:-60}"
LOG_DIR="$(mktemp -d)"
LOG="$LOG_DIR/launch.log"
NODES=(/map_server /controller_server /planner_server /behavior_server /bt_navigator
       /velocity_smoother /collision_monitor /local_costmap/local_costmap /global_costmap/global_costmap)

PIDS=()
cleanup() {
  for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null; done
  sleep 2
  for p in "${PIDS[@]}"; do kill -KILL "$p" 2>/dev/null; done
}
trap cleanup EXIT

tf() { ros2 run tf2_ros static_transform_publisher --frame-id "$1" --child-frame-id "$2" >"$LOG_DIR/tf_$2.log" 2>&1 & PIDS+=($!); }
tf map odom
tf odom base_link

# shellcheck disable=SC2086
ros2 launch social_nav_bringup social_nav.launch.py ${LAUNCH_ARGS:-} >"$LOG" 2>&1 &
PIDS+=($!)

state_of() { timeout 10 ros2 lifecycle get "$1" 2>/dev/null | head -n1; }

deadline=$((SECONDS + TIMEOUT))
all_active=0
while [ $SECONDS -lt $deadline ]; do
  not_active=()
  for n in "${NODES[@]}"; do
    s="$(state_of "$n")"
    case "$s" in *"active [3]"*) ;; *) not_active+=("$n($s)") ;; esac
  done
  if [ ${#not_active[@]} -eq 0 ]; then all_active=1; break; fi
  sleep 2
done

echo "=== lifecycle states ==="
for n in "${NODES[@]}"; do echo "$n: $(state_of "$n")"; done
if pgrep -f people_tracker_node >/dev/null; then echo "/people_tracker_node: running (non-lifecycle)"; else echo "/people_tracker_node: NOT running"; all_active=0; fi

# プラグイン・BT XML のロードエラー検出
if grep -E "Failed to load|failed to load|according to the loaded plugin descriptions|PluginlibFactory|Failed to create|\[FATAL\]|process has died|Failed to parse|Error loading|Exception thrown|terminate called" "$LOG" >"$LOG_DIR/errors.txt"; then
  echo "=== load errors in log ($LOG) ==="
  cat "$LOG_DIR/errors.txt"
  exit 2
fi
if [ "$all_active" -ne 1 ]; then
  echo "NOT all nodes active within ${TIMEOUT}s; not active: ${not_active[*]:-}"
  echo "--- log tail ($LOG) ---"; tail -n 60 "$LOG"
  exit 1
fi
echo "SMOKE OK: all nodes active (log: $LOG)"
exit 0
