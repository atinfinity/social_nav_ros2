#!/usr/bin/env bash
# ヘッダレスのスモーク起動。静的 TF(map->odom->base_link)だけで全 lifecycle ノードが active になるか判定する。
# 使い方: source install/setup.bash && smoke_launch.sh   環境変数: TIMEOUT(秒, 既定 60), LAUNCH_ARGS
# 追加の検査(任意): EXPECT_LOG(正規表現)= ログにちょうど 1 回出る、ABSENT_LOG = 出ない、EXPECT_CTRL_MAP_SUB(1/0)= controller_server が /map を購読している/いない
# 終了コード: 0 = 全ノード active かつログに致命的エラーなし / 1 = active に届かない / 2 = ログにロードエラー / 3 = 追加の検査に失敗
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
if [ -n "${EXPECT_LOG:-}" ]; then
  n=0
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    n=$(grep -cE "$EXPECT_LOG" "$LOG"); [ "$n" -ge 1 ] && break; sleep 1
  done
  if [ "$n" -ne 1 ]; then echo "EXPECT_LOG '$EXPECT_LOG' matched $n times (want 1)"; exit 3; fi
  echo "log ok: '$EXPECT_LOG' x1"
fi
if [ -n "${ABSENT_LOG:-}" ] && grep -qE "$ABSENT_LOG" "$LOG"; then
  echo "ABSENT_LOG '$ABSENT_LOG' found in log"; exit 3
fi
if [ -n "${EXPECT_CTRL_MAP_SUB:-}" ]; then
  # /map の購読者(ros2 topic info -v の Subscription 以降)。発見の遅れで欠けることがあるので、既知の購読者
  # (global_costmap, people_tracker_node)が見えるまで待ってから controller_server の有無を見る
  for _ in 1 2 3 4 5 6 7 8; do
    subs="$(timeout 10 ros2 topic info -v /map 2>/dev/null | awk '/^Subscription count/{f=1} f && /^Node name:/{print $3}' | sort | tr '\n' ' ')"
    case "$subs" in *global_costmap*people_tracker_node*) break ;; esac
    sleep 2
  done
  echo "/map subscribers: $subs"
  case "$subs" in *global_costmap*people_tracker_node*) ;; *) echo "known /map subscribers not seen"; exit 3 ;; esac
  has=0; case "$subs" in *controller_server*) has=1 ;; esac
  if [ "$has" != "$EXPECT_CTRL_MAP_SUB" ]; then echo "controller_server /map subscription: got $has, want $EXPECT_CTRL_MAP_SUB"; exit 3; fi
fi
echo "SMOKE OK: all nodes active (log: $LOG)"
exit 0
