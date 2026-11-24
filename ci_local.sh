#!/usr/bin/env bash
# Local/CI entry point. Host: re-runs itself inside the dev container.
set -euo pipefail

if [[ "${1:-}" != "--in-container" ]]; then
  REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  mkdir -p "$REPO/.ccache"
  exec docker compose -f "$REPO/docker/compose.yaml" run --rm -T \
    -e CI_SKIP_SMOKE="${CI_SKIP_SMOKE:-}" \
    dev bash /ws/src/social_nav/ci_local.sh --in-container
fi

SRC=${SRC:-/ws/src/social_nav}
cd /ws
STAGE=init
trap 'rc=$?; if [[ $rc -ne 0 ]]; then echo "CI FAILED at stage: $STAGE"; fi' EXIT

BASE_PATHS=("$SRC/social_nav_core")
[[ -d "$SRC/ros2" ]] && BASE_PATHS+=("$SRC/ros2")

# ccache の無い古いイメージでも動くよう、あるときだけ使う
CCACHE_ARGS=()
if command -v ccache >/dev/null; then
  CCACHE_ARGS=(-DCMAKE_CXX_COMPILER_LAUNCHER=ccache)
else
  echo "ccache not found: building without it (rebuild the image to enable it)"
fi

STAGE=core
echo "=== [1/3] core: cmake build + ctest ==="
cmake -S "$SRC/social_nav_core" -B /ws/build/core -DCMAKE_COMPILE_WARNING_AS_ERROR=ON \
  "${CCACHE_ARGS[@]}"
cmake --build /ws/build/core -j
ctest --test-dir /ws/build/core --output-on-failure --no-tests=error --parallel "$(nproc)"
python3 "$SRC/tools/m4_select.py" --self-test
python3 "$SRC/tools/pair_m1.py" --self-test
python3 "$SRC/tools/branch49.py" --self-test
python3 "$SRC/tools/r_select.py" --self-test
python3 "$SRC/tools/ttc_check.py" --self-test

STAGE=colcon
echo "=== [2/3] colcon: build + test ==="
set +u
source /opt/ros/jazzy/setup.bash
set -u
colcon build --base-paths "${BASE_PATHS[@]}" --cmake-args -DCMAKE_COMPILE_WARNING_AS_ERROR=ON \
  "${CCACHE_ARGS[@]}"
# social_nav_core のテストは [1/3] の ctest と同一集合（同じ CMakeLists）なので、ここでは重複して走らせない
colcon test --base-paths "${BASE_PATHS[@]}" --packages-skip social_nav_core --return-code-on-test-failure
colcon test-result --verbose

STAGE=smoke
echo "=== [3/3] smoke ==="
SMOKE=/ws/install/social_nav_bringup/share/social_nav_bringup/test/smoke_launch.sh
if [[ "${CI_SKIP_SMOKE:-}" == "1" ]]; then
  echo "skip (CI_SKIP_SMOKE=1)"
elif [[ -f "$SMOKE" ]]; then
  set +u
  source /ws/install/setup.bash
  set -u
  # (a) 既定: door_yield の購読なし(controller_server は /map を購読しない)、地図受信のログなし
  TIMEOUT=120 ABSENT_LOG="door_yield: map received" EXPECT_CTRL_MAP_SUB=0 bash "$SMOKE"
  # (b) door_yield: true(SocialDWA。両 plugin とも常に configure される)で /map を購読し、地図受信のログがちょうど 1 回
  PARAMS_B=$(mktemp --suffix=.yaml)
  python3 - "$PARAMS_B" <<'PY'
import sys, yaml
src = "/ws/install/social_nav_bringup/share/social_nav_bringup/params/nav2_social_params.yaml"
doc = yaml.safe_load(open(src))
doc["controller_server"]["ros__parameters"]["SocialDWA"]["door_yield"] = True
yaml.safe_dump(doc, open(sys.argv[1], "w"))
PY
  TIMEOUT=120 LAUNCH_ARGS="params_file:=$PARAMS_B" EXPECT_LOG="door_yield: map received" EXPECT_CTRL_MAP_SUB=1 bash "$SMOKE"
else
  echo "skip (smoke_launch.sh not installed)"
fi

STAGE=done
echo "CI OK"
