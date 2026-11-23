#!/usr/bin/env bash
# 歩行者モデル(legacy / sfm2)をロボットなしで測る ped_probe を、dev コンテナ内でビルドから実行まで 1 回で行う。
# 出力: build/ped_probe/ped_probe.txt(条件 + 表)。seed は 1001-1020 固定(ped_probe 内)。
# 前提: docker、イメージ social_nav-dev:jazzy(compose が必要なら build する)
# 環境変数: OUT_DIR(既定 build/ped_probe。リポジトリ相対)
#   H0=1: 通常の表の代わりに H0(sfm2 のシード 1021-1120)だけを実行し ped_probe_h0.txt に書く。H0_SEED0=N は動作確認用の上書き
set -euo pipefail
cd "$(dirname "$0")/.."
OUT_DIR="${OUT_DIR:-build/ped_probe}"
H0="${H0:-0}"
if [ "$H0" = 1 ]; then PROBE_ARGS="--h0"; [ -n "${H0_SEED0:-}" ] && PROBE_ARGS="$PROBE_ARGS --h0-seed0 $H0_SEED0"; OUT_NAME=ped_probe_h0.txt; else PROBE_ARGS=""; OUT_NAME=ped_probe.txt; fi

GIT_SHA="$(git rev-parse HEAD)"
if [ -n "$(git status --porcelain -- social_nav_core 2>/dev/null)" ]; then GIT_DIRTY=yes; else GIT_DIRTY=no; fi
docker image inspect social_nav-dev:jazzy >/dev/null 2>&1 || docker compose -f docker/compose.yaml build dev >/dev/null 2>&1 || true   # 無ければ 1 回だけ build(build のたびに ID が変わるため毎回はしない)
IMAGE_ID="$(docker image inspect social_nav-dev:jazzy --format '{{.Id}}' 2>/dev/null || echo unknown)"
rm -rf "$OUT_DIR"; mkdir -p "$OUT_DIR"

docker compose -f docker/compose.yaml run --rm -T \
  -e GIT_SHA="$GIT_SHA" -e GIT_DIRTY="$GIT_DIRTY" -e IMAGE_ID="$IMAGE_ID" -e OUT="src/social_nav/$OUT_DIR" -e PROBE_ARGS="$PROBE_ARGS" -e OUT_NAME="$OUT_NAME" \
  dev bash -lc '
  set -euo pipefail
  cmake -S src/social_nav/social_nav_core -B build/rel -DCMAKE_BUILD_TYPE=Release >/dev/null
  cmake --build build/rel -j --target ped_probe >/dev/null
  CACHE=build/rel/CMakeCache.txt
  BT=$(sed -n "s/^CMAKE_BUILD_TYPE:[A-Z]*=//p" $CACHE)
  CXX=$(sed -n "s/^CMAKE_CXX_COMPILER:[A-Z]*=//p" $CACHE)
  CXXV=$("$CXX" --version | head -n1)
  {
    echo "git_sha: $GIT_SHA"
    echo "git_dirty: $GIT_DIRTY (social_nav_core/ 以下)"
    echo "docker_image_id: $IMAGE_ID"
    echo "arch: $(uname -m)"
    echo "host: $(uname -srm)"
    echo "build_type: $BT"
    echo "compiler: $CXXV ($CXX)"
    echo
  } > $OUT/$OUT_NAME
  build/rel/ped_probe $PROBE_ARGS >> $OUT/$OUT_NAME
  cat $OUT/$OUT_NAME
'
echo "wrote $OUT_DIR/$OUT_NAME"
