#!/usr/bin/env bash
# sim の trace から、指定時刻の候補の費用の内訳と最接近すきまを出す cand_probe を dev コンテナ内でビルド・実行する。
# 使い方: tools/run_cand_probe.sh TRACE T [T ...]   (TRACE はリポジトリ相対、T は周期の開始時刻 [s])
# 出力: 標準出力(OUT があればそのファイルにも。リポジトリ相対)
set -euo pipefail
cd "$(dirname "$0")/.."
[ $# -ge 2 ] || { echo "usage: $0 TRACE T [T ...]" >&2; exit 2; }
TRACE="$1"; shift
TIMES="$*"
docker image inspect social_nav-dev:jazzy >/dev/null 2>&1 || docker compose -f docker/compose.yaml build dev >/dev/null 2>&1 || true
run() {
  docker compose -f docker/compose.yaml run --rm -T dev bash -lc "
  set -euo pipefail
  cmake -S src/social_nav/social_nav_core -B build/rel -DCMAKE_BUILD_TYPE=Release >/dev/null
  cmake --build build/rel -j --target cand_probe >/dev/null
  build/rel/cand_probe src/social_nav/$TRACE $TIMES
"
}
if [ -n "${OUT:-}" ]; then run | tee "$OUT"; else run; fi
