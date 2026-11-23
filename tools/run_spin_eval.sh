#!/usr/bin/env bash
# 譲るときの 1 周: 1 周(譲るときの回転)の発生率を測る。定義は docs/sim_results.md の `## 譲るときの 1 周` 節。sim は変更しない。
# 16 シナリオ(all 6、rear 3、indoor 3、blind_corner(--occlusion)、sudden 3)× シード範囲を social で 1 本ずつ --trace 付きで回し、
# tools/spin_count.py で 1 本ずつ集計する(trace は 1 周のものだけ残す)。
# 環境変数:
#   START(既定 1)、N(既定 20)   = シード範囲 START..START+N-1
#   SCENS(既定: 16 シナリオ全部)= 空白区切りのシナリオ名。例: "door_oncoming side_door group_oncoming"
#   OUT_DIR(既定 build/spin_eval。リポジトリ相対)  TAG(既定 run)= 出力名 $OUT_DIR/$TAG.csv、$OUT_DIR/$TAG.txt
#   SIM_EXTRA = sim に足す引数(例: "--v-min -0.2")。KEEP=1 なら全 trace を残す
#   JOBS(既定 1 = 直列)= シナリオ×シードを最大 JOBS 本まで並列に走らせる。出力は直列と同じ。1 本でも失敗すれば非 0
# 21–120 は走らせない(終了 2)。
set -euo pipefail
cd "$(dirname "$0")/.."
START="${START:-1}"; N="${N:-20}"; OUT_DIR="${OUT_DIR:-build/spin_eval}"; TAG="${TAG:-run}"
SIM_EXTRA="${SIM_EXTRA:-}"; KEEP="${KEEP:-0}"; JOBS="${JOBS:-1}"
[[ "$JOBS" =~ ^[1-9][0-9]{0,2}$ ]] || { echo "error: JOBS は正の整数" >&2; exit 2; }
SCENS="${SCENS:-corridor_oncoming crossing overtake follow static_group crowd rear_follow rear_overtake rear_overtake_flow door_oncoming side_door group_oncoming blind_corner sudden_stop sudden_turn sudden_reverse}"
for v in START N; do [[ "${!v}" =~ ^[1-9][0-9]{0,8}$ ]] || { echo "error: $v は正の整数" >&2; exit 2; }; done
if [ "$START" -le 120 ] && [ $((START+N-1)) -ge 21 ]; then echo "error: 範囲 $START..$((START+N-1)) が判定範囲 21–120 と重なる" >&2; exit 2; fi
if [[ " $SIM_EXTRA " =~ [[:space:]]--(seeds|seed-start|trace)([[:space:]=]) ]]; then echo "error: SIM_EXTRA に --seeds / --seed-start / --trace を入れない" >&2; exit 2; fi
if [ "$JOBS" -gt 1 ]; then   # 並列では同じ出力先・同じ trace を複数ジョブが書く指定を拒否する
  if [[ " $SIM_EXTRA " =~ [[:space:]]--(csv|contacts-csv|wall-csv|blind-csv|event-csv)([[:space:]=]) ]]; then echo "error: JOBS>1 では SIM_EXTRA に出力先の指定(--csv / --*-csv)を入れない" >&2; exit 2; fi
  if [ -n "$(printf '%s\n' $SCENS | sort | uniq -d)" ]; then echo "error: JOBS>1 では SCENS に同じシナリオを重ねない" >&2; exit 2; fi
fi
GIT_SHA="$(git rev-parse HEAD)"; [ -n "$(git status --porcelain -- social_nav_core tools 2>/dev/null)" ] && GIT_SHA="$GIT_SHA-dirty"
mkdir -p "$OUT_DIR"
docker image inspect social_nav-dev:jazzy >/dev/null 2>&1 || docker compose -f docker/compose.yaml build dev >/dev/null 2>&1 || true
docker compose -f docker/compose.yaml run --rm -T \
  -e O="src/social_nav/$OUT_DIR" -e TAG="$TAG" -e START="$START" -e N="$N" -e SCENS="$SCENS" -e SIM_EXTRA="$SIM_EXTRA" -e KEEP="$KEEP" -e JOBS="$JOBS" -e GIT_SHA="$GIT_SHA" \
  dev bash -lc '
  set -euo pipefail
  cmake -S src/social_nav/social_nav_core -B build/rel -DCMAKE_BUILD_TYPE=Release >/dev/null
  cmake --build build/rel -j --target social_nav_sim >/dev/null
  PY=src/social_nav/tools/spin_count.py
  mkdir -p $O/trace_$TAG
  python3 $PY header > $O/$TAG.csv
  # シナリオ × シードを最大 JOBS 本まで並列に走らせる。各ジョブは自分の行を $W/<s>_<k>.row に書き、下でシナリオ順・シード順に結合する
  W=$O/work_$TAG; rm -rf $W; mkdir -p $W
  one() {
    local s=$1 k=$2 OCC="" rc=0
    [ "$s" = blind_corner ] && OCC="--occlusion"
    local tr=$O/trace_$TAG/${s}_$k.trace
    build/rel/social_nav_sim --scenario $s --seeds 1 --seed-start $k --controller social $OCC $SIM_EXTRA --trace $tr > /dev/null || return $?
    python3 $PY row $tr > $W/${s}_$k.row || return $?
    if [ "$KEEP" != 1 ]; then   # 1 周のものだけ trace を残す
      python3 - $W/${s}_$k.row $O/$TAG.csv <<PYEOF || rm -f $tr
import csv,sys
h=open(sys.argv[2]).readline()
r=list(csv.DictReader([h, open(sys.argv[1]).read()]))[-1]
sys.exit(0 if int(r["spin270"]) else 1)
PYEOF
    fi
  }
  # 失敗を見たら新規起動を止める。sim / row の終了コードはそのまま返す
  for s in $SCENS; do
    for ((k=START; k<START+N; k++)); do
      while [ "$(jobs -rp | wc -l)" -ge "$JOBS" ]; do wait -n || true; done
      if compgen -G "$W/*.failed" >/dev/null; then break 2; fi
      { one $s $k || echo $? > $W/${s}_$k.failed; } &
    done
  done
  wait
  if compgen -G "$W/*.failed" >/dev/null; then
    for ff in $W/*.failed; do rc=$(cat $ff); break; done
    echo "error: 失敗(終了 $rc): $(ls $W/*.failed)" >&2; rm -rf $W; exit "${rc:-1}"
  fi
  for s in $SCENS; do
    for ((k=START; k<START+N; k++)); do cat $W/${s}_$k.row >> $O/$TAG.csv; done
  done
  rm -rf $W
  { echo "git_sha: $GIT_SHA"; echo "seeds: $START..$((START+N-1))"; echo "scenarios: $SCENS"; echo "sim_extra: $SIM_EXTRA"; echo "binary: build/rel/social_nav_sim (controller=social, ped-model default)"; echo; python3 $PY table $O/$TAG.csv; } > $O/$TAG.txt
'
cat "$OUT_DIR/$TAG.txt"
