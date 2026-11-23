#!/usr/bin/env bash
# sim の合否判定用の測定を 1 回で再生成する(design 6.3)。
# 流れ: dev コンテナ(Linux/libstdc++)で build/rel を Release ビルド → 5 構成 × 2 シード範囲 × 全 6 シナリオを実行(既定は直列。JOBS で並列)
#       → build/eval/ に表(*.txt)・CSV(*.csv)・条件(conditions.txt)・結合表(summary.txt)・結合 CSV(all.csv)を出力。
# 前提: docker、イメージ social_nav-dev:jazzy(compose が必要なら build する)
# 環境変数(既定は判定用の本番条件。動作確認は少ない値で):
#   SELECT_SEEDS(既定 20)、SELECT_START(既定 1)   = 「選定に使用」の範囲(判定に使わない)
#   JUDGE_SEEDS(既定 100)、JUDGE_START(既定 21)   = 判定用の範囲
#   OUT_DIR(既定 build/eval。リポジトリ相対)
#   JUDGE_SEEDS=0 なら判定用の範囲を実行しない(選定範囲だけの診断用。all.csv も選定範囲だけ)
#   SIM_EXTRA(既定は空)= social_nav_sim に全構成で追加する引数(例: "--v-min -0.2")
#   TRACKER_EXTRA(既定は空)= use_tracker 構成だけに追加する引数(例: "--tracker-foo 1")。空なら出力は従来と同じ
#   CONTACTS=1 なら各構成・範囲ごとに接触イベントの CSV を $OUT/<構成>_<範囲>_contacts.csv に出す(既定 0。診断用)
#   REAR=1 なら全 6 シナリオの代わりに後方からの人流 3 本(--scenario rear。記録のみ)だけを走らせる(既定 0 = 従来と同じ)。OUT_DIR は別にすること
#   INDOOR=1 なら全 6 シナリオの代わりに屋内 3 本(--scenario indoor: door_oncoming / side_door / group_oncoming。記録のみ)だけを走らせる(既定 0 = 従来と同じ)。
#     REAR=1 との併用、JUDGE_SEEDS が 0 でない、選定範囲が 21–120 と重なる、はエラー(終了 2、OUT_DIR を消す前)。OUT_DIR は別にすること
#   BLIND=1 なら全 6 シナリオの代わりに見えない人 1 本(--scenario blind: blind_corner。記録のみ)だけを、全構成に --occlusion と --blind-csv を付けて走らせる(既定 0 = 従来と同じ)。
#     OCCLUSION=0 なら --occlusion を付けない対照(既定 1)。REAR / INDOOR / TTC との併用、JUDGE_SEEDS が 0 でない、選定範囲が 21–120 と重なる、SIM_EXTRA / TRACKER_EXTRA の --seeds / --seed-start は INDOOR=1 と同じくエラー(終了 2)。
#     $OUT_DIR/blind_all.csv に構成・範囲を先頭に付けた --blind-csv の結合を出す。OUT_DIR は別にすること
#   SUDDEN=1 なら全 6 シナリオの代わりに急に止まる・振り返る人 3 本(--scenario sudden: sudden_stop / sudden_turn / sudden_reverse。記録のみ)だけを、全構成に --event-csv を付けて走らせる(既定 0 = 従来と同じ)。
#     REAR / INDOOR / BLIND / TTC との併用、JUDGE_SEEDS が 0 でない、選定範囲が 21–120 と重なる、SIM_EXTRA / TRACKER_EXTRA の --seeds / --seed-start は INDOOR=1 と同じくエラー(終了 2)。
#     $OUT_DIR/event_all.csv に構成・範囲を先頭に付けた --event-csv の結合を出す。OUT_DIR は別にすること
#   ELEVATOR=1 なら全 6 シナリオの代わりにエレベーター前・出口の流れ 1 本(--scenario elevator: elevator_flow。候補 6。記録のみ)だけを走らせる(既定 0 = 従来と同じ)。
#     REAR / INDOOR / BLIND / SUDDEN / TTC との併用、JUDGE_SEEDS が 0 でない、選定範囲が 21–120 と重なる、SIM_EXTRA / TRACKER_EXTRA の --seeds / --seed-start は INDOOR=1 と同じくエラー(終了 2)。OUT_DIR は別にすること
#   SPEEDMIX=1 なら既定 6 本(--scenario all)の全構成に --ped-speed-mix on を付けて走らせる(候補 8。速さの違う人。記録のみ。既定 0 = 従来と同じ。conditions.txt に ped_speed_mix: on)。
#     REAR / INDOOR / BLIND / SUDDEN / ELEVATOR / TTC との併用、JUDGE_SEEDS が 0 でない、選定範囲が 21–120 と重なる、SIM_EXTRA / TRACKER_EXTRA の --seeds / --seed-start は INDOOR=1 と同じくエラー(終了 2、OUT_DIR を消す前)。
#     比較元(off)は SPEEDMIX を付けず、OUT_DIR を別にして同じシード範囲を 1 回走らせる
#   TTC=1 なら sfm2(従来どおり $OUT_DIR 直下)の後に、同じシード・同じ 5 構成を --ped-model ttc で $OUT_DIR/ttc/ にも出す(既定 0 = 従来と同じ。TTC の副次確認。
#     SIM_EXTRA に --ped-model を入れる、JUDGE_SEEDS が 0 でない、選定範囲が 21–120 と重なる、はエラー(終了 2、OUT_DIR を消す前))。変更前後の比較(tools/ttc_check.py)は、変更前後を別の OUT_DIR(例: build/eval64/before、build/eval64/after)にする。
#     同じ OUT_DIR を指すと前の結果が消える(OUT_DIR は毎回 rm -rf する)
#   JOBS(既定 1 = 直列)= 構成×範囲のジョブを最大 JOBS 本まで(範囲ごとに完了を待って)並列に走らせる。SIM_EXTRA / TRACKER_EXTRA に出力先の指定があると JOBS>1 はエラー(終了 2)。出力(行順を含む)は直列と同じ。1 本でも失敗すれば非 0 で終了
# 同じ条件で 2 回実行すると M12 列以外は一致する(M12 は実測時間)。
set -euo pipefail
cd "$(dirname "$0")/.."
SELECT_SEEDS="${SELECT_SEEDS:-20}"; SELECT_START="${SELECT_START:-1}"
JUDGE_SEEDS="${JUDGE_SEEDS:-100}";  JUDGE_START="${JUDGE_START:-21}"
OUT_DIR="${OUT_DIR:-build/eval}"
CONTACTS="${CONTACTS:-0}"
REAR="${REAR:-0}"
INDOOR="${INDOOR:-0}"
BLIND="${BLIND:-0}"
OCCLUSION="${OCCLUSION:-1}"
SUDDEN="${SUDDEN:-0}"
ELEVATOR="${ELEVATOR:-0}"
SPEEDMIX="${SPEEDMIX:-0}"
if [ "$SPEEDMIX" != 0 ] && [ "$SPEEDMIX" != 1 ]; then echo "error: SPEEDMIX は 0 か 1" >&2; exit 2; fi
if [ "$SPEEDMIX" = 1 ] && { [ "$INDOOR" = 1 ] || [ "$REAR" = 1 ] || [ "$BLIND" = 1 ] || [ "$SUDDEN" = 1 ] || [ "$ELEVATOR" = 1 ] || [ "${TTC:-0}" = 1 ]; }; then echo "error: SPEEDMIX=1 は INDOOR / REAR / BLIND / SUDDEN / ELEVATOR / TTC と併用できない" >&2; exit 2; fi
if [ "$SPEEDMIX" = 1 ]; then SPEED_ARG="--ped-speed-mix on"; else SPEED_ARG=""; fi
if [ "$ELEVATOR" = 1 ] && { [ "$INDOOR" = 1 ] || [ "$REAR" = 1 ] || [ "$BLIND" = 1 ] || [ "$SUDDEN" = 1 ] || [ "${TTC:-0}" = 1 ]; }; then echo "error: ELEVATOR=1 は INDOOR / REAR / BLIND / SUDDEN / TTC と併用できない" >&2; exit 2; fi
if [ "$SUDDEN" = 1 ] && { [ "$INDOOR" = 1 ] || [ "$REAR" = 1 ] || [ "$BLIND" = 1 ] || [ "${TTC:-0}" = 1 ]; }; then echo "error: SUDDEN=1 は INDOOR / REAR / BLIND / TTC と併用できない" >&2; exit 2; fi
if [ "$INDOOR" = 1 ] && [ "$REAR" = 1 ]; then echo "error: INDOOR=1 と REAR=1 は併用できない" >&2; exit 2; fi
if [ "$BLIND" = 1 ] && { [ "$INDOOR" = 1 ] || [ "$REAR" = 1 ] || [ "${TTC:-0}" = 1 ]; }; then echo "error: BLIND=1 は INDOOR / REAR / TTC と併用できない" >&2; exit 2; fi
if [ "$OCCLUSION" != 0 ] && [ "$OCCLUSION" != 1 ]; then echo "error: OCCLUSION は 0 か 1" >&2; exit 2; fi
if [ "$REAR" = 1 ]; then SCEN=rear; elif [ "$INDOOR" = 1 ]; then SCEN=indoor; elif [ "$BLIND" = 1 ]; then SCEN=blind; elif [ "$SUDDEN" = 1 ]; then SCEN=sudden; elif [ "$ELEVATOR" = 1 ]; then SCEN=elevator; else SCEN=all; fi
if [ "$BLIND" = 1 ] && [ "$OCCLUSION" = 1 ]; then OCC_ARG="--occlusion"; else OCC_ARG=""; fi
SIM_EXTRA="${SIM_EXTRA:-}"
TRACKER_EXTRA="${TRACKER_EXTRA:-}"
TTC="${TTC:-0}"
JOBS="${JOBS:-1}"
if ! [[ "$JOBS" =~ ^[1-9][0-9]{0,2}$ ]]; then echo "error: JOBS は正の整数: '$JOBS'" >&2; exit 2; fi
# 引数の検査は OUT_DIR を消す前に行う
if [ "$JOBS" -gt 1 ] && [[ " $SIM_EXTRA $TRACKER_EXTRA " =~ [[:space:]]--(csv|trace|contacts-csv|wall-csv|blind-csv|event-csv)([[:space:]=]) ]]; then
  echo "error: JOBS>1 では SIM_EXTRA / TRACKER_EXTRA に出力先の指定(--csv / --trace / --*-csv)を入れない(全ジョブが同じファイルに書く)" >&2; exit 2
fi
if [ "$TTC" = 1 ] && [[ " $SIM_EXTRA " == *" --ped-model"* ]]; then
  echo "error: TTC=1 と SIM_EXTRA 内の --ped-model は併用できない" >&2; exit 2
fi
# TTC=1 は判定範囲(21–120)を使わない: 判定範囲を回さず(JUDGE_SEEDS=0)、選定範囲も 21–120 と重ねない(TTC の副次確認の規則)
if [ "$TTC" = 1 ] || [ "$INDOOR" = 1 ] || [ "$BLIND" = 1 ] || [ "$SUDDEN" = 1 ] || [ "$ELEVATOR" = 1 ] || [ "$SPEEDMIX" = 1 ]; then
  if [ "$SPEEDMIX" = 1 ]; then WHO=SPEEDMIX=1; elif [ "$TTC" = 1 ]; then WHO=TTC=1; elif [ "$BLIND" = 1 ]; then WHO=BLIND=1; elif [ "$SUDDEN" = 1 ]; then WHO=SUDDEN=1; elif [ "$ELEVATOR" = 1 ]; then WHO=ELEVATOR=1; else WHO=INDOOR=1; fi
  # SIM_EXTRA / TRACKER_EXTRA で --seed-start / --seeds を渡すと、上の範囲の検査を迂回して 21–120 を回せるので止める(--seeds=N の形も)
  if [[ " $SIM_EXTRA $TRACKER_EXTRA " =~ [[:space:]]--(seeds|seed-start)([[:space:]=]) ]]; then
    echo "error: $WHO では SIM_EXTRA / TRACKER_EXTRA に --seed-start / --seeds を入れない(SELECT_START / SELECT_SEEDS を使う)" >&2; exit 2
  fi
  # 範囲の計算の前に、10 進の非負整数(先頭 0 なし、9 桁以下)だけを通す。SELECT_SEEDS は 1 以上
  for v in SELECT_START SELECT_SEEDS JUDGE_SEEDS; do
    if ! [[ "${!v}" =~ ^(0|[1-9][0-9]{0,8})$ ]]; then
      echo "error: $WHO では $v は 10 進の非負整数(先頭 0 なし、9 桁以下)にする: '${!v}'" >&2; exit 2
    fi
  done
  if [ "$SELECT_SEEDS" = 0 ]; then echo "error: $WHO では SELECT_SEEDS は 1 以上" >&2; exit 2; fi
  if [ "$JUDGE_SEEDS" != 0 ]; then
    echo "error: $WHO は JUDGE_SEEDS=0 が必要(判定範囲で回さない)" >&2; exit 2
  fi
  if [ "$SELECT_START" -le 120 ] && [ $((SELECT_START+SELECT_SEEDS-1)) -ge 21 ]; then
    echo "error: $WHO の選定範囲 $SELECT_START..$((SELECT_START+SELECT_SEEDS-1)) が判定範囲 21–120 と重なる" >&2; exit 2
  fi
fi

GIT_SHA="$(git rev-parse HEAD)"
if [ -n "$(git status --porcelain -- social_nav_core 2>/dev/null)" ]; then GIT_SHA="$GIT_SHA-dirty"; fi
if [ -n "$(git status --porcelain -- social_nav_core 2>/dev/null)" ]; then GIT_DIRTY=yes; else GIT_DIRTY=no; fi
# 再現性の記録(同じイメージ ID・コミット・CPU アーキテクチャなら一致。違えば一致は保証しない)。イメージ ID はホスト側で取る
docker image inspect social_nav-dev:jazzy >/dev/null 2>&1 || docker compose -f docker/compose.yaml build dev >/dev/null 2>&1 || true   # 無ければ 1 回だけ build(build のたびに ID が変わるため毎回はしない)
IMAGE_ID="$(docker image inspect social_nav-dev:jazzy --format '{{.Id}}' 2>/dev/null || echo unknown)"
rm -rf "$OUT_DIR"; mkdir -p "$OUT_DIR"

docker compose -f docker/compose.yaml run --rm \
  -e GIT_SHA="$GIT_SHA" -e GIT_DIRTY="$GIT_DIRTY" -e IMAGE_ID="$IMAGE_ID" -e OUT="src/social_nav/$OUT_DIR" \
  -e SELECT_SEEDS="$SELECT_SEEDS" -e SELECT_START="$SELECT_START" \
  -e JUDGE_SEEDS="$JUDGE_SEEDS" -e JUDGE_START="$JUDGE_START" -e CONTACTS="$CONTACTS" -e SCEN="$SCEN" -e BLIND="$BLIND" -e SUDDEN="$SUDDEN" -e OCC_ARG="$OCC_ARG" -e SPEED_ARG="$SPEED_ARG" -e SIM_EXTRA="$SIM_EXTRA" -e TRACKER_EXTRA="$TRACKER_EXTRA" -e TTC="$TTC" -e JOBS="$JOBS" \
  dev bash -lc '
  set -euo pipefail
  cmake -S src/social_nav/social_nav_core -B build/rel -DCMAKE_BUILD_TYPE=Release >/dev/null
  cmake --build build/rel -j --target social_nav_sim >/dev/null
  CACHE=build/rel/CMakeCache.txt
  BT=$(sed -n "s/^CMAKE_BUILD_TYPE:[A-Z]*=//p" $CACHE)
  CXX=$(sed -n "s/^CMAKE_CXX_COMPILER:[A-Z]*=//p" $CACHE)
  CXXV=$("$CXX" --version | head -n1)
  STDLIB=$(dpkg-query -W -f="libstdc++6 \${Version}" libstdc++6 2>/dev/null || echo libstdc++)
  # run_set <出力先> <追加引数>: conditions 出力 → 構成ループ → 結合 CSV。追加引数は sim の末尾に足す(既定の sfm2 は空)
    run_set() {
    local OUT="$1" PEDARG="$2"
    {
      echo "git_sha: $GIT_SHA"
      echo "git_dirty: $GIT_DIRTY (social_nav_core/ 以下)"
      echo "docker_image_id: $IMAGE_ID"
      echo "arch: $(uname -m)"
      echo "build_type: $BT"
      echo "compiler: $CXXV ($CXX)"
      echo "stdlib: $STDLIB"
      echo "host: $(uname -srm)"
      echo "seeds: select=$SELECT_START..$((SELECT_START+SELECT_SEEDS-1)) judge=$( [ "$JUDGE_SEEDS" -gt 0 ] && echo "$JUDGE_START..$((JUDGE_START+JUDGE_SEEDS-1))" || echo none )"
      echo "binary: build/rel/social_nav_sim (scenario=$SCEN)"
      if [ -n "$PEDARG" ]; then echo "ped_model: ttc"; fi
      if [ -n "$SPEED_ARG" ]; then echo "ped_speed_mix: on"; fi
      if [ "$BLIND" = 1 ]; then echo "occlusion: ${OCC_ARG:-off}"; fi
      if [ -n "$TRACKER_EXTRA" ]; then echo "tracker_extra: $TRACKER_EXTRA"; fi
    } > $OUT/conditions.txt
    cat $OUT/conditions.txt
    declare -A CFG=(
      [social]="--controller social"
      [baseline]="--controller baseline --no-tactical"
      [no_tactical]="--controller social --no-tactical"
      [use_tracker]="--controller social --use-tracker${TRACKER_EXTRA:+ $TRACKER_EXTRA}"
      [ignore_robot]="--controller social --peds-ignore-robot"
    )
    ORDER="social baseline no_tactical use_tracker ignore_robot"
    RANGES="select:$SELECT_START:$SELECT_SEEDS"
    if [ "$JUDGE_SEEDS" != 0 ]; then RANGES="$RANGES judge:$JUDGE_START:$JUDGE_SEEDS"; fi
    cp $OUT/conditions.txt $OUT/summary.txt
    # 構成 × 範囲を最大 JOBS 本まで並列に走らせる(各ジョブは自分の $f.csv / $f.txt 等だけに書く)。結合は下で固定順に行う
    rm -f $OUT/*.failed
    # 範囲ごとに wait して失敗を確認してから次の範囲へ進む。失敗を見たら新規起動を止め、sim の終了コードをそのまま返す
    for r in $RANGES; do
      IFS=: read -r rname start n <<< "$r"
      for c in $ORDER; do
        f=$OUT/${c}_${rname}
        EXTRA=""
        if [ "$CONTACTS" = 1 ]; then EXTRA="--contacts-csv ${f}_contacts.csv"; fi
        if [ "$BLIND" = 1 ]; then EXTRA="$EXTRA $OCC_ARG --blind-csv ${f}_blind.csv"; fi
        if [ "$SUDDEN" = 1 ]; then EXTRA="$EXTRA --event-csv ${f}_event.csv"; fi
        while [ "$(jobs -rp | wc -l)" -ge "$JOBS" ]; do wait -n || true; done
        if compgen -G "$OUT/*.failed" >/dev/null; then break; fi
        { build/rel/social_nav_sim --scenario $SCEN --seeds $n --seed-start $start ${CFG[$c]} $SIM_EXTRA $SPEED_ARG $PEDARG --csv $f.csv $EXTRA > $f.txt || echo $? > $f.failed; } &
      done
      wait
      if compgen -G "$OUT/*.failed" >/dev/null; then
        for ff in $OUT/*.failed; do rc=$(cat $ff); break; done
        echo "error: sim が失敗(終了 $rc): $(ls $OUT/*.failed)" >&2; exit "${rc:-1}"
      fi
    done
    for r in $RANGES; do
      IFS=: read -r rname start n <<< "$r"
      for c in $ORDER; do
        f=$OUT/${c}_${rname}
        { echo; echo "## config=$c range=$rname seeds=$start..$((start+n-1)) args: ${CFG[$c]}"; cat $f.txt; } >> $OUT/summary.txt
      done
    done
    # 結合 CSV: 先頭に条件列を付ける。列名は sim の CSV をそのまま使う
    first=1
    for r in $RANGES; do
      IFS=: read -r rname start n <<< "$r"
      for c in $ORDER; do
        f=$OUT/${c}_${rname}.csv
        if [ $first = 1 ]; then
          echo "git_sha,build_type,compiler,config,range,$(head -n1 $f)" > $OUT/all.csv; first=0
        fi
        tail -n +2 $f | sed "s|^|$GIT_SHA,$BT,\"$CXXV\",$c,$rname,|" >> $OUT/all.csv
      done
    done
    if [ "$BLIND" = 1 ]; then   # --blind-csv の結合(config,range を先頭に付ける)
      first=1
      for r in $RANGES; do
        IFS=: read -r rname start n <<< "$r"
        for c in $ORDER; do
          f=$OUT/${c}_${rname}_blind.csv
          if [ $first = 1 ]; then echo "config,range,$(head -n1 $f)" > $OUT/blind_all.csv; first=0; fi
          tail -n +2 $f | sed "s|^|$c,$rname,|" >> $OUT/blind_all.csv
        done
      done
    fi
    if [ "$SUDDEN" = 1 ]; then   # --event-csv の結合(config,range を先頭に付ける)
      first=1
      for r in $RANGES; do
        IFS=: read -r rname start n <<< "$r"
        for c in $ORDER; do
          f=$OUT/${c}_${rname}_event.csv
          if [ $first = 1 ]; then echo "config,range,$(head -n1 $f)" > $OUT/event_all.csv; first=0; fi
          tail -n +2 $f | sed "s|^|$c,$rname,|" >> $OUT/event_all.csv
        done
      done
    fi
    }
  run_set "$OUT" ""
  if [ "$TTC" = 1 ]; then
    mkdir -p $OUT/ttc
    run_set "$OUT/ttc" "--ped-model ttc"
  fi
'
echo "wrote $OUT_DIR/ (summary.txt, all.csv, per-config *.txt / *.csv, conditions.txt)"
