#!/usr/bin/env bash
# build/video/sim_demo.mp4 を 1 回で作る(COMPARE=ped なら build/video_ped/ped_compare.mp4)。corridor_oncoming / crossing / crowd を各 MAX_S 秒で打ち切って連結する。
# 流れ: sim の --trace → render_trace.py → ffmpeg(区間ごと)→ concat。区間ごとの静止画 seg_*.png も出す。
# 前提: docker、イメージ social_nav-video(`docker build -f docker/video.Dockerfile -t social_nav-video docker/`)
# 環境変数: SEED(既定 21)、MAX_S(既定 40)
#   COMPARE=indoor: 屋内 3 本(記録のみ)door_oncoming / side_door / group_oncoming の social / baseline 比較 build/video_indoor/indoor.mp4。
#   COMPARE=sudden: 急に止まる・振り返る人(候補 5。記録のみ)sudden_stop / sudden_turn / sudden_reverse の social / baseline 比較 build/video_sudden/sudden.mp4。SEED の既定 21 は判定範囲なので、必ず 21–120 の外を指定する(例 SEED=3042)。
#   COMPARE=elevator: エレベーター前・出口の流れ(候補 6。記録のみ)elevator_flow の social / baseline 比較 build/video_elevator/elevator.mp4。SEED の既定 21 は判定範囲なので、必ず 21–120 の外を指定する。
#   COMPARE=speedmix: 速さの違う人(候補 8。記録のみ)corridor_oncoming / crowd の off / on 比較(robot は social)build/video_speedmix/speedmix.mp4。SEED は 21–120 の外を必ず指定する。
#   COMPARE=blind: 見えない人(記録のみ)blind_corner の遮蔽 on / off 比較 build/video_blind/blind.mp4。左は --occlusion(social)、右は遮蔽なし(social)。
#   COMPARE=ped: 歩行者モデルの比較動画 build/video_ped/ped_compare.mp4。左 legacy、右 sfm2、同シード・同ロボット(social)。
#                シナリオは corridor_oncoming と crowd。未指定なら従来の social / baseline 比較(build/video)。
set -euo pipefail
cd "$(dirname "$0")/.."
SEED="${SEED:-21}"
MAX_S="${MAX_S:-40}"
COMPARE="${COMPARE:-}"
if [ "$COMPARE" = "ped" ]; then
  SCENARIOS="corridor_oncoming crowd"
  OUT=build/video_ped; OUTNAME=ped_compare.mp4
  L_TAG=legacy; R_TAG=sfm2
  L_ARGS="--controller social --ped-model legacy"; R_ARGS="--controller social --ped-model sfm2"
  L_TXT="LEFT - legacy peds, robot=social"; R_TXT="RIGHT - sfm2 peds, robot=social"
elif [ "$COMPARE" = "indoor" ]; then
  SCENARIOS="door_oncoming side_door group_oncoming"
  OUT=build/video_indoor; OUTNAME=indoor.mp4
  L_TAG=social; R_TAG=baseline
  L_ARGS="--controller social"; R_ARGS="--controller baseline --no-tactical"
  L_TXT="LEFT - social (tactical + social cost)"; R_TXT="RIGHT - baseline (people as obstacles)"
elif [ "$COMPARE" = "sudden" ]; then
  SCENARIOS="sudden_stop sudden_turn sudden_reverse"
  OUT=build/video_sudden; OUTNAME=sudden.mp4
  L_TAG=social; R_TAG=baseline
  L_ARGS="--controller social"; R_ARGS="--controller baseline --no-tactical"
  L_TXT="LEFT - social (tactical + social cost)"; R_TXT="RIGHT - baseline (people as obstacles)"
elif [ "$COMPARE" = "elevator" ]; then
  SCENARIOS="elevator_flow"
  OUT=build/video_elevator; OUTNAME=elevator.mp4
  L_TAG=social; R_TAG=baseline
  L_ARGS="--controller social"; R_ARGS="--controller baseline --no-tactical"
  L_TXT="LEFT - social (tactical + social cost)"; R_TXT="RIGHT - baseline (people as obstacles)"
elif [ "$COMPARE" = "speedmix" ]; then
  if [ "$SEED" -ge 21 ] && [ "$SEED" -le 120 ]; then
    echo "COMPARE=speedmix: set SEED outside the judge range 21-120" >&2; exit 2
  fi
  SCENARIOS="corridor_oncoming crowd"
  OUT=build/video_speedmix; OUTNAME=speedmix.mp4
  L_TAG=off; R_TAG=on
  L_ARGS="--controller social"; R_ARGS="--controller social --ped-speed-mix on"
  L_TXT="LEFT - speed mix off, robot=social"; R_TXT="RIGHT - speed mix on, robot=social"
elif [ "$COMPARE" = "blind" ]; then
  SCENARIOS="blind_corner"
  OUT=build/video_blind; OUTNAME=blind.mp4
  L_TAG=occ; R_TAG=noocc
  L_ARGS="--controller social --occlusion"; R_ARGS="--controller social"
  L_TXT="LEFT - social, walls block sight (--occlusion)"; R_TXT="RIGHT - social, people always visible"
elif [ -z "$COMPARE" ]; then
  SCENARIOS="corridor_oncoming crossing crowd"
  OUT=build/video; OUTNAME=sim_demo.mp4
  L_TAG=social; R_TAG=baseline
  L_ARGS="--controller social"; R_ARGS="--controller baseline --no-tactical"
  L_TXT="LEFT - social (tactical + social cost)"; R_TXT="RIGHT - baseline (people as obstacles)"
else
  echo "COMPARE must be empty, 'ped', 'indoor', 'blind', 'sudden', 'elevator' or 'speedmix'" >&2; exit 2
fi
mkdir -p "$OUT"

docker image inspect social_nav-video >/dev/null 2>&1 || docker build -f docker/video.Dockerfile -t social_nav-video docker/

# 1. sim をビルドして、シナリオごとに左右 2 本の trace を出す(コンテナ内。リポジトリは /ws/src/social_nav)
docker compose -f docker/compose.yaml run --rm dev bash -lc "
  set -e
  cmake -S src/social_nav/social_nav_core -B build/rel -DCMAKE_BUILD_TYPE=Release >/dev/null
  cmake --build build/rel -j --target social_nav_sim >/dev/null
  for sc in $SCENARIOS; do
    build/rel/social_nav_sim --scenario \$sc --seeds 1 --seed-start $SEED $L_ARGS \
        --trace src/social_nav/$OUT/trace_\${sc}_$L_TAG.txt >/dev/null
    build/rel/social_nav_sim --scenario \$sc --seeds 1 --seed-start $SEED $R_ARGS \
        --trace src/social_nav/$OUT/trace_\${sc}_$R_TAG.txt >/dev/null
  done
"

# 2. 区間ごとに描画して H.264 にし(1280x480, 20 fps)、連結する
FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf
LEG1="bar PROCEED=green FOLLOW=cyan YIELD=yellow WAIT=red | blue=robot orange=person red=contact ring=1.2 m"
LEG2="family=same-color dots + line | smartphone=magenta + phone mark | white arrow=velocity"
docker run --rm -v "$PWD":/work social_nav-video bash -c "
  set -e
  : > $OUT/concat.txt
  for sc in $SCENARIOS; do
    python3 -I tools/render_trace.py $OUT/trace_\${sc}_$L_TAG.txt $OUT/trace_\${sc}_$R_TAG.txt $MAX_S |
    ffmpeg -y -loglevel error -f rawvideo -pix_fmt rgb24 -s 1280x480 -r 20 -i - \
      -vf \"drawtext=fontfile=$FONT:text='$L_TXT':x=10:y=40:fontsize=22:fontcolor=white:box=1:boxcolor=black@0.5,\
drawtext=fontfile=$FONT:text='$R_TXT':x=650:y=40:fontsize=22:fontcolor=white:box=1:boxcolor=black@0.5,\
drawtext=fontfile=$FONT:text='scenario \$sc   seed $SEED   t=%{eif\\:n/20\\:d} s':x=10:y=448:fontsize=20:fontcolor=white,\
drawtext=fontfile=$FONT:text='$LEG1':x=10:y=400:fontsize=16:fontcolor=white,\
drawtext=fontfile=$FONT:text='$LEG2':x=10:y=422:fontsize=16:fontcolor=white\" \
      -c:v libx264 -pix_fmt yuv420p -crf 23 -preset medium -movflags +faststart $OUT/seg_\$sc.mp4
    echo \"file seg_\$sc.mp4\" >> $OUT/concat.txt
    ffmpeg -y -loglevel error -ss 12 -i $OUT/seg_\$sc.mp4 -frames:v 1 $OUT/seg_\${sc%%_*}.png
  done
  ffmpeg -y -loglevel error -f concat -safe 0 -i $OUT/concat.txt -c copy -movflags +faststart $OUT/$OUTNAME
"
echo "wrote $OUT/$OUTNAME"
