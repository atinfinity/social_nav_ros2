#!/usr/bin/env bash
# build/video/costmap_demo.mp4 を 1 回で作る(ホストで実行)。
# 合成の人(3 人)を流した Nav2 の global costmap(SocialLayer 入り)を rosbag に録り、オフラインで描画する。
# ロボットは静止。人は合成。走行は検証していない(costmap の山が人に追従することだけを見せる)。
# 前提: docker、イメージ social_nav-video(`docker build -f docker/video.Dockerfile -t social_nav-video docker/`、別ブランチ)
# 出力: build/video/costmap_demo.mp4, costmap_a.png, costmap_b.png(動画はコミットしない)
set -euo pipefail
cd "$(dirname "$0")/../../.."
mkdir -p build/video
docker image inspect social_nav-video >/dev/null 2>&1 || { echo "social_nav-video が無い" >&2; exit 1; }

FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf
DT="drawtext=fontfile=$FONT:box=1:boxcolor=black@0.5:fontcolor=white"
VF="$DT:text='Nav2 global costmap + SocialLayer (synthetic people, static robot)':x=10:y=10:fontsize=16,\
$DT:text='color = cost (dark 0 .. red 100)   white ring = person and 1 s velocity   blue = robot':x=10:y=452:fontsize=13,\
$DT:text='t=%{eif\\:n/20\\:d} s':x=570:y=10:fontsize=16"

# 1. dev コンテナ: ビルド → 起動 → 録画 → 描画(標準出力は rawvideo だけ。他は標準エラー/ログへ)
docker compose -f docker/compose.yaml run -T --rm dev bash -lc '
  set -e
  exec 3>&1 1>&2                     # 以降の出力は標準エラーへ。rawvideo だけ fd 3
  set -m                             # 背景ジョブが SIGINT を無視しないようにする
  source /opt/ros/jazzy/setup.bash
  # 他のコンテナ(別の launch や smoke_launch.sh)の DDS と混ざらないようにする。混ざると
  # 別の people_tracker_node の空の TrackedPeople が bag に入り、動画の人の輪が明滅する
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=$((100 + RANDOM % 100))
  colcon build --base-paths src/social_nav --build-base build/colcon --install-base build/install \
    --packages-up-to social_nav_bringup > src/social_nav/build/video/colcon_build.log 2>&1
  source build/install/setup.bash
  V=src/social_nav/build/video
  rm -rf $V/bag
  # costmap の更新・配信を 10 Hz に上げた複製を使う(動画を滑らかにするため。既定の 1 Hz は触らない)
  sed -e "/^global_costmap:/,/^planner_server:/{s/update_frequency: 1.0/update_frequency: 10.0/;s/publish_frequency: 1.0/publish_frequency: 10.0/}" \
    src/social_nav/ros2/social_nav_bringup/params/nav2_social_params.yaml > $V/params_demo.yaml
  PIDS=""
  trap "for p in \$PIDS; do kill -INT \$p 2>/dev/null || true; done; sleep 1" EXIT
  ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom > $V/tf1.log 2>&1 & PIDS="$PIDS $!"
  ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link > $V/tf2.log 2>&1 & PIDS="$PIDS $!"
  ros2 launch social_nav_bringup social_nav.launch.py people_tracker:=false params_file:=$PWD/$V/params_demo.yaml \
    > $V/launch.log 2>&1 & PIDS="$PIDS $!"
  for i in $(seq 60); do
    ros2 lifecycle get /global_costmap/global_costmap 2>/dev/null | grep -q "active \[3\]" && break
    sleep 2
  done
  ros2 lifecycle get /global_costmap/global_costmap | grep -q "active \[3\]" || { echo "global_costmap が active にならない" ; tail -40 $V/launch.log; exit 1; }
  ros2 bag record -s mcap -o $V/bag /global_costmap/costmap /social_nav/tracked_people > $V/bag_record.log 2>&1 & BAG=$!
  sleep 3
  python3 src/social_nav/ros2/social_nav_bringup/test/demo_people_publisher.py 26
  sleep 1
  kill -INT $BAG; wait $BAG || true
  python3 src/social_nav/tools/render_costmap_bag.py $V/bag >&3
'  | docker run --rm -i -e VF="$VF" -v "$PWD":/work social_nav-video bash -c '
  ffmpeg -y -loglevel error -f rawvideo -pix_fmt rgb24 -s 640x480 -r 20 -i - -vf "$VF" \
    -c:v libx264 -pix_fmt yuv420p -crf 23 -preset medium -movflags +faststart /work/build/video/costmap_demo.mp4
  ffmpeg -y -loglevel error -ss 4  -i /work/build/video/costmap_demo.mp4 -frames:v 1 /work/build/video/costmap_a.png
  ffmpeg -y -loglevel error -ss 16 -i /work/build/video/costmap_demo.mp4 -frames:v 1 /work/build/video/costmap_b.png
'
echo "wrote build/video/costmap_demo.mp4 (costmap_a.png, costmap_b.png)"
