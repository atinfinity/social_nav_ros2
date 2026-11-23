# costmap の動画(PR 用デモ)

SocialLayer 入りの Nav2 global costmap に、合成の人(3 人)を流した様子を動画にする。
**ロボットは静止、人は合成(`demo_people_publisher.py`)で、走行は検証していない。** 見せるのは「コストの山が人と一緒に動く」ことだけ。
動く 2 人は 0.25 m/s(SocialLayer は `max_speed_for_layer` = 0.3 m/s 超の人を描かないため)、1 人は静止。

```bash
# 前提: イメージ social_nav-video(シミュレータ側のブランチの docker/video.Dockerfile から作る)
ros2/social_nav_bringup/test/record_costmap_demo.sh     # リポジトリのルートから 1 回
```

dev コンテナ内でビルド → launch(people_tracker なし、静的 TF)→ rosbag(mcap)に録画 → `tools/render_costmap_bag.py` が
rawvideo にして → `social_nav-video` の ffmpeg が H.264 にする。出力は `build/video/costmap_demo.mp4`(640x480, 20 fps, 約 24 s)と
確認用の静止画 `costmap_a.png`, `costmap_b.png`。動画はコミットしない(`build/` は無視される)。
