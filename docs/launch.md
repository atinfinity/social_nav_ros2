# 起動

```bash
ros2 launch social_nav_bringup social_nav.launch.py -p      # 引数の一覧
ros2 launch social_nav_bringup social_nav.launch.py          # 既定: diff_drive / circle / 0.3 m / MPPI + SocialCritic
ros2 launch social_nav_bringup social_nav.launch.py controller:=social   # 比較用 SocialController
```

map -> odom -> base_link の TF、`points`(LiDAR 点群)、`odom` は外部から供給する前提(localization は起動しない)。
ロボット固有の値は launch 引数(`drive_type`, `footprint_type`, `robot_radius`, `footprint`)と
`ros2/social_nav_bringup/params/nav2_social_params.yaml` にだけ置く。実装済みの組み合わせは
`diff_drive` / `circle` のみで、それ以外を指定すると launch がエラーになる。
