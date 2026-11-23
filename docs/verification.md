# 検証済み範囲と未検証範囲

検証済み(コンテナ内・ヘッドレス):
- コア: 単体テスト(`ctest`)と簡易シミュレータ(ROS なし、別ブランチ/PR)
- ROS: 全パッケージのビルド(警告 0)、gtest(型変換、pluginlib でのプラグイン生成、BT ノードの戻り値、ノード起動)、bringup のファイル検査
- スモーク起動: `ros2/social_nav_bringup/test/smoke_launch.sh`。静的 TF(map -> odom -> base_link)だけを流して 60 秒以内に
  map_server, controller_server(MPPI + SocialCritic と SocialController の両方を configure), planner_server, behavior_server,
  bt_navigator(`navigate_social.xml` を読み込み), velocity_smoother, collision_monitor, local/global costmap(SocialLayer 含む)が
  すべて active になり、プラグイン・BT XML のロードエラーがログに無いこと。`people_tracker_node` は起動を確認(lifecycle ではない)。

  ```bash
  docker compose -f docker/compose.yaml run --rm dev bash -lc 'source /opt/ros/jazzy/setup.bash && \
    colcon build --base-paths src/social_nav --build-base build/colcon --install-base build/install && \
    source build/install/setup.bash && bash src/social_nav/ros2/social_nav_bringup/test/smoke_launch.sh'
  ```

既知の制限(戦術層): `SelectSocialBehavior` は初回 tick で購読を作るため、初回 tick までは PROCEED・100% になる(反応層は有効)。
また blackboard に `goal` が無い構成ではゴール変更時の selector の reset が行われない。

未検証(Gazebo などのシミュレータが無いため。設計書 8 章 2):
- 走行挙動: (b) `HoldPosition` が RUNNING の間に FollowPath が halt される挙動(ReactiveFallback 経由)、
  (c) `StatefulActionNode` が FAILURE を返し続ける使い方、(d) `/speed_limit` による MPPI の速度制限反映(0% を含む)
- ゴールを与えての実走行、人を流した状態での避け方・譲り方、collision_monitor の実動作(点群が来る状態)
- MPPI のチューニング: `SocialCritic.cost_weight`(初期 5.0)など他 critic との相対重みは未調整。シミュレータの数値を MPPI 構成の性能として扱わないこと
- RGBD 検出、Omni / Ackermann / 多角形フットプリント(API の見取り図のみ、実装なし)
