# ビルドとテスト

Docker 環境の詳細は [docker/README.md](../docker/README.md)。CI と同じ手順をローカルで流すには [ci_local.sh](../ci_local.sh)。

## Docker

```bash
docker compose -f docker/compose.yaml run --rm dev bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  colcon build --base-paths src/social_nav --build-base build/colcon --install-base build/install &&
  colcon test  --base-paths src/social_nav --build-base build/colcon --install-base build/install;
  colcon test-result --test-result-base build/colcon'
```

## コアだけ

コアだけ(ROS なし): `cmake -S social_nav_core -B build/core && cmake --build build/core -j && ctest --test-dir build/core`。
