# 開発環境（Docker）

    docker compose -f docker/compose.yaml build
    docker compose -f docker/compose.yaml run --rm dev bash

コンテナ内:

    # ROS 非依存コア
    cmake -S src/social_nav/social_nav_core -B build/core && cmake --build build/core && ctest --test-dir build/core --output-on-failure
    # ROS 2 パッケージ
    source /opt/ros/jazzy/setup.bash && colcon build --base-paths src/social_nav

## ローカル CI

マージ前に、リポジトリ直下で次を通すこと。

    ./ci_local.sh

イメージ build 済みのホスト（Docker 必須）から実行すると、コンテナ内で毎回クリーンビルドして次の 3 ステージを順に走らせ、最後に `CI OK`（失敗時は失敗ステージ名）を表示する。終了コードが 0 なら成功。

1. core: `social_nav_core` を単体で cmake build（警告をエラー扱い）し、ctest を実行
2. colcon: `social_nav_core`（`ros2/` があればそれも）を colcon build（警告をエラー扱い）/ test / test-result
3. smoke: `social_nav_bringup` の `smoke_launch.sh` が install 先にあるときだけ実行（無ければ skip）

smoke を省略するには `CI_SKIP_SMOKE=1 ./ci_local.sh`。

GitHub Actions（`.github/workflows/ci.yml`）もイメージを build して同じ `./ci_local.sh` を呼ぶだけなので、ローカルと同じ結果になる。例外として、変更ファイルがすべて `docs/**` か `*.md` の PR では、イメージの build と `./ci_local.sh` を飛ばし、チェック `ci` は成功で終わる。判定できないときや push、定期実行では全部実行する。

ccache: `ci_local.sh` は C++ のコンパイルに ccache を使う。保存先はリポジトリ直下の `.ccache/`（git 管理外、上限 1 GB、無ければ自動作成）。CI ではこのディレクトリを `actions/cache` で保存・復元する。イメージのタグは `SOCIAL_NAV_IMAGE` で上書きできる。
