#include "social_nav_core/assignment.hpp"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

// 被約費用(ポテンシャル)付きの逐次最短増加路法による最小コスト割り当て。
// 許可された組だけを辺に持つ 2 部グラフで、未割当の行すべてを距離 0 の始点として
// Dijkstra 法で最短の増加路を 1 本ずつ求め、反転する。増加路が無くなった時点で割り当て数は最大
// (Berge の定理)で、最短路を選び続けるので、その数の中で総コストも最小になる。
// 禁止組は辺を作らないので big-M は使わない。
// 被約費用は cost - row_pot - col_pot(0 で切り上げ。許容誤差は使わない)。
// 未割当の列から終点への辺は被約費用 col_pot - sink_pot で表し、最短の終点を早期に打ち切って決める。
// 最小頂点の取り出しは、辺が疎なら二分ヒープ、密なら線形走査(順序が同じなので結果は同一)。
//
// 同コストの規則(結果は入力だけの関数。乱数・ハッシュ順・アドレスに依存しない):
//  - 距離の緩和は厳密な改善(<)のみ。未割当行を行番号の昇順に、以後は列の確定順に緩和する。
//  - 列の確定は (距離, 列番号) の辞書順で小さい方から。
//  - 終点は終点までの距離が最小の未割当列、同値なら先に確定した列。
//  - 同コストの最適解が複数あるとき、他の実装と同じ解になるとは限らない。
//  - 固定例: 全要素が等しい n x n は a[i] == i。
//
// 文献:
//  - J. Edmonds, R. M. Karp, "Theoretical improvements in algorithmic efficiency for network
//    flow problems", J. ACM 19(2):248-264, 1972.
//  - E. W. Dijkstra, "A note on two problems in connexion with graphs",
//    Numerische Mathematik 1:269-271, 1959.
//  - C. Berge, "Two theorems in graph theory", PNAS 43(9):842-844, 1957.
//  - R. K. Ahuja, T. L. Magnanti, J. B. Orlin, "Network Flows: Theory, Algorithms, and
//    Applications", Prentice Hall, 1993.(逐次最短路法)
//
// 計算量: 増加 1 回が O(辺数 log) 〜 O(辺数 + 列数^2)、全体で割り当て数を掛けた程度。

namespace social_nav {

std::vector<int> solveAssignment(const std::vector<std::vector<double>>& cost,
                                 const std::vector<std::vector<bool>>& forbidden) {
  const std::size_t nr = cost.size();
  std::vector<int> row_to_col(nr, -1);
  if (nr == 0) return row_to_col;
  const std::size_t nc = cost[0].size();
  if (nc == 0) return row_to_col;

  const double inf = std::numeric_limits<double>::infinity();
  std::vector<std::vector<std::size_t>> adj(nr);
  std::vector<double> col_pot(nc, inf);
  std::size_t edges = 0;
  for (std::size_t i = 0; i < nr; ++i)
    for (std::size_t j = 0; j < nc; ++j)
      if (forbidden.empty() || !forbidden[i][j]) {
        adj[i].push_back(j);
        col_pot[j] = std::min(col_pot[j], cost[i][j]);
        ++edges;
      }
  if (edges == 0) return row_to_col;
  double sink_pot = inf;
  for (std::size_t j = 0; j < nc; ++j) {
    if (col_pot[j] == inf) col_pot[j] = 0.0;  // 辺なし(到達しない)
    else sink_pot = std::min(sink_pot, col_pot[j]);
  }
  const bool use_heap = edges * 8 <= nr * nc;

  std::vector<int> col_to_row(nc, -1);
  std::vector<double> row_pot(nr, 0.0), dist(nc);
  std::vector<int> from_row(nc);
  std::vector<char> done(nc);
  using Item = std::pair<double, std::size_t>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> heap;

  for (;;) {
    std::fill(dist.begin(), dist.end(), inf);
    std::fill(done.begin(), done.end(), 0);
    std::fill(from_row.begin(), from_row.end(), -1);
    heap = decltype(heap)();

    auto relax = [&](std::size_t i, double base) {
      for (std::size_t j : adj[i]) {
        if (done[j] || static_cast<int>(j) == row_to_col[i]) continue;
        const double cand = base + std::max(0.0, cost[i][j] - row_pot[i] - col_pot[j]);
        if (cand < dist[j]) {
          dist[j] = cand;
          from_row[j] = static_cast<int>(i);
          if (use_heap) heap.emplace(cand, j);
        }
      }
    };
    for (std::size_t i = 0; i < nr; ++i)
      if (row_to_col[i] < 0) relax(i, 0.0);

    double best = inf;
    int end = -1;
    for (;;) {
      std::size_t b = nc;
      if (use_heap) {
        while (!heap.empty()) {
          const Item top = heap.top();
          heap.pop();
          if (done[top.second] || top.first != dist[top.second]) continue;
          b = top.second;
          break;
        }
      } else {
        for (std::size_t j = 0; j < nc; ++j)
          if (!done[j] && dist[j] < inf && (b == nc || dist[j] < dist[b])) b = j;
      }
      if (b == nc || dist[b] >= best) break;
      done[b] = 1;
      if (col_to_row[b] >= 0) {
        relax(static_cast<std::size_t>(col_to_row[b]), dist[b]);
      } else {
        const double t = dist[b] + std::max(0.0, col_pot[b] - sink_pot);
        if (t < best) { best = t; end = static_cast<int>(b); }
      }
    }
    if (end < 0) break;

    for (std::size_t j = 0; j < nc; ++j) {
      const double a = std::min(dist[j], best);
      col_pot[j] += a;
      if (col_to_row[j] >= 0) row_pot[static_cast<std::size_t>(col_to_row[j])] -= a;
    }
    sink_pot += best;

    int j = end;
    while (j >= 0) {
      const int i = from_row[static_cast<std::size_t>(j)];
      const int prev = row_to_col[static_cast<std::size_t>(i)];
      row_to_col[static_cast<std::size_t>(i)] = j;
      col_to_row[static_cast<std::size_t>(j)] = i;
      j = prev;
    }
  }
  return row_to_col;
}

}  // namespace social_nav
