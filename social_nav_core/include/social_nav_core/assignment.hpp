#pragma once
#include <vector>

namespace social_nav {

// 被約費用付きの逐次最短増加路法(多始点 Dijkstra)による最小コスト割り当て(矩形可、禁止組あり、決定的)。
// cost[i][j] は行 i を列 j に割り当てるコスト(行ごとに同じ列数)。|cost| は 1e5 未満を想定。
// forbidden が空でなければ forbidden[i][j] が true の組は割り当てない(cost は無視)。
// 割り当て可能な組の数を最大にし、その中で総コスト最小の割り当てを返す。
// 戻り値は行ごとの列番号。未割当は -1。同じ入力に対して常に同じ結果(入力だけの関数)。
// 同コストの規則: 緩和は厳密な改善のみ(未割当行は昇順、以後は確定順)、列の確定は (距離, 列番号) の小さい順、
// 終点は終点までの距離最小で同値なら先に確定した列。全要素が等しい n x n は a[i] == i。
// 同コストの最適解が複数あるとき、他の実装と同じ解になるとは限らない。
// 文献: Edmonds-Karp 1972 (J. ACM 19(2)), Dijkstra 1959 (Numer. Math. 1), Berge 1957 (PNAS 43(9)),
//       Ahuja-Magnanti-Orlin "Network Flows" (Prentice Hall, 1993)。詳細は assignment.cpp。
std::vector<int> solveAssignment(const std::vector<std::vector<double>>& cost,
                                 const std::vector<std::vector<bool>>& forbidden = {});

}  // namespace social_nav
