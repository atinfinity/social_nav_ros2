#pragma once
// 評価指標 M1–M14(design 6.3)。シミュレータ本体とは独立に、記録された値から計算する。
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <utility>
#include <memory>
#include <unordered_map>
#include <vector>

#include "social_nav_core/footprint.hpp"
#include "social_nav_core/sampling_controller.hpp"
#include "social_nav_core/tactical_selector.hpp"
#include "social_nav_core/types.hpp"

namespace social_nav {
namespace sim {

// M1b とみなす停止継続時間 [s]
constexpr double kM1StopSeconds = 1.0;
// M13 内訳の分類(排他・優先順: 過渡 → 取り違え → 加減速 → 定常)
constexpr double kM13TransientSeconds = 1.0;  // track 初出からこの時間未満は過渡
constexpr double kM13SwitchSeconds = 1.0;     // 最近傍 track id の変化からこの時間未満は取り違え
constexpr double kM13AccelThreshold = 0.5;    // 真値の加速度 [m/s^2] がこれを超えたら加減速
// M4 の方位別(診断)。人の進行方向から見たロボットの向き: 前 < 60°、側 60–120°、後 > 120°。人が静止(< 0.05 m/s)なら向きなし
constexpr double kM4MinPedSpeed = 0.05;
constexpr double kM4FrontRad = 1.0471975511965976;   // 60°
constexpr double kM4BackRad = 2.0943951023931953;    // 120°
enum M4Bearing { kM4Front = 0, kM4Side = 1, kM4Back = 2, kM4NoDir = 3 };
enum M13Class { kM13Transient = 0, kM13Switch = 1, kM13Accel = 2, kM13Steady = 3 };

struct RunMetrics {
  int m1_collisions{0};
  // M1 の内訳。M1a = m1_moving: 接触開始時に走行中(速度 >= 0.05 m/s)、または停止後 kM1StopSeconds 未満
  //             M1b = m1_stopped: 速度 < 0.05 m/s の状態が kM1StopSeconds 以上続いた後の接触
  // (m1_moving + m1_stopped == m1_collisions)
  int m1_moving{0};
  int m1_stopped{0};
  // M1a の内訳(合否には使わない。m1_moving_fast + m1_moving_recent == m1_moving)
  int m1_moving_fast{0};    // 接触開始時の速度 >= 0.05 m/s
  int m1_moving_recent{0};  // 接触開始時は停止中だが、停止後 kM1StopSeconds 未満
  double m2_min_clearance{1e9};
  double m3_intimate_time{0};
  double m4_personal_time{0};
  // 診断: M4 の内訳。どちらも合計が m4_personal_time に一致(M4 の定義は変えない)
  std::array<double, 4> m4_by_behavior{};   // PROCEED, FOLLOW, YIELD, WAIT
  std::array<double, 4> m4_by_bearing{};    // M4Bearing: 前, 側, 後, 向きなし
  double m5_reach_time{0};           // 未到達なら timeout の値
  bool reached{false};               // M6 は run ごとに !reached
  double m7_path_ratio{0};
  double m8_stopped_time{0};
  int m9_transitions{0};
  std::array<double, 4> m10_dwell{};  // PROCEED, FOLLOW, YIELD, WAIT
  int m11_safety_stops{0};
  double m12_mean_ms{0}, m12_max_ms{0};
  double m13_sumsq{0};                // ‖v̂ − v‖^2 の和(集計側でプールして RMS)
  long m13_count{0};
  // M13 の内訳(合否には使わない)。各配列を M13Class で引く。合計は m13_sumsq / m13_count に一致
  std::array<double, 4> m13_class_sumsq{};
  std::array<long, 4> m13_class_count{};
  // 診断: 0.5 m 以内に確定 track が 2 つ以上ある真値・周期の数(M14 の定義は変えない)
  long dup_steps{0};
  // 診断: 人同士の接触数。外形間距離 < 0 の連続区間を 1 回と数える(M1 とは別、ロボットは含まない)
  int ped_ped_contacts{0};
  double m13_rms{0};
  int m14_id_switches{0};
};

// 接触した歩行者に対応する確定 track の情報(診断用。tracker を使わない構成では既定値のまま)
struct PedTrackInfo {
  bool has_track{false};        // 確定 track があるか
  double vel_err{std::nan("")}; // 推定速度と真値の差のノルム [m/s]
  int m13_class{-1};            // M13Class(-1 = なし)
  double track_age{std::nan("")};   // track 齢 [s]
};

// 直近の recordTracks で、1 つの確定 track が対応した真値(M13 本体と同じ対応付け)
struct TrackMatch {
  uint32_t track_id{0};
  uint32_t truth_id{0};
  Vec2 truth_vel;      // 対応した真値の速度
  double dist{0};      // track と真値の距離 [m]
  PedTrackInfo info;   // vel_err / M13 分類 / track 齢(has_track=true)
};

// 診断用の接触イベント(--contacts-csv。指標の値には影響しない)
struct ContactEvent {
  double t{0};                 // 接触開始の時刻 [s](recordPhysics の累積)
  uint32_t ped_id{0};
  double robot_speed{0};       // 安全停止適用後
  double cmd_speed{0};         // 制御器の指令(安全停止前)
  double cmd_vx{0};            // 指令の vx(符号付き、安全停止前)
  double neg_cmd_frac_1s{0};   // 直前 1 s のうち指令 vx < 0 だったステップの割合
  double neg_applied_frac_1s{0};   // 直前 1 s のうち安全停止後の vx < 0 だったステップの割合
  double speed_ratio{1};       // 戦術層の速度比
  double stopped_dur{0};       // metrics の stopped_dur_(単一閾値 0.05)
  std::array<double, 3> hyst_dur{};   // ヒステリシス停止継続(入 <0.05、出 >= 0.10 / 0.15 / 0.20)
  bool safety_stop{false};     // このステップで安全停止
  double safety_frac_1s{0};    // 直前 1 s のうち安全停止だったステップの割合
  double max_speed_1s{0};      // 直前 1 s のロボット速度の最大
  int behavior{0};
  double behavior_age{0};      // 現在の戦術状態に入ってからの時間
  double close_robot{0};       // 接触時の接近速度のうちロボットの分 v_r·n(n はロボット→人)
  double close_ped{0};         // 人の分 -v_p·n
  double approach_robot_1s{0}; // 直前 1 s の距離短縮のうちロボットの分 [m]
  double approach_ped_1s{0};   // 人の分 [m]
  double ped_speed{0};
  double bearing{0};           // ロボット進行方向から見た人の方位 [rad]
  double prev_gap{-1};         // 同じ人との前回接触の終了からの時間(なければ -1)
  int steps{0};                // 接触が続いたステップ数
  double min_clear{0};         // 接触中の最小外形間距離
  PedTrackInfo track;          // 接触開始時点の、この人に対応する track 情報
  int cls{0};                  // 0 = M1a_fast, 1 = M1a_recent, 2 = M1b
  ControlDiag diag_1s, diag_05s;   // 接触の 1.0 s 前 / 0.5 s 前以前で最も新しい制御周期の診断(なければ valid=false)
};

class MetricsRecorder {
 public:
  MetricsRecorder(std::shared_ptr<const Footprint> footprint, double intimate_dist = 0.45,
                  double personal_dist = 1.2);

  // 物理ステップごと(sim は dt=0.02)。peds は真値。pos/radius/vel/id を使う
  void recordPhysics(const Pose2D& robot, const Twist2D& vel, const std::vector<Agent>& peds,
                     bool safety_stop, double dt);
  // 制御周期ごと。最初の呼び出しは遷移に数えない
  void recordBehavior(Behavior b, double dt);
  void recordControlTime(double ms);
  // 追跡出力と真値の比較(M13, M14)。確定 track を最近傍の真値(0.5 m 以内)に対応付ける
  // dt は前回呼び出しからの経過時間(内訳の時間判定用。累積して内部時計にする)
  void recordTracks(const std::vector<Agent>& truth, const TrackedPeople& tracks, double dt = 0.1);
  RunMetrics finish(bool reached, double reach_time, double straight_length) const;
  // 診断: 接触イベントを記録する(既定は無効)。setControlContext は制御周期ごと
  void enableContactLog() { log_contacts_ = true; }
  void setControlContext(double cmd_speed, double speed_ratio, const ControlDiag& diag = {},
                         double cmd_vx = 0.0) {
    cmd_speed_ = cmd_speed; cmd_vx_ = cmd_vx; speed_ratio_ = speed_ratio;
    diag_hist_.emplace_back(clock_, diag);
    while (diag_hist_.size() > 30) diag_hist_.pop_front();
  }
  // 本体(recordPhysics)は track_info_ を直接読む。これは主にテストと診断のための公開 API。
  // 直近の recordTracks で、真値の歩行者 id に対応した track の情報(M13 本体と同じ対応付け・分類)。
  // 対応する track がなければ has_track=false。同じ真値に複数 track が対応したら距離が最小のもの。
  // vel_err は tracker の推定速度と真値の差(呼び出し側が速度を差し替える前の値)。接触開始時に ContactEvent::track へ写す
  PedTrackInfo pedTrackInfo(uint32_t ped_id) const {
    auto it = track_info_.find(ped_id);
    return it == track_info_.end() ? PedTrackInfo{} : it->second;
  }
  // 直近の recordTracks で、真値に対応した track ごとの結果(対応しなかった track は含まない)
  const std::vector<TrackMatch>& trackMatches() const { return track_matches_; }
  const std::vector<ContactEvent>& contacts() const { return contacts_; }

 private:
  std::shared_ptr<const Footprint> fp_;
  double intimate_, personal_;
  RunMetrics m_;
  std::unordered_map<uint32_t, bool> in_collision_;
  std::unordered_map<uint64_t, bool> pp_in_contact_;   // 人ペア(小さい id << 32 | 大きい id)-> 接触中
  std::unordered_map<uint32_t, uint32_t> last_track_id_;
  double track_clock_{0};                                    // recordTracks の累積時間
  std::unordered_map<uint32_t, double> track_first_seen_;    // track id -> 初出時刻
  std::unordered_map<uint32_t, double> truth_switch_time_;   // 真値 id -> 最近傍 track id の最終変化時刻
  std::unordered_map<uint32_t, Vec2> truth_prev_vel_;        // 真値 id -> 前回速度
  double stopped_dur_{0};            // 速度 < 0.05 m/s が続いている時間(直前のステップまで)
  bool safety_active_{false};
  bool has_prev_{false};
  Pose2D prev_pose_;
  double path_len_{0};
  bool has_behavior_{false};
  Behavior last_behavior_{Behavior::PROCEED};
  bool log_contacts_{false};
  double cmd_speed_{0}, cmd_vx_{0}, speed_ratio_{1};
  double clock_{0}, behavior_since_{0};
  std::array<double, 3> hyst_dur_{};
  std::array<bool, 3> hyst_stopped_{};
  std::deque<std::pair<double, bool>> hist_;   // 直前 1 s の (速度, 安全停止)
  std::deque<std::pair<bool, bool>> neg_hist_; // 直前 1 s の (指令 vx < 0, 適用 vx < 0)
  std::unordered_map<uint32_t, std::deque<std::pair<double, double>>> close_hist_;   // 人ごとの (ロボット分, 人分) [m]
  std::unordered_map<uint32_t, int> open_contact_;      // 人 id -> contacts_ の添字
  std::unordered_map<uint32_t, double> last_contact_end_;
  std::vector<ContactEvent> contacts_;
  std::unordered_map<uint32_t, PedTrackInfo> track_info_;   // 真値 id -> 直近の recordTracks の対応
  std::vector<TrackMatch> track_matches_;
  std::deque<std::pair<double, ControlDiag>> diag_hist_;   // (制御周期の開始時刻, 診断)
  double ctrl_ms_sum_{0};
  long ctrl_n_{0};
};

}  // namespace sim
}  // namespace social_nav
