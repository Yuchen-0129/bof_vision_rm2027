#ifndef AUTO_AIM__OUTPOST_TARGET_HPP
#define AUTO_AIM__OUTPOST_TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <deque>
#include <list>
#include <string>
#include <vector>

#include "tasks/auto_aim/common/armor.hpp"
#include "tasks/auto_aim/auto_aim_track/solver.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{
// ============================================================================
// 前哨站专用跟踪器
// ============================================================================
//
// 为什么不复用整车 Target：
//   整车 Target 是 11 维（x vx y vy z vz a w r l h），把 r 和 w 都当自由变量估，
//   因为对方步兵的半径、转速、加减速都是未知的。前哨站不是这样——它的几何和
//   转速由规则写死，唯一未知的是**旋转方向**和**当前相位**。
//
//   把已知量交给 EKF 去"估"，等于白送它自由度去拟合观测噪声。实测
//   records/2026-07-31_05-45-29（24.7s、1042 帧有 target）：
//     - |w| p50 = 2.512 rad/s，与规则值 0.4r/s = 2.513 精确吻合 → w 是常量
//     - r  = 0.2774 ± 0.0035 m，与规则值 0.2765 吻合         → r 是常量
//     - 但 w 的符号翻转 9 次，|w| 一度冲到 4.02（超出 2.51 钳位）
//     - t=11.70 / 17.00 / 21.40 各只丢 1 帧，w 就被打回 0 重新收敛
//     - cmd_yaw 有 88 帧（6.7%）单帧跳 >5°，p95 83°、max 89.7°
//   也就是说：不稳定不是滤波调参不到位，而是模型里塞了本不该估的自由度。
//
// 本类的状态只有 4 维（旋转中心 + 相位）：
//     [ x, y, z, theta ]
//   - x,y,z  旋转中心位置。前哨站是固定建筑，不会平移，所以不建速度项
//            （整车模型的 vx/vy/vz 在这里纯粹是噪声入口）
//   - theta  0 号板的方位相位，唯一真正随时间演化的量
//
//   w（转速）不是状态，是常量 OUTPOST_W * spin_dir_。
//   r（半径）不是状态，是常量 OUTPOST_R。
//   三块板的高度阶梯也是常量表，见 OUTPOST_DZ。
//
// 旋转方向：不做滤波，用「a 的差分投票 + 滞回锁定」。方向只有 ±1 两种取值，
//   是离散量，交给 EKF 连续估计本身就是错的建模（这正是 w 符号翻转的来源）。
//   锁定后除非出现足够多的反向证据才允许翻转，避免噪声把方向抖翻。
// ============================================================================

// —— 规则常量（RMUC 2026 前哨站）——
// 注意 OUTPOST_W 只作**先验初值**，不当死常量用。
// records/2026-07-31_06-22-34 实测：换板间隔 p50=0.7467s（30 次正序切换，
// 排除了 2 次逆序抖动），推出真实转速 120°/0.7467s = 2.805 rad/s = 0.446 r/s，
// 比标称 0.4 r/s 高 11.7%。原代码把 w 每帧硬钳到 ±2.51，导致相位每 5s 漂 84°，
// 观测匹配误差累积到 20~31° 后 Tracker 判失败重建 —— 这就是"锁定三块板跳变"
// 的根因（实测 5 次重建，间隔 4.7~5.7s，与漂移周期吻合）。
// 所以转速必须在线估计：标称值当先验，实际值由观测修正。
constexpr double OUTPOST_W = 0.4 * 2 * M_PI;  // 0.4 r/s = 2.5133 rad/s（先验）
// 在线估计允许的范围：标称 ±25%，防止被换板跳变带跑
constexpr double OUTPOST_W_MIN = 0.75 * OUTPOST_W;
constexpr double OUTPOST_W_MAX = 1.35 * OUTPOST_W;
constexpr double OUTPOST_R = 0.2765;          // 旋转半径 m
constexpr int OUTPOST_ARMOR_NUM = 3;          // 三块装甲板，互隔 120°

// 三块板的高度阶梯（顺时针 1→2→3 递增 10.2cm）。
// phase 表示 0 号板当前对应 L/H/M 中哪一阶；EKF id 按 yaw 递增（CCW）。
constexpr double OUTPOST_STEP = 0.102;
constexpr double OUTPOST_DZ[3][3] = {
  {0.0, 2 * OUTPOST_STEP, OUTPOST_STEP},  // phase=0: plate-0 = L
  {2 * OUTPOST_STEP, OUTPOST_STEP, 0.0},  // phase=1: plate-0 = H
  {OUTPOST_STEP, 0.0, 2 * OUTPOST_STEP},  // phase=2: plate-0 = M
};

class OutpostTarget
{
public:
  ArmorName name = ArmorName::outpost;
  ArmorType armor_type;
  ArmorPriority priority;
  int last_id = 0;  // debug only

  OutpostTarget() = default;
  OutpostTarget(const Armor & armor, std::chrono::steady_clock::time_point t);
  void set_gate(double m) { gate_m_ = m; }

  void predict(std::chrono::steady_clock::time_point t);
  void update(const Armor & armor);

  // —— 对外接口，与 Target 保持一致，便于 aimer / main 共用 ——
  // 完整 11 维向量，布局与整车 Target 相同（x vx y vy z vz a w r l h），
  // 这样 aimer、plotter、signals 的既有下标全部沿用，不用改下游。
  // vx/vy/vz 恒 0（前哨站不平移），w = 常量×方向，r = 常量。
  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const { return ekf_; }
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  bool diverged() const;
  bool convergened() const { return converged_; }

  // 在线估计的转速幅值（rad/s，恒正）。未收敛时返回先验 OUTPOST_W。
  double w_mag() const { return w_mag_; }

  // 转速是否已收敛（够多次换板观测）。未收敛时相位外推还不可靠，
  // 实测框跳 14 次里有 12 次都在"方向未锁 + 转速未收敛"这两个阶段，
  // 稳态只剩 2 次 —— 所以这两个阶段就该老实不发瞄准点。
  bool w_converged() const { return switch_seen_ >= kWSwitchNeeded; }
  static constexpr int kWSwitchNeededPublic = 3;

  // 整体就绪：方向锁定 且 转速收敛，才认为可以拿去瞄准
  bool ready() const { return dir_locked_ && w_converged(); }

  // 旋转方向：+1 / -1，未锁定时为 0
  int spin_dir() const { return spin_dir_; }
  bool dir_locked() const { return dir_locked_; }
  int phase() const { return phase_; }

  // 给定弹丸飞行时间，返回命中时刻"最先正对我方"的那块板的 xyza。
  // 前哨站打的是远距离（5m 外，v=23 时飞行 217ms，期间目标转 31°），
  // 所以必须按命中时刻选板，而不是按当前帧选——按当前帧选的话，
  // 板在子弹到达前就已经转走 31°，等于必偏。
  // 返回 valid=false 表示该时刻没有板处于可打角度内。
  struct AimChoice
  {
    bool valid;
    Eigen::Vector4d xyza;
    double delta_angle;  // 命中时刻该板法线与视线的夹角 rad，供开火判据用
  };
  AimChoice choose_at(double fly_time, double max_delta) const;

  // 重捕时沿用上次锁定的方向（前哨站转向整局不变，不该从零重估）。
  // 只恢复方向，相位仍由观测重新确定——相位会随时间变，方向不会。
  void restore_dir(int dir);

  // 重捕时沿用上次学到的转速。转速是物理常量（整局不变），重建后从先验
  // 2.5133 重新学要 3 次换板 ≈ 2.3s，这段时间相位按错误转速外推会漂 —— 
  // 实测 12.9s 的录像里因此一次都没 ready。所以必须跨重建保留。
  void restore_w(double w_mag, int switch_seen);

  // Tracker 在"丢帧后首次重新拿到观测"时调用，触发渐进收敛
  void mark_reacquire(int frames) { reacquire_frames_ = frames; }

  // 重捕时沿用上次的旋转中心。前哨站是固定建筑，中心是常量——
  // 重建时由"单块观测板 + 半径"反推中心，5m 外 PnP 纵深噪声会让它偏几十 cm
  // （实测两段稳定值差 56cm，整段 std 26.5cm，而单段内只有 3.5cm）。
  // 沿用旧中心 + 让 EKF 慢慢修，比每次重新猜准得多。
  void restore_center(double cx, double cy, double cz);

private:
  // 状态：x y z theta（4 维）
  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  int update_count_ = 0;
  bool converged_ = false;

  // —— 旋转方向：离散量，投票 + 滞回，不进 EKF ——
  // 转速收敛所需的换板观测次数。实测（records/2026-07-31_06-55-25）第 3 次
  // 就到 0.436 r/s（真值 0.444，误差 1.8%），6 次门槛过高：那份录像 12.9s
  // 只攒到 4 次就判 lost，重建后计数归零 → ready() 永远为假 → 前哨站链路
  // 从不接管。降到 3，配合 restore_w 跨重建保留，实际首次收敛更快。
  static constexpr int kWSwitchNeeded = 3;

  // 转速幅值在线估计：用"同一块板转过 120° 的时间"更新，比对 yaw 差分更抗噪
  // （单帧 yaw 噪声 ±6°，而一次换板积累 120°，相对误差小 20 倍）
  double w_mag_ = OUTPOST_W;
  // 换板间隔样本池，取中位数而非 EMA。
  // EMA 的问题：识别丢帧会让换板事件被延迟发现 → 间隔算大 → 转速算小，
  // 而这种偏小样本很常见（实测识别率仅 45%，60 次换板只有 2 次完全无丢帧），
  // EMA 会被它们持续拉低。实测 EMA 从第 10 次的 0.428 r/s 退化到第 20 次的
  // 0.393（真值 0.4425），相位滞后随之在 9~37° 间漂 —— 这就是"打着打着滞后"。
  // 中位数天然抗离群：同一份数据滑动中位数稳定在 0.424~0.476。
  std::deque<double> sw_intervals_;
  static constexpr size_t kSwWindow = 9;
  int switch_seen_ = 0;
  std::chrono::steady_clock::time_point last_switch_t_;
  bool has_switch_t_ = false;
  int last_matched_id_ = -1;
  // 这两个都扫过参，对滞后无影响（q_theta 2e-4~5e-2 只让偏差 16.8→15.8cm，
  // r_yaw 1e-2~3e-4 只让 16.8→16.3cm）——因为 16cm 是观测噪声地板
  // （观测自身逐帧 p95 就有 15.4cm），不是 EKF 滞后。保持原值。
  double q_theta_ = 2e-4;
  double r_yaw_ = 1e-2;
  double gate_m_ = 0.45;  // 野值门阈值(m)；OP_GATE 环境变量可覆盖（扫参用）
  int outlier_run_ = 0;   // 连续被拒的帧数，太多说明预测自己错了，得放行重收敛

  int spin_dir_ = 0;        // +1 CCW / -1 CW / 0 未定
  bool dir_locked_ = false;
  double dir_score_ = 0.0;  // 正向证据累积，超阈值锁定；带滞回防抖翻
  double last_obs_yaw_ = 0.0;
  bool has_last_obs_ = false;
  std::chrono::steady_clock::time_point last_obs_t_;

  // —— 相位（哪块板是 L）：同样是离散量，用最小二乘选一次后锁定 ——
  int phase_ = 0;
  bool phase_locked_ = false;
  std::vector<std::vector<double>> z_obs_{3};  // 每块板的 z 观测累积

  // 每块板 z 的 EMA：锁定 phase 后用观测替代常量表，
  // 因为 DZ 表压缩了真实高度差，直接用观测更准
  double z_ema_[3] = {-1000.0, -1000.0, -1000.0};

  // 野值判定：观测与预测差太多就拒绝这一帧的 update。
  // 前哨站几何完全已知（中心固定、半径恒定、三板互隔 120°），所以"预测得到的
  // 板位置"很可靠，可以反过来用它筛观测——这是整车模型做不到的（那边半径、
  // 转速都在估，预测本身就不可信）。
  bool is_outlier(const Armor & armor, int id) const;

  // 丢失后重现的首帧，观测与外推位置会有落差（丢失期间只能靠常量转速外推）。
  // 一次性吸收这个落差 → 瞄准点单帧跳 15~21cm（实测 3 次残余跳全是这个原因，
  // delta 分散在 5.7~33.9°，不是换板边界）。
  // 对策：重现后若干帧内把观测噪声 R 放大，让 EKF 分几帧渐进收敛而不是一步跳。
  int reacquire_frames_ = 0;

  void vote_direction(const Armor & armor, std::chrono::steady_clock::time_point t);
  void try_lock_phase();
  int match_armor_id(const Armor & armor) const;
  double armor_z(int id) const;
};


// ============================================================================
// 前哨站独立跟踪器（自带状态机）
// ============================================================================
// 与 Tracker 完全平行的一条链路：自己筛前哨站装甲板、自己管生命周期。
// 整车的 Tracker / Target 一行不改，所以不会影响整车 tracking。
//
// 为什么要自己的状态机而不复用 Tracker 的：
//   Tracker 的 detect_count / temp_lost_count 是为"目标可能开走、可能换车"
//   设计的。前哨站是固定建筑，丢失只可能是遮挡或识别抖动，而且它的 phase /
//   方向一旦锁定就长期有效——重捕时应当**沿用**而不是重新收敛。
//   这两种生命周期语义不同，混在一个状态机里只会互相将就。
class OutpostTracker
{
public:
  explicit OutpostTracker(const std::string & config_path);

  // 从当帧所有装甲板里挑出前哨站并跟踪。
  // 返回空 = 当前没有可用的前哨站目标（未捕获 / 已丢失 / 方向未锁）。
  // armors 只读不改，避免与整车链路争用同一份数据。
  std::vector<OutpostTarget> track(
    const std::list<Armor> & armors, std::chrono::steady_clock::time_point t, Solver & solver);

  std::string state() const { return state_; }

private:
  OutpostTarget target_;
  std::string state_ = "lost";
  int detect_count_ = 0;
  int lost_count_ = 0;

  int min_detect_count_;
  int max_lost_count_;
  double lost_timeout_;
  std::chrono::steady_clock::time_point last_t_;
  bool has_last_t_ = false;

  // 方向/相位跨重捕缓存：前哨站是固定建筑，这些量重捕后应沿用而非重估
  bool cache_valid_ = false;
  int cache_dir_ = 0;
  double cache_w_ = 0.0;
  int cache_switch_seen_ = 0;
  bool cache_has_center_ = false;
  double cache_cx_ = 0, cache_cy_ = 0, cache_cz_ = 0;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__OUTPOST_TARGET_HPP
