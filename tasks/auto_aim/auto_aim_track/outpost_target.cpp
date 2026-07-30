#include "outpost_target.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
namespace
{
// 方向锁定阈值：证据分累积到 ±LOCK 才锁；锁定后要反向累积到 ∓UNLOCK 才允许翻。
// 两者不同 = 滞回，防止噪声在边界反复穿越把方向抖翻（实测 w 符号翻了 9 次）。
constexpr double DIR_LOCK_SCORE = 6.0;
constexpr double DIR_UNLOCK_SCORE = 12.0;
// 单帧观测的角速度落在 [0.3, 3.0]×OUTPOST_W 之外就不计票——
// 太小是静止噪声，太大多半是换板导致的角度跳变（120° 突跳）
constexpr double RATE_MIN_RATIO = 0.3;
constexpr double RATE_MAX_RATIO = 3.0;
// phase 锁定需要每块板至少这么多 z 观测
constexpr size_t PHASE_MIN_OBS = 8;
constexpr double Z_EMA_ALPHA = 0.15;
}  // namespace

OutpostTarget::OutpostTarget(const Armor & armor, std::chrono::steady_clock::time_point t)
: armor_type(armor.type), priority(armor.priority), t_(t)
{
  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  // 由观测板反推旋转中心：板在半径 r 上，法线朝外
  auto cx = xyz[0] + OUTPOST_R * std::cos(ypr[0]);
  auto cy = xyz[1] + OUTPOST_R * std::sin(ypr[0]);
  auto cz = xyz[2];

  // 状态 [x y z theta]，theta 取当前观测板的方位
  Eigen::VectorXd x0{{cx, cy, cz, ypr[0]}};

  // P0：中心位置给中等不确定度，theta 给较大（第一帧只有一块板，相位未知）
  Eigen::VectorXd P0_dig{{0.25, 0.25, 0.25, 0.4}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);

  last_obs_yaw_ = ypr[0];
  last_obs_t_ = t;
  has_last_obs_ = true;
}

void OutpostTarget::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  t_ = t;
  if (dt <= 0) return;

  // 状态转移：中心不动，相位按已知转速匀速推进
  //   x' = x,  y' = y,  z' = z
  //   theta' = theta + w*dt   其中 w = OUTPOST_W * spin_dir_（常量，不是状态）
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(4, 4);

  // 过程噪声：中心几乎不动（前哨站是固定建筑），给很小的值只为防 P 退化；
  // theta 的噪声反映"转速常量假设"的误差，给得比整车模型小得多——
  // 整车 v2=0.1 是留给未知角加速度的，这里转速已知，不需要那份余量。
  double q_pos = 1e-8 * dt;  // 固定建筑，中心基本不动
  double q_theta = 2e-4 * dt;
  Eigen::VectorXd q_dig{{q_pos, q_pos, q_pos, q_theta}};
  Eigen::MatrixXd Q = q_dig.asDiagonal();

  // 方向未锁定时不推进相位：宁可让 theta 停着等观测，也不要往错误方向外推
  // （方向错了的话，外推 0.23s 就偏 33°，比不推进更糟）
  double w = dir_locked_ ? OUTPOST_W * spin_dir_ : 0.0;

  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd xp = x;
    xp[3] = tools::limit_rad(x[3] + w * dt);
    return xp;
  };

  ekf_.predict(F, Q, f);
}

void OutpostTarget::vote_direction(const Armor & armor, std::chrono::steady_clock::time_point t)
{
  double obs_yaw = armor.ypr_in_world[0];

  if (!has_last_obs_) {
    last_obs_yaw_ = obs_yaw;
    last_obs_t_ = t;
    has_last_obs_ = true;
    return;
  }

  auto dt = tools::delta_time(t, last_obs_t_);
  if (dt <= 1e-4) return;

  // 观测板法线的角速度。注意这里可能因为换板而跳 ±120°，靠下面的幅值门限滤掉
  double dyaw = tools::limit_rad(obs_yaw - last_obs_yaw_);
  double rate = dyaw / dt;

  last_obs_yaw_ = obs_yaw;
  last_obs_t_ = t;

  double ratio = std::abs(rate) / OUTPOST_W;
  if (ratio < RATE_MIN_RATIO || ratio > RATE_MAX_RATIO) return;  // 不可信，不计票

  // 计票：权重取 min(dt/0.02, 1)，让长间隔帧不至于一票定音
  double weight = std::min(dt / 0.02, 1.0);
  dir_score_ += (rate > 0 ? weight : -weight);
  dir_score_ = std::clamp(dir_score_, -3.0 * DIR_UNLOCK_SCORE, 3.0 * DIR_UNLOCK_SCORE);

  if (!dir_locked_) {
    if (std::abs(dir_score_) >= DIR_LOCK_SCORE) {
      spin_dir_ = dir_score_ > 0 ? 1 : -1;
      dir_locked_ = true;
      tools::logger()->info(
        "[Outpost] 旋转方向锁定: {} (score={:+.1f})", spin_dir_ > 0 ? "CCW(+)" : "CW(-)",
        dir_score_);
    }
  } else {
    // 已锁定：只有反向证据累积到 UNLOCK 才翻，且翻转后重置分数
    if (spin_dir_ > 0 && dir_score_ <= -DIR_UNLOCK_SCORE) {
      spin_dir_ = -1;
      dir_score_ = 0;
      tools::logger()->warn("[Outpost] 旋转方向翻转 → CW(-)");
    } else if (spin_dir_ < 0 && dir_score_ >= DIR_UNLOCK_SCORE) {
      spin_dir_ = 1;
      dir_score_ = 0;
      tools::logger()->warn("[Outpost] 旋转方向翻转 → CCW(+)");
    }
  }
}

int OutpostTarget::match_armor_id(const Armor & armor) const
{
  // 用观测板方位与三块板预测方位的角差最小者匹配
  const auto & x = ekf_.x;
  double obs_yaw = armor.ypr_in_world[0];
  int best = 0;
  double best_err = 1e10;
  for (int i = 0; i < OUTPOST_ARMOR_NUM; i++) {
    double pred = tools::limit_rad(x[3] + i * 2 * M_PI / OUTPOST_ARMOR_NUM);
    double err = std::abs(tools::limit_rad(obs_yaw - pred));
    if (err < best_err) {
      best_err = err;
      best = i;
    }
  }
  return best;
}

void OutpostTarget::try_lock_phase()
{
  if (phase_locked_) return;
  for (int i = 0; i < 3; i++)
    if (z_obs_[i].size() < PHASE_MIN_OBS) return;

  // 三块板各取 z 观测中位数，与三种 phase 假设的阶梯形状做最小二乘，选残差最小的
  double zm[3];
  for (int i = 0; i < 3; i++) {
    auto v = z_obs_[i];
    std::sort(v.begin(), v.end());
    zm[i] = v[v.size() / 2];
  }

  int best_phase = 0;
  double best_cost = 1e10;
  for (int p = 0; p < 3; p++) {
    // 去掉共同偏置后比形状（我们只关心相对阶梯，不关心整体高度）
    double mean_obs = (zm[0] + zm[1] + zm[2]) / 3.0;
    double mean_mod = (OUTPOST_DZ[p][0] + OUTPOST_DZ[p][1] + OUTPOST_DZ[p][2]) / 3.0;
    double cost = 0;
    for (int i = 0; i < 3; i++) {
      double d = (zm[i] - mean_obs) - (OUTPOST_DZ[p][i] - mean_mod);
      cost += d * d;
    }
    if (cost < best_cost) {
      best_cost = cost;
      best_phase = p;
    }
  }

  phase_ = best_phase;
  phase_locked_ = true;
  for (int i = 0; i < 3; i++) z_ema_[i] = zm[i];
  tools::logger()->info(
    "[Outpost] phase 锁定 = {} (cost={:.5f}, z=[{:.3f} {:.3f} {:.3f}])", phase_, best_cost, zm[0],
    zm[1], zm[2]);
}

void OutpostTarget::update(const Armor & armor)
{
  update_count_++;

  vote_direction(armor, t_);

  int id = match_armor_id(armor);
  last_id = id;

  // 累积该板的 z 观测，用于锁 phase
  // z 单独维护：每块板只累积自己的观测，不跨板推断（见 armor_z 注释）
  if (z_ema_[id] < -100.0)
    z_ema_[id] = armor.xyz_in_world[2];
  else
    z_ema_[id] = (1 - Z_EMA_ALPHA) * z_ema_[id] + Z_EMA_ALPHA * armor.xyz_in_world[2];

  // —— 观测模型：由状态推该板的 (x, y, yaw) ——
  // 板中心 = 旋转中心 - r*(cos, sin)(theta_id)，板法线方位 = theta_id
  // z 不进 EKF：三块板高度各不同且由常量表/EMA 给出，塞进状态只会污染中心 z
  double theta_id = tools::limit_rad(ekf_.x[3] + id * 2 * M_PI / OUTPOST_ARMOR_NUM);

  auto h = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    double th = tools::limit_rad(x[3] + id * 2 * M_PI / OUTPOST_ARMOR_NUM);
    Eigen::VectorXd z(3);
    z[0] = x[0] - OUTPOST_R * std::cos(th);
    z[1] = x[1] - OUTPOST_R * std::sin(th);
    z[2] = th;
    return z;
  };

  // 雅可比 ∂h/∂[x y z theta]
  Eigen::MatrixXd H(3, 4);
  // clang-format off
  H << 1, 0, 0,  OUTPOST_R * std::sin(theta_id),
       0, 1, 0, -OUTPOST_R * std::cos(theta_id),
       0, 0, 0,  1;
  // clang-format on

  // 观测噪声。这里的相对大小很关键，调错会让 theta 和 center 互相补偿：
  //
  // theta 名义上靠 yaw 观测确定，但位置观测也间接约束它
  //   ∂(板xy)/∂theta = r ≈ 0.28
  // 所以位置观测对 theta 的等效精度 = σ_pos / 0.28。
  // 若位置给 σ=2cm、yaw 给 σ=14°，则位置的等效角精度 4° 反而比 yaw 更"可信"，
  // EKF 就会主要用位置去定 theta —— 而位置观测含 PnP 横向误差，
  // 结果 theta 与 center 互相补偿，实测 center 被拖偏 0.19m，
  // 板位置偏 0.25m，重投影误差 28px（旧模型 7px）。
  //
  // 正确做法：两者都填**真实**精度，让 EKF 自己权衡。
  // 5m 外 PnP 横向误差实测 5~10cm → σ=7cm；optimize_yaw 后角度误差约 6° → σ=0.1rad
  Eigen::VectorXd R_dig{{5e-3, 5e-3, 1e-2}};
  Eigen::MatrixXd R = R_dig.asDiagonal();

  Eigen::VectorXd z_obs(3);
  z_obs[0] = armor.xyz_in_world[0];
  z_obs[1] = armor.xyz_in_world[1];
  z_obs[2] = armor.ypr_in_world[0];

  // 角度残差要走 limit_rad，否则 ±π 附近会得到 2π 的假残差
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[2] = tools::limit_rad(c[2]);
    return c;
  };

  ekf_.update(z_obs, H, R, h, z_subtract);

  // z（中心高度）不进 EKF，直接用观测 EMA 回填：板 z = 中心 z + 阶梯
  // 中心 z 只作为"没见过该板"时的兜底，用全部观测的慢速平均
  ekf_.x[2] = 0.98 * ekf_.x[2] + 0.02 * armor.xyz_in_world[2];

  // 收敛判据：方向已锁 + 足够更新次数 + 未发散
  if (dir_locked_ && update_count_ > 10 && !diverged()) converged_ = true;
}

double OutpostTarget::armor_z(int id) const
{
  // 只用该板自己的 z 观测 EMA，没有观测过就退回中心 z。
  //
  // 这里踩过一个坑：本来想「phase 锁定后按 OUTPOST_DZ 常量表推各板高度」，
  // 但实测三板 z 的阶梯是 [0.087, 0, 0.223] m，除以规则阶梯 0.102 得
  // [0.85, 0, 2.19] —— 不是整数倍。说明 5m 外 PnP 的高度精度本身就撑不起
  // 「板号 → 固定高度」这个映射（横向 xy 准，z 差得多）。
  // 用常量表或跨板 EMA 反而会把某块板的 z 误差搬到另一块板上，实测重投影
  // 误差从 7.4px 恶化到 29px。所以 z 这一维不做任何跨板推断。
  if (z_ema_[id] > -100.0) return z_ema_[id];
  return ekf_.x[2];
}

std::vector<Eigen::Vector4d> OutpostTarget::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> list;
  const auto & x = ekf_.x;
  for (int i = 0; i < OUTPOST_ARMOR_NUM; i++) {
    double th = tools::limit_rad(x[3] + i * 2 * M_PI / OUTPOST_ARMOR_NUM);
    Eigen::Vector4d xyza;
    xyza[0] = x[0] - OUTPOST_R * std::cos(th);
    xyza[1] = x[1] - OUTPOST_R * std::sin(th);
    xyza[2] = armor_z(i);
    xyza[3] = th;
    list.push_back(xyza);
  }
  return list;
}

OutpostTarget::AimChoice OutpostTarget::choose_at(double fly_time, double max_delta) const
{
  AimChoice out{false, Eigen::Vector4d::Zero(), 0.0};
  if (!dir_locked_) return out;  // 方向未知就不发瞄准点，避免往错方向外推

  const auto & x = ekf_.x;
  // 命中时刻的相位：只用已知常量外推，不含任何被估计的量
  double theta_hit = tools::limit_rad(x[3] + OUTPOST_W * spin_dir_ * fly_time);

  // 三块板里挑"法线最正对我方"的那块。delta_angle = 板法线方位 - 板中心方位角，
  // 为 0 表示板正对我方（此时投影最宽、最好打）
  double best_delta = 1e10;
  int best_id = -1;
  Eigen::Vector4d best_xyza;
  for (int i = 0; i < OUTPOST_ARMOR_NUM; i++) {
    double th = tools::limit_rad(theta_hit + i * 2 * M_PI / OUTPOST_ARMOR_NUM);
    Eigen::Vector4d xyza;
    xyza[0] = x[0] - OUTPOST_R * std::cos(th);
    xyza[1] = x[1] - OUTPOST_R * std::sin(th);
    xyza[2] = armor_z(i);
    xyza[3] = th;

    double center_yaw = std::atan2(xyza[1], xyza[0]);
    double delta = std::abs(tools::limit_rad(th - center_yaw));
    if (delta < best_delta) {
      best_delta = delta;
      best_id = i;
      best_xyza = xyza;
    }
  }

  if (best_id < 0 || best_delta > max_delta) return out;
  out.valid = true;
  out.xyza = best_xyza;
  out.delta_angle = best_delta;
  return out;
}

Eigen::VectorXd OutpostTarget::ekf_x() const
{
  // 拼成与整车 Target 相同的 11 维布局，让 aimer / plotter / signals 的既有下标复用
  //   x vx y vy z vz a w r l h
  Eigen::VectorXd out = Eigen::VectorXd::Zero(11);
  out[0] = ekf_.x[0];
  out[2] = ekf_.x[1];
  out[4] = ekf_.x[2];
  out[6] = ekf_.x[3];
  out[7] = OUTPOST_W * spin_dir_;  // 常量×方向，不是估计量
  out[8] = OUTPOST_R;              // 常量
  // vx vy vz = 0（前哨站不平移）；l h = 0（高度差走 armor_z 而非状态）
  return out;
}

void OutpostTarget::restore_dir(int dir)
{
  if (dir == 0) return;
  spin_dir_ = dir;
  dir_locked_ = true;
  dir_score_ = dir > 0 ? DIR_LOCK_SCORE : -DIR_LOCK_SCORE;
  tools::logger()->info("[Outpost] 重捕沿用方向 {}", dir > 0 ? "CCW(+)" : "CW(-)");
}

bool OutpostTarget::diverged() const
{
  const auto & x = ekf_.x;
  // 中心位置必须有限且在合理距离内；theta 必须有限
  if (!std::isfinite(x[0]) || !std::isfinite(x[1]) || !std::isfinite(x[2]) ||
      !std::isfinite(x[3]))
    return true;
  auto d = std::hypot(x[0], x[1]);
  if (d < 0.3 || d > 20.0) {
    tools::logger()->debug("[Outpost] diverged: d={:.2f}", d);
    return true;
  }
  return false;
}


// ============================================================================
// OutpostTracker
// ============================================================================
OutpostTracker::OutpostTracker(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  // 全部用带默认值的读法：这条链路是新加的，不能让没同步改 yaml 的机器人开不了机
  min_detect_count_ =
    yaml["outpost_min_detect_count"] ? yaml["outpost_min_detect_count"].as<int>() : 3;
  max_lost_count_ =
    yaml["outpost_max_temp_lost_count"] ? yaml["outpost_max_temp_lost_count"].as<int>() : 150;
  lost_timeout_ = yaml["lost_timeout"] ? yaml["lost_timeout"].as<double>() : 0.5;
}

std::vector<OutpostTarget> OutpostTracker::track(
  const std::list<Armor> & armors, std::chrono::steady_clock::time_point t, Solver & solver)
{
  // 只取前哨站，且只取一块（前哨站是唯一目标，多块时取最靠近图像中心的）
  const Armor * pick = nullptr;
  double best = 1e18;
  for (const auto & a : armors) {
    if (a.name != ArmorName::outpost) continue;
    auto d = cv::norm(a.center - cv::Point2f(720, 540));
    if (d < best) {
      best = d;
      pick = &a;
    }
  }

  // 长时间没帧就整体复位（比如切换目标或程序空转），避免拿旧状态硬推
  if (has_last_t_ && tools::delta_time(t, last_t_) > lost_timeout_) {
    state_ = "lost";
    detect_count_ = 0;
    lost_count_ = 0;
  }
  last_t_ = t;
  has_last_t_ = true;

  if (pick == nullptr) {
    // 丢帧：保持状态，靠常量转速继续外推，直到超过容忍上限
    if (state_ != "lost") {
      lost_count_++;
      if (lost_count_ > max_lost_count_) {
        state_ = "lost";
        detect_count_ = 0;
        tools::logger()->debug("[OutpostTracker] lost");
      } else {
        target_.predict(t);
      }
    }
    if (state_ == "tracking") return {target_};
    return {};
  }

  Armor armor = *pick;  // 拷一份再 solve，不改调用方的 armors
  solver.solve(armor);

  if (state_ == "lost") {
    target_ = OutpostTarget(armor, t);
    // 沿用上次锁定的方向：前哨站转向整局不变，重捕不该从零重估
    if (cache_valid_ && cache_dir_ != 0) target_.restore_dir(cache_dir_);
    state_ = "detecting";
    detect_count_ = 1;
    lost_count_ = 0;
    return {};
  }

  target_.predict(t);
  target_.update(armor);
  lost_count_ = 0;

  if (target_.diverged()) {
    tools::logger()->debug("[OutpostTracker] diverged, reset");
    state_ = "lost";
    detect_count_ = 0;
    return {};
  }

  if (state_ == "detecting") {
    detect_count_++;
    if (detect_count_ >= min_detect_count_) state_ = "tracking";
    return {};
  }

  // tracking：缓存方向供重捕沿用
  if (target_.dir_locked()) {
    cache_valid_ = true;
    cache_dir_ = target_.spin_dir();
  }
  return {target_};
}

}  // namespace auto_aim
