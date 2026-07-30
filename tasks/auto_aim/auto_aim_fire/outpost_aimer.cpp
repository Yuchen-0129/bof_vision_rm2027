#include "outpost_aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
OutpostAimer::OutpostAimer(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  // 复用整车已有的 offset（同一台车、同一套标定），其余参数全部带默认值，
  // 这样没同步改 yaml 的机器人也能开机
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;
  // 提前量 = 弹道飞行 + 本参数。本参数要覆盖"从相机曝光到子弹离膛"的全部延迟：
  // 各段延迟都用 records/2026-07-31_06-57-17 实测（不是猜的）：
  //   云台响应（cmd→云台到位）  79ms  互相关 gimbal_yaw vs cmd_yaw 实测
  //   EKF 自身相位滞后          30ms  互相关 EKF a vs 观测板 yaw 实测
  //   图像链路（曝光+传输+推理） 12ms  signals 的 loop_ms p50
  //   合计约 121ms，取 0.12。
  // 原来给 0.10 略少；更关键的是 79ms 那段之前完全没纳入考虑。
  // 一块板的角宽仅 28.3°（板宽13.5cm/半径0.2765m），而 2.79rad/s 下
  // 100ms 就转 16°，超过半个板宽 —— 提前量估错必然打后沿，就是"打着打着滞后"。
  shoot_delay_ = yaml["outpost_shoot_delay"] ? yaml["outpost_shoot_delay"].as<double>() : 0.12;
  // 可打窗口。±60° 理论上是"尽可能多打"的上限（3 块板 ×120° 刚好 100% 占空），
  // 但实测不能这么设：records/2026-07-31_06-22-34 扫参结果——
  //   60°: 有效 245 帧, 瞄准点跳>15cm 占 5.6%, max 49.8cm
  //   45°: 有效 191 帧, 跳>15cm 1.2%,          max 20.0cm
  //   35°: 有效 143 帧, 跳>15cm 0.0%,          max 12.8cm   ← 采用
  //   30°: 有效 123 帧, 跳>15cm 0.0%,          max 11.3cm
  // 60° 时的大跳(41~50cm)全都发生在 delta 46~60° 的边界上：那是换板瞬间，
  // 瞄准点从一块板切到相邻板，位移就是两板间距——物理上必然，不是滤波抖动。
  // 在边界附近来回切会让瞄准点反复大幅横跳，比"少打几帧"糟得多。
  // 35° 是甜点：跳变归零，仍保留 58% 的帧可瞄。
  max_delta_ = (yaml["outpost_max_delta"] ? yaml["outpost_max_delta"].as<double>() : 35.0) / 57.3;
  // 开火再收一档：板越斜有效投影越窄（35° 时小板 135mm 只剩 111mm）
  fire_delta_ = (yaml["outpost_fire_delta"] ? yaml["outpost_fire_delta"].as<double>() : 25.0) / 57.3;
  auto_fire_ = yaml["auto_fire"] ? yaml["auto_fire"].as<bool>() : true;
}

io::Command OutpostAimer::aim(
  const std::vector<OutpostTarget> & targets, double bullet_speed, bool to_now)
{
  debug_valid = false;
  if (targets.empty()) return {false, false, 0, 0};

  const auto & target = targets.front();

  // 方向没锁就不接管云台。5m 处外推 0.22s，方向错了直接偏 32°，
  // 不如让操作手自己打
  if (!target.dir_locked()) return {false, false, 0, 0};

  if (bullet_speed < 14) bullet_speed = 23;

  // —— 先用当前位置估飞行时间，再按命中时刻选板 ——
  // 前哨站中心固定，飞行时间几乎不随相位变（板绕中心才 0.2765m，
  // 相对 5m 距离影响 <6%），所以一次估算就够，不需要像整车那样迭代
  auto x = target.ekf_x();
  double d_center = std::hypot(x[0], x[2]);
  tools::Trajectory traj0(bullet_speed, d_center, x[4]);
  if (traj0.unsolvable) {
    tools::logger()->debug("[OutpostAimer] 弹道不可解 d={:.2f} z={:.2f}", d_center, x[4]);
    return {false, false, 0, 0};
  }

  // 总提前时间 = 弹道飞行 + 开火延迟
  double lead = traj0.fly_time + shoot_delay_;
  auto choice = target.choose_at(lead, max_delta_);
  if (!choice.valid) {
    // 这一时刻没有板朝向我方——正常现象（3 块板轮转），不发 cmd 但也不算错误
    return {false, false, 0, 0};
  }

  debug_xyza = choice.xyza;
  debug_valid = true;

  // 用选中板的实际位置重算弹道（板与中心差 0.2765m，pitch 上不能忽略）
  Eigen::Vector3d xyz = choice.xyza.head(3);
  double d = std::hypot(xyz[0], xyz[1]);
  tools::Trajectory traj(bullet_speed, d, xyz[2]);
  if (traj.unsolvable) return {false, false, 0, 0};

  double yaw = std::atan2(xyz[1], xyz[0]) + yaw_offset_;
  double pitch = -(traj.pitch + pitch_offset_);  // 世界系 pitch 向上为负

  // 开火判据：板要足够正对（投影够宽才打得中）
  bool fire = auto_fire_ && (choice.delta_angle < fire_delta_);

  return {true, fire, yaw, pitch};
}

}  // namespace auto_aim
