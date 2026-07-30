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
  shoot_delay_ = yaml["outpost_shoot_delay"] ? yaml["outpost_shoot_delay"].as<double>() : 0.10;
  // 可打窗口：±60° 时三块板刚好 100% 占空（3×120°），是"尽可能多打"的理论上限。
  // 但板越斜有效投影越窄（60° 时小板 135mm 只剩 67mm），所以瞄准放宽到 60°、
  // 开火收紧到 35°：宁可多瞄准少开火，也不要在斜板上浪费弹丸。
  max_delta_ = (yaml["outpost_max_delta"] ? yaml["outpost_max_delta"].as<double>() : 60.0) / 57.3;
  fire_delta_ = (yaml["outpost_fire_delta"] ? yaml["outpost_fire_delta"].as<double>() : 35.0) / 57.3;
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
