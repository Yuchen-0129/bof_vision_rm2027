#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path)
: last_command_{false, false, 0, 0}, last_fire_time_(std::chrono::steady_clock::now())
{
  auto yaml = YAML::LoadFile(config_path);
  first_tolerance_ = yaml["first_tolerance"].as<double>() / 57.3;    // degree to rad
  second_tolerance_ = yaml["second_tolerance"].as<double>() / 57.3;  // degree to rad
  judge_distance_ = yaml["judge_distance"].as<double>();
  auto_fire_ = yaml["auto_fire"].as<bool>();
  // 开火节奏：默认两次开火至少间隔 0.1s（10 发/秒），yaml 可覆盖
  min_fire_interval_ = yaml["min_fire_interval"].as<double>(0.1);
  // 板法线最大允许偏角：默认 25°，超过这个角度不开火（避免侧倾跳弹）
  max_plate_yaw_for_fire_ = yaml["max_plate_yaw_for_fire"].as<double>(25.0) / 57.3;
}

bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos)
{
  if (!command.control || targets.empty() || !auto_fire_) return false;

  // 节奏限制：避免一帧打 N 发的疯狂开火
  const auto now = std::chrono::steady_clock::now();
  const double dt_since_last_fire =
    std::chrono::duration<double>(now - last_fire_time_).count();
  if (dt_since_last_fire < min_fire_interval_) {
    last_command_ = command;
    return false;
  }

  auto target_x = targets.front().ekf_x()[0];
  auto target_y = targets.front().ekf_x()[2];
  auto tolerance = std::sqrt(tools::square(target_x) + tools::square(target_y)) > judge_distance_
                     ? second_tolerance_
                     : first_tolerance_;

  // 云台对齐 + cmd 稳定 + aim_point 有效（原条件）
  const bool gimbal_ok =
    std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&
    std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance && aimer.debug_aim_point.valid;

  // 板法线检查：debug_aim_point.xyza = (x, y, z, armor_yaw_world)
  // 命中时刻板法线相对视线偏角 < max_plate_yaw_for_fire_ 才开火
  const auto & ap = aimer.debug_aim_point.xyza;
  const double aim_los = std::atan2(ap[1], ap[0]);
  const double plate_yaw_err = std::abs(tools::limit_rad(ap[3] - aim_los));
  const bool plate_facing = plate_yaw_err < max_plate_yaw_for_fire_;

  if (gimbal_ok && plate_facing) {
    last_command_ = command;
    last_fire_time_ = now;
    return true;
  }

  last_command_ = command;
  return false;
}

}  // namespace auto_aim
