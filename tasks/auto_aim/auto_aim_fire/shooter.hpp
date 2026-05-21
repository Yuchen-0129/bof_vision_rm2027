#ifndef AUTO_AIM__SHOOTER_HPP
#define AUTO_AIM__SHOOTER_HPP

#include <chrono>
#include <string>

#include "io/command.hpp"
#include "tasks/auto_aim/auto_aim_fire/aimer.hpp"

namespace auto_aim
{
class Shooter
{
public:
  Shooter(const std::string & config_path);

  bool shoot(
    const io::Command & command, const auto_aim::Aimer & aimer,
    const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos);

private:
  io::Command last_command_;
  double judge_distance_;
  double first_tolerance_;
  double second_tolerance_;
  bool auto_fire_;
  // 开火节奏限制
  double min_fire_interval_;          // 最短两次开火间隔 (s)
  double max_plate_yaw_for_fire_;     // 板法线相对视线最大偏角 (rad)
  std::chrono::steady_clock::time_point last_fire_time_;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__SHOOTER_HPP