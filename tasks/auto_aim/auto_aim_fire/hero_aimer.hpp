#ifndef AUTO_AIM__HERO_AIMER_HPP
#define AUTO_AIM__HERO_AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <string>

#include "io/command.hpp"
#include "tasks/auto_aim/auto_aim_track/target.hpp"

namespace auto_aim
{

struct HeroAimPoint
{
  bool valid;
  Eigen::Vector4d xyza;
};

class HeroAimer
{
public:
  explicit HeroAimer(const std::string & config_path);
  ~HeroAimer() = default;

  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now = true, Eigen::Vector3d ypr = Eigen::Vector3d::Zero());

  Eigen::Vector3d get_outpost_hit_xyz(const Target & target) const;
  Eigen::Vector3d get_top_front(const Target & target_origin) const;

  HeroAimPoint debug_aim_point_;

private:
  double shoot_delay_;
  double yaw_offset_;
  double pitch_offset_;
  double max_fire_error_yaw_;
  double max_fire_error_pitch_;
  double max_shoot_angle_;
  int lock_id_ = -1;

  HeroAimPoint choose_aim_point(const Target & target);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__HERO_AIMER_HPP
