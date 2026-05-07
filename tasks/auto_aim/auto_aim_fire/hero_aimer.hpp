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

// 英雄自瞄：与 Planner::heroaim 一致的几何 + 阻力弹道；中转速用车体中心 z
class HeroAimer
{
public:
  struct AimPoint
  {
    bool valid;
    Eigen::Vector4d xyza;
  };

  AimPoint debug_aim_point;

  explicit HeroAimer(const std::string & config_path);

  io::Command shoot(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now, const Eigen::Vector3d & gimbal_ypr);

private:
  /// yaml `shoot_delay`：发弹/控制延时，全路径仅此一项可调延时
  double ctrl_to_fire_;
  double yaw_offset_;
  double pitch_offset_;
  double max_fire_error_yaw_;
  double max_fire_error_pitch_;
  double w_rotate_;
  double w_center_height_;
  double tower_pitch_prediction_time_;

  int last_hit_id_ = -1;

  Eigen::Vector3d center_hit_xy(const Eigen::VectorXd & ekf_x) const;
  /// 与 Planner::heroaim 对齐：最近装甲 / last_id 视野条件、旋转中心圆上瞄准点；|ω|≥w_center 用车体中心；
  /// 1≤|ω|<w_center 保留 xy 圆瞄准、z 用车体中心 ekf_x[4]；|ω|<1 用装甲 xyz；前哨 predict 后用中心高度。
  Eigen::Vector4d hero_planner_aim_xyza(const Target & target) const;

  AimPoint choose_aim_point(const Target & target);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__HERO_AIMER_HPP
