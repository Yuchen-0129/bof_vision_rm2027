#include "hero_aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <vector>

#include "tasks/auto_aim/common/armor.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/resistance_trajectory.hpp"

namespace auto_aim
{

HeroAimer::HeroAimer(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  ctrl_to_fire_ = yaml["shoot_delay"].as<double>();  // 唯一延时参数（秒）
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;        // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;  // degree to rad
  max_fire_error_yaw_ = yaml["max_fire_error_yaw"].as<double>() / 57.3;    // degree to rad
  max_fire_error_pitch_ = yaml["max_fire_error_pitch"].as<double>() / 57.3;  // degree to rad
  w_rotate_ = yaml["w_rotate"].as<double>(1.0);
  w_center_height_ = yaml["w_center_height"].as<double>(3.0);
  tower_pitch_prediction_time_ = yaml["tower_pitch_prediction_time"].as<double>(0.05);
}

Eigen::Vector3d HeroAimer::center_hit_xy(const Eigen::VectorXd & ekf_x) const
{
  Eigen::Vector2d cxy(ekf_x[0], ekf_x[2]);
  const double norm = cxy.norm();
  if (norm < 1e-6) return {0, 0, ekf_x[4]};
  const double scale = (norm - ekf_x[8]) / norm;
  return {cxy.x() * scale, cxy.y() * scale, ekf_x[4]};
}

Eigen::Vector4d HeroAimer::hero_planner_aim_xyza(const Target & target) const
{
  static constexpr double k90 = 90.0 / 57.3;
  static constexpr double k60 = 60.0 / 57.3;

  const std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  const int armor_num = static_cast<int>(armor_xyza_list.size());
  if (armor_num <= 0) return Eigen::Vector4d::Zero();

  Eigen::VectorXd ekf_x = target.ekf_x();

  if (target.name == ArmorName::outpost) {
    Target tp = target;
    tp.predict(tower_pitch_prediction_time_);
    const Eigen::VectorXd ekf_p = tp.ekf_x();
    const Eigen::Vector3d c = center_hit_xy(ekf_p);
    const double cy = std::atan2(ekf_p[2], ekf_p[0]);
    return Eigen::Vector4d(c[0], c[1], c[2], cy);
  }

  const double center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  std::vector<double> delta_angle_list(static_cast<size_t>(armor_num));
  for (int i = 0; i < armor_num; ++i) {
    delta_angle_list[static_cast<size_t>(i)] =
      tools::limit_rad(armor_xyza_list[static_cast<size_t>(i)][3] - center_yaw);
  }

  double min_dist = 1e10;
  Eigen::Vector3d xyz = armor_xyza_list[0].head<3>();
  double yaw = armor_xyza_list[0][3];
  for (int i = 0; i < armor_num; ++i) {
    const double dist = armor_xyza_list[static_cast<size_t>(i)].head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = armor_xyza_list[static_cast<size_t>(i)].head<3>();
      yaw = armor_xyza_list[static_cast<size_t>(i)][3];
    }
  }

  const double abs_vyaw = std::abs(ekf_x[7]);
  const int lid = target.last_id;
  if (abs_vyaw < k90 && lid >= 0 && lid < armor_num &&
      std::abs(delta_angle_list[static_cast<size_t>(lid)]) < k60) {
    xyz = armor_xyza_list[static_cast<size_t>(lid)].head<3>();
    yaw = armor_xyza_list[static_cast<size_t>(lid)][3];
  }

  const double r = ekf_x[8];
  const double center_x = ekf_x[0];
  const double center_y = ekf_x[2];
  const double direction_yaw = std::atan2(center_y, center_x);
  double aim_x = center_x - r * std::cos(direction_yaw);
  double aim_y = center_y - r * std::sin(direction_yaw);
  double aim_z = xyz.z();
  const double v_yaw = ekf_x[7];

  if (std::abs(v_yaw) >= w_center_height_) {
    const Eigen::Vector3d c = center_hit_xy(ekf_x);
    aim_x = c[0];
    aim_y = c[1];
    aim_z = c[2];
  } else if (std::abs(v_yaw) >= 1.0) {
    // 中转速：圆上瞄准 xy，z 用车体中心高度（与 Planner 装甲 z 区分）
    aim_z = ekf_x[4];
  }

  if (std::abs(v_yaw) < 1.0) {
    aim_x = xyz.x();
    aim_y = xyz.y();
    aim_z = xyz.z();
  }

  return Eigen::Vector4d(aim_x, aim_y, aim_z, yaw);
}

HeroAimer::AimPoint HeroAimer::choose_aim_point(const Target & target)
{
  const std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  const int armor_num = static_cast<int>(armor_xyza_list.size());
  if (armor_num <= 0) return {false, Eigen::Vector4d::Zero()};
  if (!target.jumped) return {true, armor_xyza_list[0]};

  return {true, hero_planner_aim_xyza(target)};
}

io::Command HeroAimer::shoot(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  bool to_now, const Eigen::Vector3d & gimbal_ypr)
{
  if (targets.empty()) return {false, false, 0, 0};

  (void)bullet_speed;
  const double fixed_bullet_speed = 11.8;  // m/s，与入参 bullet_speed 无关

  auto target = targets.front();
  Eigen::VectorXd ekf0 = target.ekf_x();
  double w = ekf0[7];

  if (std::abs(w) > w_rotate_) {
    auto tgt = target;
    auto t_decide = timestamp + std::chrono::microseconds(int(
                      tools::delta_time(std::chrono::steady_clock::now(), timestamp) * 1e6));
    tgt.predict(t_decide);
    Eigen::VectorXd ekf_x = tgt.ekf_x();

    const Eigen::Vector4d ap = hero_planner_aim_xyza(tgt);
    const Eigen::Vector3d xyz0(ap[0], ap[1], ap[2]);

    auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
    tools::ResistanceTrajectory traj0(fixed_bullet_speed, d0, xyz0[2]);
    if (traj0.unsolvable) {
      tools::logger()->debug(
        "[HeroAimer] Unsolvable trajectory: {:.2f} {:.2f} {:.2f}", fixed_bullet_speed, d0, xyz0[2]);
      return {false, false, 0, 0};
    }

    debug_aim_point = {true, Eigen::Vector4d(xyz0[0], xyz0[1], xyz0[2], 0)};

    auto yaw0 = std::atan2(xyz0[1], xyz0[0]) + yaw_offset_;
    auto pitch0 = -(traj0.pitch + pitch_offset_);  // 世界坐标系下 pitch 向上为负

    auto t_hit =
      t_decide + std::chrono::microseconds(int((ctrl_to_fire_ + traj0.fly_time) * 1e6));
    tgt.predict(t_hit);
    ekf_x = tgt.ekf_x();
    std::vector<Eigen::Vector4d> armor_xyza_list = tgt.armor_xyza_list();
    auto armor_num = static_cast<int>(armor_xyza_list.size());
    int sig = ekf_x[7] < 0 ? -1 : 1;
    double err_window = (tgt.name == ArmorName::outpost) ? 0.01 : (std::abs(ekf_x[7]) > 5 ? 0.04 : 0.02);
    auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

    for (int aim_id = 0; aim_id < armor_num; aim_id++) {
      auto delta = sig * (-armor_xyza_list[aim_id][3] + center_yaw);
      auto z_armor = armor_xyza_list[aim_id][2];
      bool yaw_ok = (aim_id != last_hit_id_) && (delta <= err_window) && (delta >= 0);
      bool z_ok = std::abs(z_armor - xyz0[2]) <= 0.02;
      if (yaw_ok && z_ok) {
        last_hit_id_ = aim_id;
        return {true, true, yaw0, pitch0};
      }
    }
    return {true, false, yaw0, pitch0};
  }

  auto t_fire = timestamp;
  if (to_now) {
    double dt =
      tools::delta_time(std::chrono::steady_clock::now(), timestamp) + ctrl_to_fire_;
    t_fire = timestamp + std::chrono::microseconds(int(dt * 1e6));
    target.predict(t_fire);
  }

  auto aim_point0 = choose_aim_point(target);
  debug_aim_point = aim_point0;
  if (!aim_point0.valid) {
    return {false, false, 0, 0};
  }

  Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
  auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
  tools::ResistanceTrajectory tr0(fixed_bullet_speed, d0, xyz0[2]);
  if (tr0.unsolvable) {
    tools::logger()->debug(
      "[HeroAimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", fixed_bullet_speed, d0, xyz0[2]);
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  auto t_hit = t_fire + std::chrono::microseconds(int((ctrl_to_fire_ + tr0.fly_time) * 1e6));
  target.predict(t_hit);

  auto aim_point1 = choose_aim_point(target);
  debug_aim_point = aim_point1;
  if (!aim_point1.valid) {
    return {true, false, 0, 0};
  }

  Eigen::Vector3d xyz1 = aim_point1.xyza.head(3);
  auto d1 = std::sqrt(xyz1[0] * xyz1[0] + xyz1[1] * xyz1[1]);
  tools::ResistanceTrajectory tr1(fixed_bullet_speed, d1, xyz1[2]);
  if (tr1.unsolvable) {
    tools::logger()->debug(
      "[HeroAimer] Unsolvable trajectory1: {:.2f} {:.2f} {:.2f}", fixed_bullet_speed, d1, xyz1[2]);
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  if (std::abs(tr1.fly_time - tr0.fly_time) > 0.1) {
    tools::logger()->debug("[HeroAimer] Large time error: {:.3f}", tr1.fly_time - tr0.fly_time);
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  auto yaw = std::atan2(xyz1[1], xyz1[0]) + yaw_offset_;
  auto pitch = -(tr1.pitch + pitch_offset_);

  // 火控 看差距大不大
  bool aligned = (std::abs(gimbal_ypr[0] - yaw) < max_fire_error_yaw_) &&
                 (std::abs(gimbal_ypr[1] - pitch) < max_fire_error_pitch_);
  return {true, aligned, yaw, pitch};
}

}  // namespace auto_aim
