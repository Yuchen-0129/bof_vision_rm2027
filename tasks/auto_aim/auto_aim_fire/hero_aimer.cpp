#include "hero_aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <limits>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
HeroAimer::HeroAimer(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  shoot_delay_ = yaml["shoot_delay"].as<double>();
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;
  max_fire_error_yaw_ = yaml["max_fire_error_yaw"].as<double>();
  max_fire_error_pitch_ = yaml["max_fire_error_pitch"].as<double>();
  max_shoot_angle_ = yaml["max_shoot_angle"].as<double>();
}

Eigen::Vector3d HeroAimer::get_outpost_hit_xyz(const Target & target) const
{
  auto ekf_x = target.ekf_x();
  Eigen::Vector3d center = {ekf_x[0], ekf_x[2], ekf_x[4]};
  const auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  Eigen::Vector2d center_xy = center.head(2);
  const double horizonal_distance = center_xy.norm() - ekf_x[8];
  Eigen::Vector2d hit_point_xy = (horizonal_distance / center_xy.norm()) * center_xy;

  double z = ekf_x[4];
  const auto armors = target.armor_xyza_list();
  if (!armors.empty()) {
    int best = 0;
    double best_d = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < armors.size(); ++i) {
      const double d = std::abs(tools::limit_rad(armors[i][3] - center_yaw));
      if (d < best_d) {
        best_d = d;
        best = static_cast<int>(i);
      }
    }
    z = armors[static_cast<size_t>(best)][2];
  }
  return {hit_point_xy[0], hit_point_xy[1], z};
}

Eigen::Vector3d HeroAimer::get_top_front(const Target & target_origin) const
{
  auto ekf_x = target_origin.ekf_x();

  Eigen::Vector3d center = {ekf_x[0], ekf_x[2], ekf_x[4]};
  const auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  Eigen::Vector2d center_xy = center.head(2);
  const double horizonal_distance = center_xy.norm() - ekf_x[8];
  Eigen::Vector2d hit_point_xy = (horizonal_distance / center_xy.norm()) * center_xy;

  const auto armors = target_origin.armor_xyza_list();
  const int armor_num = static_cast<int>(armors.size());
  if (armor_num <= 0) {
    return {hit_point_xy[0], hit_point_xy[1], ekf_x[4]};
  }

  const double w = ekf_x[7];
  if (std::fabs(w) < 1e-6) {
    return {hit_point_xy[0], hit_point_xy[1], ekf_x[4]};
  }
  if (std::fabs(w) > 3) {
    return {hit_point_xy[0], hit_point_xy[1], ekf_x[4]};
  }

  const double T = 2 * M_PI / w;
  const int sig_w = w > 0 ? 1 : -1;
  int armor_id = -1;
  double dt_min = INFINITY;
  double t = 0;
  for (int aim_id = 0; aim_id < armor_num; aim_id++) {
    if (sig_w == 1) {
      t = (armors[static_cast<size_t>(aim_id)][3] > center_yaw)
            ? T - (armors[static_cast<size_t>(aim_id)][3] - center_yaw) / w
            : (center_yaw - armors[static_cast<size_t>(aim_id)][3]) / w;
    } else {
      t = (armors[static_cast<size_t>(aim_id)][3] < center_yaw)
            ? T + (armors[static_cast<size_t>(aim_id)][3] - center_yaw) / w
            : (-center_yaw + armors[static_cast<size_t>(aim_id)][3]) / w;
    }
    if (t < dt_min) {
      armor_id = aim_id;
      dt_min = t;
    }
  }
  return {hit_point_xy[0], hit_point_xy[1], armors[static_cast<size_t>(armor_id)][2]};
}

io::Command HeroAimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  bool to_now, Eigen::Vector3d ypr)
{
  if (targets.empty()) {
    return {false, false, 0, 0};
  }

  auto target = targets.front();
  auto target_predicted = target;
  auto t_fire = timestamp;
  if (bullet_speed <= 0.0) {
    bullet_speed = 11.8;
  }
  auto ekf_x = target_predicted.ekf_x();

  if (std::abs(ekf_x[7]) > 1) {
    auto target_rotate = target;
    const auto t_decide = std::chrono::steady_clock::now();
    target_rotate.predict(t_decide);
    ekf_x = target_rotate.ekf_x();

    Eigen::Vector3d xyz0;
    if (target_rotate.name == ArmorName::outpost) {
      xyz0 = get_outpost_hit_xyz(target_rotate);
    } else {
      xyz0 = get_top_front(target_rotate);
    }

    const double yaw = std::atan2(xyz0[1], xyz0[0]) + yaw_offset_;

    const double d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
    tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
    if (trajectory0.unsolvable) {
      tools::logger()->debug("[HeroAimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
      return {false, false, 0, 0};
    }
    const double pitch = -(trajectory0.pitch + pitch_offset_);

    debug_aim_point_ = {true, {xyz0[0], xyz0[1], xyz0[2], 0}};

    const auto t_hit = t_decide + std::chrono::microseconds(
                                   static_cast<int>((shoot_delay_ + trajectory0.fly_time) * 1e6));
    target_rotate.predict(t_hit);
    const auto armors_hit = target_rotate.armor_xyza_list();
    const int armor_num = static_cast<int>(armors_hit.size());
    const int sig = ekf_x[7] < 0 ? -1 : +1;
    const double error =
      (target_rotate.name == ArmorName::outpost ? 0.01 : (std::fabs(ekf_x[7]) > 5 ? 0.04 : 0.02));
    const auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

    for (int aim_id = 0; aim_id < armor_num; ++aim_id) {
      const double armor_yaw = armors_hit[static_cast<size_t>(aim_id)][3];
      const double yaw_err = sig * (-armor_yaw + center_yaw);
      if (yaw_err <= error && yaw_err >= 0 &&
          std::fabs(armors_hit[static_cast<size_t>(aim_id)][2] - xyz0[2]) <= 0.01) {
        tools::logger()->info("########## fire ##########");
        return {true, true, yaw, pitch};
      }
    }
    return {true, false, yaw, pitch};
  }

  if (to_now) {
    const double dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + shoot_delay_;
    t_fire = timestamp + std::chrono::microseconds(static_cast<int>(dt * 1e6));
    target_predicted.predict(t_fire);
  }

  const auto aim_point0 = choose_aim_point(target_predicted);
  debug_aim_point_ = aim_point0;
  if (!aim_point0.valid) {
    return {false, false, 0, 0};
  }

  Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
  double d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
  tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
  if (trajectory0.unsolvable) {
    tools::logger()->debug("[HeroAimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
    debug_aim_point_.valid = false;
    return {false, false, 0, 0};
  }

  const auto t_hit = t_fire + std::chrono::microseconds(static_cast<int>(trajectory0.fly_time * 1e6));
  target_predicted.predict(t_hit);

  const auto aim_point1 = choose_aim_point(target_predicted);
  debug_aim_point_ = aim_point1;
  if (!aim_point1.valid) {
    return {false, false, 0, 0};
  }

  const Eigen::Vector3d xyz1 = aim_point1.xyza.head(3);
  const double d1 = std::sqrt(xyz1[0] * xyz1[0] + xyz1[1] * xyz1[1]);
  tools::Trajectory trajectory1(bullet_speed, d1, xyz1[2]);
  if (trajectory1.unsolvable) {
    tools::logger()->debug("[HeroAimer] Unsolvable trajectory1: {:.2f} {:.2f} {:.2f}", bullet_speed, d1, xyz1[2]);
    debug_aim_point_.valid = false;
    return {false, false, 0, 0};
  }

  const double time_error = trajectory1.fly_time - trajectory0.fly_time;
  if (std::abs(time_error) > 0.1) {
    tools::logger()->debug("[HeroAimer] Large time error: {:.3f}", time_error);
    debug_aim_point_.valid = false;
    return {false, false, 0, 0};
  }

  const double yaw = std::atan2(xyz1[1], xyz1[0]) + yaw_offset_;
  const double pitch = -(trajectory1.pitch + pitch_offset_);

  bool is_fire = false;
  if ((std::abs(ypr[0] - yaw) < max_fire_error_yaw_) && (std::abs(ypr[1] - pitch) < max_fire_error_pitch_)) {
    // is_fire = true;
  }

  return {true, is_fire, yaw, pitch};
}

HeroAimPoint HeroAimer::choose_aim_point(const Target & target)
{
  Eigen::VectorXd ekf_x = target.ekf_x();
  std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  const auto armor_num = static_cast<int>(armor_xyza_list.size());

  if (!target.jumped) return {true, armor_xyza_list[0]};

  const auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  int first_id = -1, second_id = -1;
  float min1 = std::numeric_limits<float>::max();
  float min2 = std::numeric_limits<float>::max();

  for (int aim_id = 0; aim_id < armor_num; aim_id++) {
    const float delta = std::abs(armor_xyza_list[static_cast<size_t>(aim_id)][3] - center_yaw);

    if (delta < min1) {
      min2 = min1;
      second_id = first_id;

      min1 = delta;
      first_id = aim_id;
    } else if (delta < min2) {
      min2 = delta;
      second_id = aim_id;
    }
  }

  int dmin_first_id = -1, dmin_second_id = -1;
  float dmin1 = std::numeric_limits<float>::max();
  float dmin2 = std::numeric_limits<float>::max();

  for (int aim_id = 0; aim_id < armor_num; aim_id++) {
    const float delta =
      armor_xyza_list[static_cast<size_t>(aim_id)][0] * armor_xyza_list[static_cast<size_t>(aim_id)][0] +
      armor_xyza_list[static_cast<size_t>(aim_id)][1] * armor_xyza_list[static_cast<size_t>(aim_id)][1];

    if (delta < dmin1) {
      dmin2 = dmin1;
      dmin_second_id = dmin_first_id;

      dmin1 = delta;
      dmin_first_id = aim_id;
    } else if (delta < dmin2) {
      dmin2 = delta;
      dmin_second_id = aim_id;
    }
  }

  const Eigen::Vector3d xyz1 = armor_xyza_list[static_cast<size_t>(first_id)].head(3);
  const Eigen::Vector3d xyz2 = armor_xyza_list[static_cast<size_t>(second_id)].head(3);

  const double yaw1 = std::atan2(xyz1[1], xyz1[0]);
  const double yaw2 = std::atan2(xyz2[1], xyz2[0]);

  if (first_id != dmin_first_id && first_id != dmin_second_id) return {true, armor_xyza_list[static_cast<size_t>(second_id)]};

  if (second_id != dmin_first_id && second_id != dmin_second_id) return {true, armor_xyza_list[static_cast<size_t>(first_id)]};

  if (std::abs(armor_xyza_list[static_cast<size_t>(first_id)][3] - yaw1) > max_shoot_angle_ / 57.3)
    return {true, armor_xyza_list[static_cast<size_t>(second_id)]};

  if (std::abs(armor_xyza_list[static_cast<size_t>(second_id)][3] - yaw2) > max_shoot_angle_ / 57.3)
    return {true, armor_xyza_list[static_cast<size_t>(first_id)]};

  if (lock_id_ != first_id && lock_id_ != second_id)
    lock_id_ =
      (std::abs(armor_xyza_list[static_cast<size_t>(first_id)][3] - yaw1) <
       std::abs(armor_xyza_list[static_cast<size_t>(second_id)][3] - yaw2))
        ? first_id
        : second_id;

  return {true, armor_xyza_list[static_cast<size_t>(lock_id_)]};
}

}  // namespace auto_aim
