#ifndef AUTO_AIM__OUTPOST_AIMER_HPP
#define AUTO_AIM__OUTPOST_AIMER_HPP

#include <string>
#include <vector>

#include "io/command.hpp"
#include "tasks/auto_aim/auto_aim_track/outpost_target.hpp"

namespace auto_aim
{
// 前哨站专用瞄准/开火决策，与整车 Aimer 平行，互不影响。
//
// 为什么不复用整车 Aimer：
//   整车 Aimer 的 choose_aim_point 要处理"平移 / 低速 / 小陀螺"三种行为、
//   要靠 lock_id_ 防止在两块板间抖，还要迭代解飞行时间。前哨站的情况完全不同：
//     - 转速已知恒定 → 不需要按转速分档，也不需要迭代（飞行时间一次就能算准）
//     - 3 块板互隔 120°，同一时刻最多一块朝向我方 → 不存在"两块都45°"的抖动
//     - 打的是 5m 开外 → 弹道飞行 217ms 内目标转 31°，必须按**命中时刻**选板
//   把这些塞进整车 Aimer 只会让两边互相将就。
class OutpostAimer
{
public:
  explicit OutpostAimer(const std::string & config_path);

  // targets 为空或没锁定方向时返回 control=false（不接管云台）。
  io::Command aim(
    const std::vector<OutpostTarget> & targets, double bullet_speed, bool to_now = true);

  // 最近一次的瞄准点，供 debug 画框
  Eigen::Vector4d debug_xyza = Eigen::Vector4d::Zero();
  bool debug_valid = false;

private:
  double yaw_offset_;
  double pitch_offset_;
  double shoot_delay_;    // 开火延迟：cmd 发出到子弹离膛
  double max_delta_;      // 可打的板法线偏角上限 rad
  double fire_delta_;     // 允许开火的板法线偏角上限 rad（比 max_delta_ 紧）
  bool auto_fire_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__OUTPOST_AIMER_HPP
