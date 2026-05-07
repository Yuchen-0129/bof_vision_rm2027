#ifndef TOOLS__RESISTANCE_TRAJECTORY_HPP
#define TOOLS__RESISTANCE_TRAJECTORY_HPP

namespace tools
{
// 水平方向二次空气阻力下的弹道近似解；pitch 为瞄准仰角，与 atan2(h,d) 同号（目标更高时一般为正）
struct ResistanceTrajectory
{
  bool unsolvable;
  double fly_time;
  double pitch;

  // v0 子弹初速度大小 (m/s)，d 目标水平距离 (m)，h 目标竖直高度 (m)
  ResistanceTrajectory(double v0, double d, double h);
};

}  // namespace tools

#endif  // TOOLS__RESISTANCE_TRAJECTORY_HPP
