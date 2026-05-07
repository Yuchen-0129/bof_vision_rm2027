#include "resistance_trajectory.hpp"

#include <cmath>

namespace tools
{
namespace
{
constexpr double g = 9.7946;
constexpr int kMaxIter = 100;
// 水平方向二次空气阻力 F = k0 * v^2；k1 = k0/m = (Cd * rho * pi * r^2) / (2 * m)，量纲 1/m
constexpr double k1 = (0.47 * 1.169 * M_PI * 0.02125 * 0.02125) / (2 * 0.041);
}  // namespace

ResistanceTrajectory::ResistanceTrajectory(const double v0, const double d, const double h)
{
  if (d < 1e-6) {
    unsolvable = true;
    return;
  }

  double theta = std::atan2(h, d);
  const double A = (std::exp(k1 * d) - 1.0) / (k1 * v0);  // T = A / cos(theta)

  for (int i = 0; i < kMaxIter; i++) {
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);
    const double T = A / cos_theta;

    // 残差 f(theta) = h - v0*sin(theta)*T + 0.5*g*T^2
    const double f = h - v0 * sin_theta * T + 0.5 * g * T * T;
    if (std::fabs(f) < 1e-6) {
      break;
    }

    // df/dtheta = -v0*T/cos(theta) + g*T^2*tan(theta)
    const double df = -v0 * T / cos_theta + g * T * T * sin_theta / cos_theta;
    if (std::fabs(df) < 1e-12) {
      unsolvable = true;
      return;
    }
    theta -= f / df;
  }

  unsolvable = false;
  fly_time = A / std::cos(theta);
  pitch = theta;
}

}  // namespace tools
