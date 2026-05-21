#include "tasks/auto_aim/auto_aim_track/target.hpp"

#include <numeric>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{

// 构造函数
Target::Target(
  const Armor & armor, 
  std::chrono::steady_clock::time_point t, 
  double radius, int armor_num,
  Eigen::VectorXd P0_dig)
: name(armor.name),
  armor_type(armor.type),
  jumped(false),
  last_id(0),
  update_count_(0),
  armor_num_(armor_num),
  t_(t),
  is_switch_(false),
  is_converged_(false),
  switch_count_(0)
{
  auto r = radius;
  priority = armor.priority;
  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  // 旋转中心的坐标
  auto center_x = xyz[0] + r * std::cos(ypr[0]);
  auto center_y = xyz[1] + r * std::sin(ypr[0]);
  auto center_z = xyz[2];

  // x vx y vy z vz a w r l h
  // a: angle
  // w: angular velocity
  // l: r2 - r1
  // h: z2 - z1
  Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0}};  //初始化预测量
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  // 这步再看看
  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
}


Target::Target(double x, double vyaw, double radius, double h) : armor_num_(4)
{
  Eigen::VectorXd x0{{x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h}};
  Eigen::VectorXd P0_dig{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  predict(dt);
  t_ = t;
}



// 预测dt段的状态
void Target::predict(double dt)
{
  // 状态转移矩阵
  // clang-format off
  Eigen::MatrixXd F{
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
  };
  // clang-format on


  
  // Piecewise White Noise Model
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  double v1, v2, q_l, q_h;
  if (name == ArmorName::outpost) {
    v1 = 10;     // 前哨站加速度方差
    v2 = 0.05;   // 前哨站角加速度方差（转速锁更死）
    q_l = 0;     // 高度阶梯由 DZ 表给出，l/h 不再更新
    q_h = 0;
  } else {
    v1 = 100;  // 加速度方差
    v2 = 400;  // 角加速度方差
    q_l = 0;   // 4装甲板的l参数过程噪声（根据实际情况设置）
    q_h = 0;   // 4装甲板的h参数过程噪声（根据实际情况设置）


  }
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;



  // 预测过程噪声偏差的方差
  // clang-format off
  Eigen::MatrixXd Q{

    // x       vx         y     vy        z     vz       a     w     r   l h
    {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, q_l, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, q_h}
  };
  // clang-format on

  
  // 防止夹角求和出现异常值
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  
  // 前哨站转速钳位：sign(w) 一旦锁定就钉到 ±2.51
  if (this->name == ArmorName::outpost && this->outpost_w_sign_locked_) {
    this->ekf_.x[7] = (this->ekf_.x[7] > 0) ? 2.51 : -2.51;
  }

  
    ekf_.predict(F, Q, f);
}


// 更新装甲板
void Target::update(const Armor & armor)
{
  int id;
  auto min_angle_error = 1e10;
  const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();

  // 得到 pair vector: (xyza, id)
  std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
  for (int i = 0; i < armor_num_; i++) {
    xyza_i_list.push_back({xyza_list[i], i});
  }

  // 按照距离排序
  std::sort(
    xyza_i_list.begin(), xyza_i_list.end(),
    [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
      Eigen::Vector3d ypd1 = tools::xyz2ypd(a.first.head(3));
      Eigen::Vector3d ypd2 = tools::xyz2ypd(b.first.head(3));
      return ypd1[2] < ypd2[2];
    });



  // 取前3个distance最小的装甲板
  for (int i = 0; i < 3; i++) {
    const auto & xyza = xyza_i_list[i].first;
    Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
    auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) +
                       std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));

    if (std::abs(angle_error) < std::abs(min_angle_error)) {
      id = xyza_i_list[i].second;
      min_angle_error = angle_error;
    }
  }

  if (id != 0) jumped = true;

  if (id != last_id) {
    is_switch_ = true;
  } else {
    is_switch_ = false;
  }

  if (is_switch_) switch_count_++;

  // 前哨站：在更新 last_id 之前识别旋转方向和相位
  if (name == ArmorName::outpost && is_switch_ && update_count_ >= 1) {
    // 锁定 sign(w)：EKF 中 id 按 yaw 递增（CCW 为正方向）
    // last_id -> id 顺序为 (id - last_id) mod 3，1 = CCW(w>0)，2 = CW(w<0)
    if (!outpost_w_sign_locked_) {
      int delta = ((id - last_id) % 3 + 3) % 3;
      ekf_.x[7] = (delta == 1) ? +2.51 : -2.51;
      outpost_w_sign_locked_ = true;
      tools::logger()->info(
        "[Target] outpost w sign locked: w = {:.2f} (last_id={}, id={})",
        ekf_.x[7], last_id, id);
    }

    // 累积当前观测到 outpost_z_obs_[id]（用世界坐标系下的 z）
    outpost_z_obs_[id].push_back(armor.xyz_in_world[2]);
    // 控制大小，避免无限增长
    if (outpost_z_obs_[id].size() > 32) outpost_z_obs_[id].erase(outpost_z_obs_[id].begin());

    // 锁定 phase：要求至少两块不同 id 都有观测，做最小二乘选 phase*
    if (!outpost_phase_locked_) {
      int seen = 0;
      for (auto & v : outpost_z_obs_) if (!v.empty()) seen++;
      if (seen >= 2) {
        // 对每个候选 phase 求 z_c 的最小二乘解，再计算残差
        int best_phi = 0;
        double best_err = 1e18;
        double best_zc = ekf_.x[4];
        for (int p = 0; p < 3; p++) {
          // z_obs - DZ[p][k] 在所有观测上的均值即 z_c 的 LS 解
          double sum = 0;
          int n = 0;
          for (int k = 0; k < 3; k++) {
            for (double z : outpost_z_obs_[k]) {
              sum += (z - OUTPOST_DZ[p][k]);
              n++;
            }
          }
          if (n == 0) continue;
          double zc = sum / n;
          double err = 0;
          for (int k = 0; k < 3; k++) {
            for (double z : outpost_z_obs_[k]) {
              double r = z - (zc + OUTPOST_DZ[p][k]);
              err += r * r;
            }
          }
          if (err < best_err) { best_err = err; best_phi = p; best_zc = zc; }
        }
        outpost_phase_ = best_phi;
        outpost_phase_locked_ = true;
        ekf_.x[4] = best_zc;  // 把 z_c 直接拉到 LS 解
        tools::logger()->info(
          "[Target] outpost phase locked: phase={}, z_c={:.3f}, rms={:.3f}",
          best_phi, best_zc, std::sqrt(best_err));
      }
    }
  }

  last_id = id;
  update_count_++;

  // 更新 yaw pitch distance angle
  update_ypda(armor, id);
}



// 观测 和 预测 更新
void Target::update_ypda(const Armor & armor, int id)
{
  // 观测噪声
  //观测jacobi
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);
  // Eigen::VectorXd R_dig{{4e-3, 4e-3, 1, 9e-2}};
  auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
  Eigen::VectorXd R_dig{
    {4e-3, 4e-3, log(std::abs(delta_angle) + 1) + 1, log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-2}};
  //测量过程噪声偏差的方差
  Eigen::MatrixXd R = R_dig.asDiagonal();// 转换为对角矩阵

  // R = [
  //   [4e-3,    0,    0,    0    ]
  //   [0,    4e-3,    0,    0    ]
  //   [0,       0,  var2,   0    ]  // var2 = log(|delta_angle|+1) + 1
  //   [0,       0,    0,  var3   ]  // var3 = log(|distance|+1)/200 + 9e-2
  // ]



  // h(x) 表示从状态 x（EKF 预测的状态）计算出预测的观测值

  // 定义非线性转换函数h: x -> z
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = h_armor_xyz(x, id);
    Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
    auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  // 防止夹角求差出现异常值
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  // z反映的才是真正的识别出的观测数值
  const Eigen::VectorXd & ypd = armor.ypd_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;
  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};  //获得观测量

  ekf_.update(z, H, R, h, z_subtract);
}



Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }



// 返回 ekf_x的装甲板序列
std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> _armor_xyza_list;

  for (int i = 0; i < armor_num_; i++) {
    auto angle = tools::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
    _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return _armor_xyza_list;
}




// 判断滤波器是否发散
bool Target::diverged() const
{
  auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
  // auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;

  // if (r_ok && l_ok) return false;

  if (name == ArmorName::outpost) {
    // 前哨站：高度阶梯由 DZ 表给出，不再依赖 l/h；只看 r 是否合理
    if (r_ok) return false;
} else {
    // 4装甲板：检查半径差
    auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;
    if (r_ok && l_ok) return false;
}


  tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
  return true;
}

// 判断滤波器是否收敛
bool Target::convergened()
{
  if (this->name != ArmorName::outpost && update_count_ > 3 && !this->diverged()) {
    is_converged_ = true;
  }

  //前哨站：phase 锁定且 update_count_ > 4 即可视为收敛
  if (
    this->name == ArmorName::outpost && this->outpost_phase_locked_ &&
    update_count_ > 4 && !this->diverged()) {
    is_converged_ = true;
  }

  return is_converged_;
}

// 通过ekf_x（车体中心）计算出装甲板中心的坐标（考虑长短轴）
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);
  auto outpost = armor_num_ == 3;


  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  // auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];
  auto armor_z = (outpost) ? getoutpost_armor_z(id, x) : (use_l_h) ? x[4] + x[10] : x[4];


  
  return {armor_x, armor_y, armor_z};
}


// 没看懂VvV
// 观测雅可比矩阵
Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);// 角度
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);// 维护长短轴半径
  auto outpost = (armor_num_ == 3); 

  
  auto r = (use_l_h) ? x[8] + x[9] : x[8];// 半径
  auto dx_da = r * std::sin(angle);// 角度对x的偏导
  auto dy_da = -r * std::cos(angle);// 角度对y的偏导

  auto dx_dr = -std::cos(angle);// 半径对x的偏导
  auto dy_dr = -std::sin(angle);// 半径对y的偏导
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;// 半径差对x的偏导
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;// 半径差对y的偏导

  auto dz_dh = (use_l_h) ? 1.0 : 0.0;// 高度对z的偏导
  // 前哨站新模型：z 偏移 DZ[phase][id] 是常量，对 x[9]/x[10] 偏导为 0
  (void)outpost;

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0,     dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
  // clang-format on

  // 得到装甲板观测的雅可比矩阵
  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
}


bool Target::checkinit() { return isinit; }


}  // namespace auto_aim
