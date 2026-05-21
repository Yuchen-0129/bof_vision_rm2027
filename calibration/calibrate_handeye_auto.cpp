// 非交互的手眼标定工具，自动跑全部数据 + 按重投影残差筛选最优子集
// 用法：./build/calibrate_handeye_auto <input-folder>
//   -c=<yaml>          配置文件（默认 configs/calibration.yaml）
//   --top-n=<N>        只用残差最小的前 N 张（0 = 全用）
//   --max-resid=<f>    丢弃单帧 PnP 残差 > 阈值（像素）
//   --skip=<list>      手动跳过的图像编号，逗号分隔
#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <algorithm>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <set>
#include <sstream>

#include "tools/img_tools.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{top-n          | 0                        | 只用 PnP 残差最小的前 N 张 (0=全部)}"
  "{max-resid      | 999                      | 丢弃单帧 PnP 残差 > 阈值（像素）}"
  "{skip           |                          | 手动跳过的图像编号，逗号分隔}"
  "{@input-folder  | assets/img_with_q        | 输入文件夹路径   }";

struct Sample
{
  int idx;
  cv::Mat R_g2w;
  cv::Mat t_g2w;
  cv::Mat rvec;
  cv::Mat tvec;
  double resid_px;  // 单帧 PnP 重投影残差（平均像素）
  Eigen::Vector3d ypr;  // 云台姿态（degree）
};

std::vector<cv::Point3f> centers_3d(const cv::Size & pattern_size, const float center_distance)
{
  std::vector<cv::Point3f> centers;
  for (int i = 0; i < pattern_size.height; i++)
    for (int j = 0; j < pattern_size.width; j++)
      centers.push_back({j * center_distance, i * center_distance, 0});
  return centers;
}

Eigen::Quaterniond read_q(const std::string & q_path)
{
  std::ifstream q_file(q_path);
  double w, x, y, z;
  q_file >> w >> x >> y >> z;
  return {w, x, y, z};
}

std::set<int> parse_skip(const std::string & s)
{
  std::set<int> result;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    if (item.empty()) continue;
    try {
      result.insert(std::stoi(item));
    } catch (...) {
    }
  }
  return result;
}

void run_calibration(
  const std::vector<Sample> & samples, const std::vector<double> & R_gimbal2imubody_data,
  const std::string & label)
{
  if (samples.size() < 3) {
    fmt::print("[{}] 样本太少 ({}), 跳过\n", label, samples.size());
    return;
  }

  std::vector<cv::Mat> R_g2w_list, t_g2w_list, rvecs, tvecs;
  for (const auto & s : samples) {
    R_g2w_list.push_back(s.R_g2w);
    t_g2w_list.push_back(s.t_g2w);
    rvecs.push_back(s.rvec);
    tvecs.push_back(s.tvec);
  }

  cv::Mat R_camera2gimbal, t_camera2gimbal;
  cv::calibrateHandEye(R_g2w_list, t_g2w_list, rvecs, tvecs, R_camera2gimbal, t_camera2gimbal);
  t_camera2gimbal /= 1e3;

  Eigen::Matrix3d R_c2g_eigen;
  cv::cv2eigen(R_camera2gimbal, R_c2g_eigen);
  Eigen::Matrix3d R_gimbal2ideal{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
  Eigen::Matrix3d R_camera2ideal = R_gimbal2ideal * R_c2g_eigen;
  Eigen::Vector3d ypr = tools::eulers(R_camera2ideal, 1, 0, 2) * 57.3;

  std::vector<double> R_data(R_camera2gimbal.begin<double>(), R_camera2gimbal.end<double>());
  std::vector<double> t_data(t_camera2gimbal.begin<double>(), t_camera2gimbal.end<double>());

  fmt::print("\n========== {} (n={}) ==========\n", label, samples.size());
  fmt::print(
    "# 相机同理想情况的偏角: yaw{:+.3f} pitch{:+.3f} roll{:+.3f} degree\n", ypr[0], ypr[1], ypr[2]);
  fmt::print("# t_camera2gimbal_norm: {:.4f} m\n", cv::norm(t_camera2gimbal));

  YAML::Emitter result;
  result << YAML::BeginMap;
  result << YAML::Key << "R_camera2gimbal";
  result << YAML::Value << YAML::Flow << R_data;
  result << YAML::Key << "t_camera2gimbal";
  result << YAML::Value << YAML::Flow << t_data;
  result << YAML::EndMap;
  fmt::print("{}\n", result.c_str());
}

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_folder = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto top_n = cli.get<int>("top-n");
  auto max_resid = cli.get<double>("max-resid");
  auto skip_str = cli.get<std::string>("skip");
  auto skip_set = parse_skip(skip_str);

  // 读取 yaml
  auto yaml = YAML::LoadFile(config_path);
  auto pattern_cols = yaml["pattern_cols"].as<int>();
  auto pattern_rows = yaml["pattern_rows"].as<int>();
  auto center_distance_mm = yaml["center_distance_mm"].as<double>();
  auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();

  cv::Size pattern_size(pattern_cols, pattern_rows);
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_gimbal2imubody(R_gimbal2imubody_data.data());
  cv::Matx33d camera_matrix(camera_matrix_data.data());
  cv::Mat distort_coeffs(distort_coeffs_data);

  auto centers_3d_ = centers_3d(pattern_size, center_distance_mm);

  // 加载所有样本
  std::vector<Sample> samples;
  int n_total = 0, n_failed_detect = 0, n_skipped = 0, n_high_resid = 0;
  for (int i = 1; i <= 9999; i++) {
    auto img_path = fmt::format("{}/{}.jpg", input_folder, i);
    auto q_path = fmt::format("{}/{}.txt", input_folder, i);
    auto img = cv::imread(img_path);
    if (img.empty()) break;
    n_total++;

    if (skip_set.count(i)) {
      n_skipped++;
      continue;
    }

    Eigen::Quaterniond q = read_q(q_path);

    // 识别标定板
    std::vector<cv::Point2f> centers_2d;
    auto success = cv::findCirclesGrid(img, pattern_size, centers_2d);
    if (!success) {
      n_failed_detect++;
      continue;
    }

    // PnP
    cv::Mat rvec, tvec;
    cv::solvePnP(
      centers_3d_, centers_2d, camera_matrix, distort_coeffs, rvec, tvec, false,
      cv::SOLVEPNP_IPPE);

    // 重投影残差
    std::vector<cv::Point2f> proj;
    cv::projectPoints(centers_3d_, rvec, tvec, camera_matrix, distort_coeffs, proj);
    double resid = 0;
    for (size_t k = 0; k < proj.size(); k++) resid += cv::norm(proj[k] - centers_2d[k]);
    resid /= proj.size();

    if (resid > max_resid) {
      n_high_resid++;
      continue;
    }

    // 云台姿态（用于诊断视角分布）
    Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
    Eigen::Matrix3d R_g2w_eigen =
      R_gimbal2imubody.transpose() * R_imubody2imuabs * R_gimbal2imubody;
    Eigen::Vector3d ypr = tools::eulers(R_g2w_eigen, 2, 1, 0) * 57.3;

    Sample s;
    s.idx = i;
    cv::Mat R_g2w_cv;
    cv::eigen2cv(R_g2w_eigen, R_g2w_cv);
    s.R_g2w = R_g2w_cv;
    s.t_g2w = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    s.rvec = rvec;
    s.tvec = tvec;
    s.resid_px = resid;
    s.ypr = ypr;
    samples.push_back(s);
  }

  fmt::print(
    "\n=== 数据统计 ===\n"
    "总图像: {}\n"
    "识别失败: {}\n"
    "手动跳过: {}\n"
    "残差>{:.2f}px 丢弃: {}\n"
    "可用样本: {}\n",
    n_total, n_failed_detect, n_skipped, max_resid, n_high_resid, samples.size());

  // 视角分布统计
  if (!samples.empty()) {
    double yaw_min = 1e9, yaw_max = -1e9;
    double pitch_min = 1e9, pitch_max = -1e9;
    double resid_min = 1e9, resid_max = 0, resid_mean = 0;
    for (const auto & s : samples) {
      yaw_min = std::min(yaw_min, s.ypr[0]);
      yaw_max = std::max(yaw_max, s.ypr[0]);
      pitch_min = std::min(pitch_min, s.ypr[1]);
      pitch_max = std::max(pitch_max, s.ypr[1]);
      resid_min = std::min(resid_min, s.resid_px);
      resid_max = std::max(resid_max, s.resid_px);
      resid_mean += s.resid_px;
    }
    resid_mean /= samples.size();
    fmt::print(
      "\n视角分布:\n"
      "  yaw   范围: [{:+.2f}, {:+.2f}] (跨度 {:.2f}°)\n"
      "  pitch 范围: [{:+.2f}, {:+.2f}] (跨度 {:.2f}°)\n"
      "PnP 残差: min={:.3f}, max={:.3f}, mean={:.3f} px\n",
      yaw_min, yaw_max, yaw_max - yaw_min, pitch_min, pitch_max, pitch_max - pitch_min, resid_min,
      resid_max, resid_mean);
  }

  // baseline: 全部用上
  run_calibration(samples, R_gimbal2imubody_data, "ALL");

  // 子集：按残差筛选
  if (top_n > 0 && top_n < (int)samples.size()) {
    auto sorted = samples;
    std::sort(
      sorted.begin(), sorted.end(),
      [](const Sample & a, const Sample & b) { return a.resid_px < b.resid_px; });
    sorted.resize(top_n);
    run_calibration(sorted, R_gimbal2imubody_data, fmt::format("TOP-{} (lowest resid)", top_n));
  }

  // 子集：每个 (yaw_bin, pitch_bin) 取 1 张（视角均匀）
  // 把 yaw 切成 10 个 bin、pitch 切成 5 个 bin，每格挑残差最小那张
  if (samples.size() > 30) {
    double yaw_min = 1e9, yaw_max = -1e9, pitch_min = 1e9, pitch_max = -1e9;
    for (const auto & s : samples) {
      yaw_min = std::min(yaw_min, s.ypr[0]);
      yaw_max = std::max(yaw_max, s.ypr[0]);
      pitch_min = std::min(pitch_min, s.ypr[1]);
      pitch_max = std::max(pitch_max, s.ypr[1]);
    }
    constexpr int YBIN = 10, PBIN = 5;
    std::map<int, Sample> grid;
    for (const auto & s : samples) {
      int yi = std::min<int>(
        YBIN - 1, std::max(0, int((s.ypr[0] - yaw_min) / (yaw_max - yaw_min + 1e-6) * YBIN)));
      int pi = std::min<int>(
        PBIN - 1, std::max(0, int((s.ypr[1] - pitch_min) / (pitch_max - pitch_min + 1e-6) * PBIN)));
      int key = yi * PBIN + pi;
      auto it = grid.find(key);
      if (it == grid.end() || s.resid_px < it->second.resid_px) grid[key] = s;
    }
    std::vector<Sample> uniform;
    for (auto & kv : grid) uniform.push_back(kv.second);
    run_calibration(
      uniform, R_gimbal2imubody_data, fmt::format("UNIFORM (one per yaw/pitch grid cell)"));
  }

  return 0;
}
