#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "tools/img_tools.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | configs/calibration_chessboard.yaml | yaml配置文件路径 }"
  "{@input-folder  | assets/chessboard_calib   | 输入文件夹路径   }";

namespace
{
bool detect_chessboard_corners_refine(
  const cv::Mat & img, const cv::Size & pattern_size, double preview_scale,
  std::vector<cv::Point2f> & corners)
{
  cv::Mat gray;
  cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  cv::Mat search_gray = gray;
  if (preview_scale < 0.999) {
    cv::resize(gray, search_gray, {}, preview_scale, preview_scale, cv::INTER_AREA);
  }
  const int flags =
    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;
  std::vector<cv::Point2f> search_corners;
  if (!cv::findChessboardCorners(search_gray, pattern_size, search_corners, flags)) {
    corners.clear();
    return false;
  }
  corners = search_corners;
  if (preview_scale < 0.999) {
    for (auto & c : corners) {
      c.x /= static_cast<float>(preview_scale);
      c.y /= static_cast<float>(preview_scale);
    }
  }
  cv::cornerSubPix(
    gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));
  return true;
}
}  // namespace

/// 与 calibrate_robotworld_handeye.cpp 相同的竖直标定板模型（间距用 square_size_mm）
std::vector<cv::Point3f> board_corners_3d_vertical(
  const cv::Size & pattern_size, float square_size_mm)
{
  std::vector<cv::Point3f> centers_3d;
  for (int i = 0; i < pattern_size.height; i++) {
    for (int j = 0; j < pattern_size.width; j++) {
      float x = 0;
      float y = (-j + 0.5f * pattern_size.width) * square_size_mm;
      float z = (-i + 0.5f * pattern_size.height) * square_size_mm;
      centers_3d.push_back({x, y, z});
    }
  }
  return centers_3d;
}

Eigen::Quaterniond read_q(const std::string & q_path)
{
  std::ifstream q_file(q_path);
  double w, x, y, z;
  q_file >> w >> x >> y >> z;
  return {w, x, y, z};
}

void load(
  const std::string & input_folder, const std::string & config_path,
  std::vector<double> & R_gimbal2imubody_data, std::vector<cv::Mat> & R_world2gimbal_list,
  std::vector<cv::Mat> & t_world2gimbal_list, std::vector<cv::Mat> & rvecs,
  std::vector<cv::Mat> & tvecs)
{
  auto yaml = YAML::LoadFile(config_path);
  auto pattern_cols = yaml["pattern_cols"].as<int>();
  auto pattern_rows = yaml["pattern_rows"].as<int>();
  auto square_size_mm = yaml["square_size_mm"].as<double>();
  auto preview_scale = yaml["preview_scale"].as<double>(0.5);
  R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();

  cv::Size pattern_size(pattern_cols, pattern_rows);
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_gimbal2imubody(R_gimbal2imubody_data.data());
  cv::Matx33d camera_matrix(camera_matrix_data.data());
  cv::Mat distort_coeffs(distort_coeffs_data);

  for (int i = 1; true; i++) {
    auto img_path = fmt::format("{}/{}.jpg", input_folder, i);
    auto q_path = fmt::format("{}/{}.txt", input_folder, i);
    auto img = cv::imread(img_path);
    Eigen::Quaterniond q = read_q(q_path);
    if (img.empty()) break;

    Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
    Eigen::Matrix3d R_gimbal2world =
      R_gimbal2imubody.transpose() * R_imubody2imuabs * R_gimbal2imubody;
    Eigen::Vector3d ypr = tools::eulers(R_gimbal2world, 2, 1, 0) * 57.3;

    auto drawing = img.clone();
    tools::draw_text(drawing, fmt::format("yaw   {:.2f}", ypr[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(drawing, fmt::format("pitch {:.2f}", ypr[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(drawing, fmt::format("roll  {:.2f}", ypr[2]), {40, 120}, {0, 0, 255});

    std::vector<cv::Point2f> corners_2d;
    bool success = detect_chessboard_corners_refine(img, pattern_size, preview_scale, corners_2d);

    cv::drawChessboardCorners(drawing, pattern_size, corners_2d, success);
    cv::resize(drawing, drawing, {}, 0.5, 0.5);
    cv::imshow("Press any to continue", drawing);
    cv::waitKey(0);

    fmt::print("[{}] {}\n", success ? "success" : "failure", img_path);
    if (!success) continue;

    Eigen::Matrix3d R_world2gimbal = R_gimbal2world.transpose();
    cv::Mat t_world2gimbal = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    cv::Mat R_world2gimbal_cv;
    cv::eigen2cv(R_world2gimbal, R_world2gimbal_cv);
    cv::Mat rvec, tvec;
    auto obj_pts = board_corners_3d_vertical(pattern_size, static_cast<float>(square_size_mm));
    cv::solvePnP(
      obj_pts, corners_2d, camera_matrix, distort_coeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE);

    R_world2gimbal_list.emplace_back(R_world2gimbal_cv);
    t_world2gimbal_list.emplace_back(t_world2gimbal);
    rvecs.emplace_back(rvec);
    tvecs.emplace_back(tvec);
  }
}

void print_yaml(
  const std::vector<double> & R_gimbal2imubody_data, const cv::Mat & R_camera2gimbal,
  const cv::Mat & t_camera2gimbal, const Eigen::Vector3d & camera_ypr, double distance,
  const Eigen::Vector3d & board_ypr)
{
  YAML::Emitter result;
  std::vector<double> R_camera2gimbal_data(
    R_camera2gimbal.begin<double>(), R_camera2gimbal.end<double>());
  std::vector<double> t_camera2gimbal_data(
    t_camera2gimbal.begin<double>(), t_camera2gimbal.end<double>());

  result << YAML::BeginMap;
  result << YAML::Key << "R_gimbal2imubody";
  result << YAML::Value << YAML::Flow << R_gimbal2imubody_data;
  result << YAML::Newline;
  result << YAML::Newline;
  result << YAML::Comment(fmt::format(
    "相机同理想情况的偏角: yaw{:.2f} pitch{:.2f} roll{:.2f} degree", camera_ypr[0], camera_ypr[1],
    camera_ypr[2]));
  result << YAML::Newline;
  result << YAML::Comment(fmt::format("标定板到世界坐标系原点的水平距离: {:.2f} m", distance));
  result << YAML::Newline;
  result << YAML::Comment(fmt::format(
    "标定板同竖直摆放时的偏角: yaw{:.2f} pitch{:.2f} roll{:.2f} degree", board_ypr[0], board_ypr[1],
    board_ypr[2]));
  result << YAML::Key << "R_camera2gimbal";
  result << YAML::Value << YAML::Flow << R_camera2gimbal_data;
  result << YAML::Key << "t_camera2gimbal";
  result << YAML::Value << YAML::Flow << t_camera2gimbal_data;
  result << YAML::Newline;
  result << YAML::EndMap;

  fmt::print("\n{}\n", result.c_str());
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

  std::vector<double> R_gimbal2imubody_data;
  std::vector<cv::Mat> R_world2gimbal_list, t_world2gimbal_list;
  std::vector<cv::Mat> rvecs, tvecs;
  load(
    input_folder, config_path, R_gimbal2imubody_data, R_world2gimbal_list, t_world2gimbal_list,
    rvecs, tvecs);

  if (R_world2gimbal_list.empty()) {
    fmt::print("错误: 没有有效的标定数据\n");
    return -1;
  }

  cv::Mat R_gimbal2camera, t_gimbal2camera;
  cv::Mat R_world2board, t_world2board;
  cv::calibrateRobotWorldHandEye(
    rvecs, tvecs, R_world2gimbal_list, t_world2gimbal_list, R_world2board, t_world2board,
    R_gimbal2camera, t_gimbal2camera);
  t_gimbal2camera /= 1e3;
  t_world2board /= 1e3;

  cv::Mat R_camera2gimbal, t_camera2gimbal;
  cv::Mat R_board2world, t_board2world;
  cv::transpose(R_gimbal2camera, R_camera2gimbal);
  cv::transpose(R_world2board, R_board2world);
  t_camera2gimbal = -R_camera2gimbal * t_gimbal2camera;
  t_board2world = -R_board2world * t_world2board;

  Eigen::Matrix3d R_camera2gimbal_eigen;
  cv::cv2eigen(R_camera2gimbal, R_camera2gimbal_eigen);
  Eigen::Matrix3d R_gimbal2ideal{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
  Eigen::Matrix3d R_camera2ideal = R_gimbal2ideal * R_camera2gimbal_eigen;
  Eigen::Vector3d camera_ypr = tools::eulers(R_camera2ideal, 1, 0, 2) * 57.3;

  auto x = t_board2world.at<double>(0);
  auto y = t_board2world.at<double>(1);
  auto distance = std::sqrt(x * x + y * y);

  Eigen::Matrix3d R_board2world_eigen;
  cv::cv2eigen(R_board2world, R_board2world_eigen);
  Eigen::Vector3d board_ypr = tools::eulers(R_board2world_eigen, 2, 1, 0) * 57.3;

  print_yaml(
    R_gimbal2imubody_data, R_camera2gimbal, t_camera2gimbal, camera_ypr, distance, board_ypr);

  return 0;
}
