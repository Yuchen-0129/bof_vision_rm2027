#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"  // 替换 cboard.hpp
#include "tasks/auto_aim/auto_aim_fire/aimer.hpp"
#include "tasks/auto_aim/auto_aim_fire/shooter.hpp"
#include "tasks/auto_aim/auto_aim_track/solver.hpp"
#include "tasks/auto_aim/auto_aim_track/tracker.hpp"
#include "tasks/auto_aim/auto_aim_detect/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/tuishao.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;// 退出器
  tools::Plotter plotter;// 绘图器
  tools::Recorder recorder;// 录制器

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO detector(config_path, false);//识别器
  auto_aim::Solver solver(config_path);//坐标结算
  auto_aim::Tracker tracker(config_path, solver);//跟踪器
  auto_aim::Aimer aimer(config_path);//瞄准器    作用：弹道结算 生成瞄准命令
  auto_aim::Shooter shooter(config_path);// 发射器   作用：开火决策

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  // 修改模式类型为 GimbalMode
  auto mode = io::GimbalMode::IDLE;  // 替换 io::Mode::idle
  auto last_mode = io::GimbalMode::IDLE;  // 替换 io::Mode::idle

  while (!exiter.exit()) {
    auto t0 = std::chrono::steady_clock::now();

    // 读取图像
    camera.read(img, t);
    
    // 从串口获取四元数
    q = gimbal.q(t - 2ms);  // 替换 cboard.imu_at(t - 1ms);
    
    // 从串口获取模式
    mode = gimbal.mode();  // 替换 mode = cboard.mode;

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", gimbal.str(mode));  // 替换 io::MODES[mode]
      last_mode = mode;
    }

    recorder.record(img, q, t);

    // 设定 gimbal 到 world 的旋转矩阵
    solver.set_R_gimbal2world(q);

    // 从串口获取欧拉角
    auto gimbal_state = gimbal.state();
    // Eigen::Vector3d ypr = {gimbal_state.yaw, gimbal_state.pitch, 0};  // 直接从串口获取

    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    //识别装甲板
    auto armors = detector.detect(img);

    //跟踪目标
    auto targets = tracker.track(armors, t, gimbal_state.enemy_color);

    // 使用串口获取的子弹速度
    // 获取击打逻辑
    auto command = aimer.aim(targets, t, gimbal_state.bullet_speed);  // 替换 cboard.bullet_speed

    // 决定是否射击
    command.shoot = shooter.shoot(command, aimer, targets, ypr);

    // 使用串口发送命令
    gimbal.send(
      command.control, command.shoot, 
      command.yaw, 0, 0,  // yaw, yaw_vel, yaw_acc
      command.pitch, 0, 0  // pitch, pitch_vel, pitch_acc
    );
}
  

  return 0;
}
