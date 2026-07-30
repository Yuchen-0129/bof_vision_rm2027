#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"  // 替换 cboard.hpp
#include "tasks/auto_aim/auto_aim_fire/aimer.hpp"
#include "tasks/auto_aim/auto_aim_fire/outpost_aimer.hpp"
#include "tasks/auto_aim/auto_aim_track/outpost_target.hpp"
#include "tasks/auto_aim/auto_aim_fire/shooter.hpp"
#include "tasks/auto_aim/auto_aim_track/solver.hpp"
#include "tasks/auto_aim/auto_aim_track/tracker.hpp"
#include "tasks/auto_aim/auto_aim_detect/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/record_config.hpp"
#include "tools/recorder.hpp"
#include "tools/signal_recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

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
  // 录制器：三类录制的开关都在 yaml 里（record_video / record_signals /
  // record_serial_raw，后者由 io::Gimbal 自己读）。缺省值 = 原行为。
  auto rec = tools::make_recorders(config_path);
  auto & recorder = rec.video;
  auto & signal_recorder = rec.signals;

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO detector(config_path, true);//识别器
  auto_aim::Solver solver(config_path);//坐标结算
  auto_aim::Tracker tracker(config_path, solver);//跟踪器
  auto_aim::Aimer aimer(config_path);//瞄准器    作用：弹道结算 生成瞄准命令
  auto_aim::Shooter shooter(config_path);

  // 前哨站独立链路：自己的 tracker + aimer，与整车那条完全平行。
  // 前哨站转速/半径由规则写死、只有方向随机，跟整车的未知运动学是两回事，
  // 所以分开走，互不影响（整车的 Tracker/Target/Aimer 一行未改）。
  auto_aim::OutpostTracker outpost_tracker(config_path);
  auto_aim::OutpostAimer outpost_aimer(config_path);// 发射器   作用：开火决策

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

    if (recorder) recorder->record(img, q, t);

    // 设定 gimbal 到 world 的旋转矩阵
    solver.set_R_gimbal2world(q);

    // 从串口获取欧拉角
    auto gimbal_state = gimbal.state();
    // Eigen::Vector3d ypr = {gimbal_state.yaw, gimbal_state.pitch, 0};  // 直接从串口获取

    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    //识别装甲板
    auto armors = detector.detect(img);

    // 前哨站先走独立链路（只消费前哨站装甲板，armors 只读）
    auto outpost_targets = outpost_tracker.track(armors, t, solver);

    //跟踪目标
    auto targets = tracker.track(armors, t, gimbal_state.enemy_color);

    // 使用串口获取的子弹速度
    // 获取击打逻辑
    auto command = aimer.aim(targets, t, gimbal_state.bullet_speed);  // 替换 cboard.bullet_speed

    // 决定是否射击
    command.shoot = shooter.shoot(command, aimer, targets, ypr);

    // 前哨站有解时优先用它的 cmd 覆盖：前哨站是固定建筑、转速已知，
    // 它的解比整车模型可靠得多。没解时（未捕获/方向未锁/无板朝向我方）
    // 完全不动整车的 command，整车行为保持原样。
    if (!outpost_targets.empty()) {
      auto oc = outpost_aimer.aim(outpost_targets, gimbal_state.bullet_speed);
      if (oc.control) command = oc;
    }

    // 使用串口发送命令
    gimbal.send(
      command.control, command.shoot, 
      command.yaw, 0, 0,  // yaw, yaw_vel, yaw_acc
      command.pitch, 0, 0  // pitch, pitch_vel, pitch_acc
    );

    /// debug
    tools::draw_text(img, fmt::format("[{}]", tracker.state()), {10, 30}, {255, 255, 255});

    nlohmann::json data;
    // 键名不能叫 "t"：SignalRecorder 会用帧时间戳覆盖 "t"，这个耗时就永远进不了
    // jsonl。同时换成 ms，与日志里 yolo/tracker 耗时同量纲。
    data["loop_ms"] = tools::delta_time(std::chrono::steady_clock::now(), t0) * 1e3;

    // 装甲板原始观测数据
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      auto min_x = 1e10;
      auto & armor = armors.front();
      for (auto & a : armors) {
        if (a.center.x < min_x) {
          min_x = a.center.x;
          armor = a;
        }
      }  //always left
      solver.solve(armor);
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
    }

    if (!targets.empty()) {
      auto target = targets.front();

      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (int i = 0; i < static_cast<int>(armor_xyza_list.size()); ++i) {
        const auto & xyza = armor_xyza_list[i];
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
        if (image_points.empty()) continue;
        float rx = image_points[0].x, ty = image_points[0].y;
        for (const auto & p : image_points) {
          if (p.x > rx) rx = p.x;
          if (p.y < ty) ty = p.y;
        }
        tools::draw_text(
          img, fmt::format("{}", i), {static_cast<int>(rx) - 8, static_cast<int>(ty) + 18},
          {0, 255, 255}, 1.2, 2);
      }

      // aimer瞄准位置
      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid)
        tools::draw_points(img, image_points, {0, 0, 255});
      else
        tools::draw_points(img, image_points, {255, 0, 0});

      // 观测器内部数据
      Eigen::VectorXd x = target.ekf_x();
      data["x"] = x[0];
      data["vx"] = x[1];
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4];
      data["vz"] = x[5];
      data["a"] = x[6] * 57.3;
      data["w"] = x[7];
      data["r"] = x[8];
      data["l"] = x[9];
      data["h"] = x[10];
      data["last_id"] = target.last_id;

      // 卡方检验数据
      data["residual_yaw"] = target.ekf().data.at("residual_yaw");
      data["residual_pitch"] = target.ekf().data.at("residual_pitch");
      data["residual_distance"] = target.ekf().data.at("residual_distance");
      data["residual_angle"] = target.ekf().data.at("residual_angle");
      data["nis"] = target.ekf().data.at("nis");
      data["nees"] = target.ekf().data.at("nees");
      data["nis_fail"] = target.ekf().data.at("nis_fail");
      data["nees_fail"] = target.ekf().data.at("nees_fail");
      data["recent_nis_failures"]= target.ekf().data.at("recent_nis_failures");
    }

    // 云台响应情况（从串口获取）
    data["gimbal_yaw"] = gimbal_state.yaw*57.3;
    data["gimbal_pitch"] = gimbal_state.pitch*57.3;
    data["gimbal_yaw_vel"] = gimbal_state.yaw_vel;
    data["gimbal_pitch_vel"] = gimbal_state.pitch_vel;
    data["bullet_speed"] = gimbal_state.bullet_speed;
    data["bullet_count"] = gimbal_state.bullet_count;



    data["cmd_yaw"] = command.yaw*57.3;
    data["cmdl_pitch"] = command.pitch*57.3;


    plotter.plot(data);
    if (signal_recorder) signal_recorder->record(t, data);

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;


  }

  // 程序退出时发送停止命令
  //gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}