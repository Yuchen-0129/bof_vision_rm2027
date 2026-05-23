#include "gimbal.hpp"

#include <iostream>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  // 串口字节级 raw dump 开关，默认关闭
  if (yaml["record_serial_raw"]) {
    record_serial_raw_ = yaml["record_serial_raw"].as<bool>();
  }
  if (record_serial_raw_) {
    raw_recorder_ = std::make_unique<tools::RawSerialRecorder>();
    tools::logger()->info("[Gimbal] Raw serial recording enabled.");
  }

  try {
    serial_.setPort(com_port);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100); // 100毫秒
    serial_.setTimeout(timeout);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  tx_data_.head = 0x50;
  tx_data_.mode = VisionToGimbal.mode;
  tx_data_.yaw = VisionToGimbal.yaw;
  tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
  tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
  tx_data_.pitch = VisionToGimbal.pitch;
  tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
  tx_data_.pitch_acc = VisionToGimbal.pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }

  if (raw_recorder_) {
    raw_recorder_->record_tx(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  tx_data_.head = 0x50;
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }

  if (raw_recorder_) {
    raw_recorder_->record_tx(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (rx_data_.head != 0x53) continue;

    auto t = std::chrono::steady_clock::now();

    if (!read(
          reinterpret_cast<uint8_t *>(&rx_data_.mode),
          sizeof(rx_data_) - sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      tools::logger()->debug("[Gimbal] CRC16 check failed.");
      continue;
    }

    error_count = 0;

    if (raw_recorder_) {
      raw_recorder_->record_rx(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_));
    }

    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
    queue_.push({q, t});

    std::lock_guard<std::mutex> lock(mutex_);

    state_.yaw = rx_data_.yaw;
    state_.yaw_vel = rx_data_.yaw_vel;
    state_.pitch = rx_data_.pitch;
    state_.pitch_vel = rx_data_.pitch_vel;
    state_.bullet_speed = rx_data_.bullet_speed;
    state_.bullet_count = rx_data_.bullet_count;
    state_.enemy_color = rx_data_.enemy_color;
    //std::cout << "[Gimbal] enemy_color: " << static_cast<int>(state_.enemy_color) << std::endl;

    switch (rx_data_.mode) {
      case 0:
        mode_ = GimbalMode::IDLE;
        break;
      case 1:
        mode_ = GimbalMode::AUTO_AIM;
        break;
      case 2:
        mode_ = GimbalMode::SMALL_BUFF;
        break;
      case 3:
        mode_ = GimbalMode::BIG_BUFF;
        break;
      default:
        mode_ = GimbalMode::IDLE;
        tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
        break;
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}


// void Gimbal::read_thread()
// {
//   tools::logger()->info("[Gimbal] read_thread started.");

//   int error_count = 0;
//   int fail_head_read = 0;
//   int fail_head_wrong = 0;
//   int fail_body_read = 0;
//   int fail_crc = 0;

//   auto last_report = std::chrono::steady_clock::now();

//   while (!quit_)
//   {
//     // 每 3 秒报告一次当前错误统计（避免刷屏但能看到趋势）
//     auto now = std::chrono::steady_clock::now();
//     if (now - last_report >= std::chrono::seconds(3))
//     {
//       tools::logger()->info("[Gimbal] Error stats: total_err={}, head_read_fail={}, head_wrong={}, body_read_fail={}, crc_fail={}",
//                             error_count, fail_head_read, fail_head_wrong, fail_body_read, fail_crc);
//       last_report = now;
//     }

//     // 触发重连条件
//     if (error_count > 5000)
//     {
//       tools::logger()->warn("[Gimbal] Too many errors (total={}), reconnecting. Breakdown: head_read_fail={}, head_wrong={}, body_read_fail={}, crc_fail={}",
//                             error_count, fail_head_read, fail_head_wrong, fail_body_read, fail_crc);
//       error_count = 0;
//       fail_head_read = fail_head_wrong = fail_body_read = fail_crc = 0;
//       reconnect();
//       continue;
//     }

//     // ----- 读帧头 -----
//     if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head)))
//     {
//       error_count++;
//       fail_head_read++;
//       continue;
//     }

//     // ----- 检查帧头 -----
//     if (rx_data_.head != 0x53)
//     {
//       fail_head_wrong++;
//       // 注意：这里不增加 error_count，所以不会直接导致重连
//       continue;
//     }

//     auto t = std::chrono::steady_clock::now();

//     // ----- 读剩余部分 -----
//     if (!read(reinterpret_cast<uint8_t *>(&rx_data_.mode),
//               sizeof(rx_data_) - sizeof(rx_data_.head)))
//     {
//       error_count++;
//       fail_body_read++;
//       continue;
//     }

//     // ----- CRC 校验 -----
//     if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_)))
//     {
//       fail_crc++;
//       // CRC 错误通常不增加 error_count，但如果你希望它也触发重连，可取消下面注释
//       // error_count++;
//       continue;
//     }

//     // ----- 成功收到有效帧 -----
//     error_count = 0;
//     fail_head_read = fail_head_wrong = fail_body_read = fail_crc = 0;

//     // 处理四元数
//     Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
//     queue_.push({q, t});

//     std::lock_guard<std::mutex> lock(mutex_);
//     state_.yaw = rx_data_.yaw;
//     state_.yaw_vel = rx_data_.yaw_vel;
//     state_.pitch = rx_data_.pitch;
//     state_.pitch_vel = rx_data_.pitch_vel;
//     state_.bullet_speed = rx_data_.bullet_speed;
//     state_.bullet_count = rx_data_.bullet_count;
//     state_.enemy_color = rx_data_.enemy_color;

//     switch (rx_data_.mode)
//     {
//       case 0: mode_ = GimbalMode::IDLE; break;
//       case 1: mode_ = GimbalMode::AUTO_AIM; break;
//       case 2: mode_ = GimbalMode::SMALL_BUFF; break;
//       case 3: mode_ = GimbalMode::BIG_BUFF; break;
//       default: mode_ = GimbalMode::IDLE;
//                tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
//                break;
//     }
//   }

//   tools::logger()->info("[Gimbal] read_thread stopped.");
// }
void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io