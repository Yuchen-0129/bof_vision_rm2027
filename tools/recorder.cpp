#include "recorder.hpp"

#include <fmt/chrono.h>

#include <filesystem>
#include <string>

#include "math_tools.hpp"
#include "tools/logger.hpp"

namespace tools
{
Recorder::Recorder(double fps) : init_(false), fps_(fps), queue_(1), stop_thread_(false)
{
  start_time_ = std::chrono::steady_clock::now();
  last_time_ = start_time_;

  auto folder_path = "records";
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  text_path_ = fmt::format("{}/{}.txt", folder_path, file_name);
  video_path_ = fmt::format("{}/{}.avi", folder_path, file_name);

  std::filesystem::create_directory(folder_path);
}

Recorder::~Recorder()
{
  stop_thread_ = true;
  // 退出时给队列中额外推入一个空帧，避免pop一直等待
  queue_.push({cv::Mat::zeros(0, 0, 0), {0, 0, 0, 0}, std::chrono::steady_clock::now()});
  if (saving_thread_.joinable()) saving_thread_.join();  // 等待视频保存线程结束

  if (!init_) return;
  text_writer_.close();
  video_writer_.release();
}

void Recorder::save_to_file()
{
  while (!stop_thread_) {
    FrameData frame;
    queue_.pop(frame);  // 从队列中取出帧数据
    if (frame.img.empty()) {
      tools::logger()->debug("Recorder received empty img. Skip this frame.");
      continue;
    }
    // 写入视频文件
    video_writer_.write(frame.img);

    // 写入文本文件（输出顺序为wxyz）
    Eigen::Vector4d xyzw = frame.q.coeffs();
    auto since_begin = tools::delta_time(frame.timestamp, start_time_);
    text_writer_ << fmt::format(
      "{} {} {} {} {}\n", since_begin, xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  }
}

void Recorder::record(
  const cv::Mat & img, const Eigen::Quaterniond & q,
  const std::chrono::steady_clock::time_point & timestamp)
{
  if (img.empty()) return;
  if (!init_) init(img);

  auto since_last = tools::delta_time(timestamp, last_time_);
  if (since_last < 1.0 / fps_) return;

  last_time_ = timestamp;
  queue_.push({img, q, timestamp});
}

void Recorder::init(const cv::Mat & img)
{
  text_writer_.open(text_path_);
  auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  video_writer_ = cv::VideoWriter(video_path_, fourcc, fps_, img.size());
  saving_thread_ = std::thread(&Recorder::save_to_file, this);  // 启动保存线程
  init_ = true;
}

// ===================== RawSerialRecorder =====================

RawSerialRecorder::RawSerialRecorder()
: rx_init_(false), tx_init_(false), stop_thread_(false), queue_(4096)
{
  auto folder_path = "records";
  std::filesystem::create_directory(folder_path);

  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  rx_path_ = fmt::format("{}/{}_rx.bin", folder_path, file_name);
  tx_path_ = fmt::format("{}/{}_tx.bin", folder_path, file_name);

  saving_thread_ = std::thread(&RawSerialRecorder::save_to_file, this);
}

RawSerialRecorder::~RawSerialRecorder()
{
  stop_thread_ = true;
  // 推一个空包唤醒消费线程
  queue_.push({Direction::RX, 0, {}});
  if (saving_thread_.joinable()) saving_thread_.join();

  if (rx_init_ && rx_writer_.is_open()) rx_writer_.close();
  if (tx_init_ && tx_writer_.is_open()) tx_writer_.close();
}

void RawSerialRecorder::enqueue(Direction dir, const uint8_t * data, size_t size)
{
  if (data == nullptr || size == 0) return;
  Packet pkt;
  pkt.dir = dir;
  auto now = std::chrono::steady_clock::now();
  pkt.ts_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  pkt.bytes.assign(data, data + size);
  queue_.push(std::move(pkt));
}

void RawSerialRecorder::record_rx(const uint8_t * data, size_t size)
{
  enqueue(Direction::RX, data, size);
}

void RawSerialRecorder::record_tx(const uint8_t * data, size_t size)
{
  enqueue(Direction::TX, data, size);
}

void RawSerialRecorder::save_to_file()
{
  while (!stop_thread_) {
    Packet pkt;
    queue_.pop(pkt);
    if (pkt.bytes.empty()) continue;

    std::ofstream * writer = nullptr;
    if (pkt.dir == Direction::RX) {
      if (!rx_init_) {
        rx_writer_.open(rx_path_, std::ios::binary | std::ios::out);
        rx_init_ = true;
      }
      writer = &rx_writer_;
    } else {
      if (!tx_init_) {
        tx_writer_.open(tx_path_, std::ios::binary | std::ios::out);
        tx_init_ = true;
      }
      writer = &tx_writer_;
    }

    if (writer == nullptr || !writer->is_open()) continue;

    uint8_t size_byte = static_cast<uint8_t>(std::min<size_t>(pkt.bytes.size(), 255));
    writer->write(reinterpret_cast<const char *>(&pkt.ts_ns), sizeof(pkt.ts_ns));
    writer->write(reinterpret_cast<const char *>(&size_byte), sizeof(size_byte));
    writer->write(reinterpret_cast<const char *>(pkt.bytes.data()), size_byte);
  }
}

}  // namespace tools
