#include "recorder.hpp"

#include <fmt/chrono.h>

#include <filesystem>
#include <string>

#include "math_tools.hpp"
#include "tools/logger.hpp"
#include "tools/record_session.hpp"

namespace tools
{
Recorder::Recorder(double fps) : init_(false), fps_(fps), queue_(1), stop_thread_(false)
{
  start_time_ = std::chrono::steady_clock::now();
  last_time_ = start_time_;

  // 用进程级共享的时间戳，保证 .avi/.txt/_signals.jsonl/_rx.bin 前缀一致——
  // 各自 now() 会在跨整秒构造时错开一秒，对齐脚本就配不上对了
  text_path_ = fmt::format("{}/{}.txt", records_dir(), session_stamp());
  video_path_ = fmt::format("{}/{}.avi", records_dir(), session_stamp());

  std::filesystem::create_directory(records_dir());
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
  // 同 SignalRecorder / RawSerialRecorder：要求"停止标志已置位 且 队列已空"才退出，
  // 否则析构时会丢掉已入队的尾帧，视频比 .txt / signals 短一截
  while (true) {
    FrameData frame;
    queue_.pop(frame);  // 从队列中取出帧数据
    if (frame.img.empty()) {
      if (stop_thread_ && queue_.empty()) break;
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

    if (stop_thread_ && queue_.empty()) break;
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
  std::filesystem::create_directory(records_dir());

  rx_path_ = fmt::format("{}/{}_rx.bin", records_dir(), session_stamp());
  tx_path_ = fmt::format("{}/{}_tx.bin", records_dir(), session_stamp());

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
  // 同 SignalRecorder：要求"停止标志已置位 且 队列已空"才退出，否则析构时会丢掉
  // 已入队的尾包——而复盘最需要的往往就是崩溃前最后那几帧
  while (true) {
    Packet pkt;
    queue_.pop(pkt);
    if (pkt.bytes.empty()) {
      if (stop_thread_ && queue_.empty()) break;
      continue;
    }

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

    if (writer == nullptr || !writer->is_open()) {
      if (stop_thread_ && queue_.empty()) break;
      continue;
    }

    uint8_t size_byte = static_cast<uint8_t>(std::min<size_t>(pkt.bytes.size(), 255));
    writer->write(reinterpret_cast<const char *>(&pkt.ts_ns), sizeof(pkt.ts_ns));
    writer->write(reinterpret_cast<const char *>(&size_byte), sizeof(size_byte));
    writer->write(reinterpret_cast<const char *>(pkt.bytes.data()), size_byte);

    if (stop_thread_ && queue_.empty()) break;
  }
}

}  // namespace tools
