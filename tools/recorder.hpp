#ifndef TOOLS__RECORDER_HPP
#define TOOLS__RECORDER_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>

#include "tools/thread_safe_queue.hpp"
namespace tools
{
class Recorder
{
public:
  Recorder(double fps = 30);
  ~Recorder();
  void record(
    const cv::Mat & img, const Eigen::Quaterniond & q,
    const std::chrono::steady_clock::time_point & timestamp);

private:
  struct FrameData
  {
    cv::Mat img;
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };
  bool init_;
  std::atomic<bool> stop_thread_;
  double fps_;
  std::string text_path_;
  std::string video_path_;
  std::ofstream text_writer_;
  cv::VideoWriter video_writer_;
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_time_;
  tools::ThreadSafeQueue<FrameData> queue_;
  std::thread saving_thread_;  // 负责保存帧数据的线程
  void init(const cv::Mat & img);
  void save_to_file();
};

// 串口字节级 raw dump：把每一帧 RX/TX 字节落盘，便于赛后复盘上下位机通讯流。
// 异步落盘，不阻塞调用线程。
class RawSerialRecorder
{
public:
  RawSerialRecorder();
  ~RawSerialRecorder();
  void record_rx(const uint8_t * data, size_t size);
  void record_tx(const uint8_t * data, size_t size);

private:
  enum class Direction : uint8_t { RX, TX };
  struct Packet
  {
    Direction dir;
    uint64_t ts_ns;
    std::vector<uint8_t> bytes;
  };

  std::string rx_path_;
  std::string tx_path_;
  std::ofstream rx_writer_;
  std::ofstream tx_writer_;
  bool rx_init_;
  bool tx_init_;
  std::atomic<bool> stop_thread_;
  tools::ThreadSafeQueue<Packet> queue_;
  std::thread saving_thread_;

  void enqueue(Direction dir, const uint8_t * data, size_t size);
  void save_to_file();
};

}  // namespace tools

#endif  // TOOLS__RECORDER_HPP