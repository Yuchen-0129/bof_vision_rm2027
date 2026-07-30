#ifndef TOOLS__SIGNAL_RECORDER_HPP
#define TOOLS__SIGNAL_RECORDER_HPP

#include <atomic>
#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "tools/thread_safe_queue.hpp"

namespace tools
{
// JSONL 格式的"时间序列中间量"记录器：每帧一行 JSON，写入
// records/<timestamp>_signals.jsonl，便于离线用 pandas.read_json(lines=True)
// 复盘 EKF 状态、cmd、Tracker 状态等信号。
//
// 使用模式与 tools::Recorder 一致：构造时只生成路径并建好 records/ 目录，
// 文件在第一次 record() 调用时才打开，并启动后台保存线程。JSON 序列化在保
// 存线程里做，避免主循环卡顿。
//
// ---------------------------------------------------------------------------
// 分频（rate tiering）
// ---------------------------------------------------------------------------
// 不是所有信号都需要每帧存。EKF 状态、cmd、云台反馈这些"要和视频逐帧对齐、
// 要算抖动 p99"的必须全速；而标定外参、bullet_count、tracker_state 这类慢变量
// 每帧存只是放大文件体积（signals 是除视频外最大的一项，见
// data_recording_guide.md §7）。
//
// 所以每个 key 归入一个"档位"（Tier）：
//   - Tier::high  每帧都写（默认档，未显式配置的 key 都进这里）
//   - Tier::low   按 low_rate_hz 限流，两次写入间隔不足时该 key 从这一行里删掉
//
// 关键设计：**分频是按 key 而不是按行**。一行 JSON 里既有 high 也有 low 的 key，
// 到点了才把 low 的 key 带上，没到点就只写 high 的。这样：
//   - 时间轴仍然是每帧一行，逐帧对齐视频/`_tx.bin` 的逻辑完全不变
//   - low 档的 key 在多数行里"缺席"，离线读的时候是 NaN，用 ffill 就能还原
//     （pandas: df[["R_camera2gimbal"]].ffill()）
// 反过来如果按行分频（整行丢掉），high 档信号就跟着一起没了，起不到降频作用。
//
// 未在 yaml 里配置分频时，所有 key 都是 high 档，行为与分频前完全一致。
class SignalRecorder
{
public:
  enum class Tier
  {
    high,  // 每帧写
    low    // 按 low_rate_hz 限流
  };

  // low_rate_hz  低频档的目标频率（Hz）；<= 0 表示"只写一次"（适合纯静态量，
  //              例如标定外参——存一次就够，后面全靠 ffill）
  // low_keys     归入低频档的 key 列表；其余 key 一律 high 档
  explicit SignalRecorder(
    double low_rate_hz = 0.0, const std::vector<std::string> & low_keys = {});

  ~SignalRecorder();

  // 把一帧的多个信号合并成一行 JSON 写入磁盘。
  // - t       帧时间戳（steady_clock）；写入文件时换算成相对构造时刻的秒数
  // - signals 该帧的所有信号，例如 {"ekf_x": 1.2, "cmd_yaw": 0.3, ...}
  //
  // 注意：分频判定（哪些 low 档 key 这一帧该写）在调用线程里做，因为它依赖
  // "上次写这个 key 是什么时候"这个顺序状态；真正的 JSON 序列化仍在后台线程。
  // 判定只是每个 low key 一次哈希查找 + 一次浮点比较，开销可忽略。
  void record(std::chrono::steady_clock::time_point t, const nlohmann::json & signals);

  // 本次会话的 jsonl 路径（供 main 打日志用）
  const std::string & path() const { return text_path_; }

  // 因队列满而被丢弃的行数。非 0 说明磁盘跟不上采样速度，这时 jsonl 有空洞，
  // 不能再假设"每帧一行"——析构时会 warn 一次。
  uint64_t dropped() const { return dropped_.load(); }

private:
  struct FrameData
  {
    double since_begin;      // 相对 start_time_ 的秒数
    nlohmann::json signals;  // 该帧的信号字典（low 档未到点的 key 已被剔除）
  };

  bool init_;
  std::atomic<bool> stop_thread_;
  std::string text_path_;
  std::ofstream text_writer_;
  std::chrono::steady_clock::time_point start_time_;
  tools::ThreadSafeQueue<FrameData> queue_;
  std::thread saving_thread_;
  std::atomic<uint64_t> dropped_{0};

  // 分频配置与状态
  double low_period_;  // 低频档最小写入间隔（s）；0 表示只写一次
  std::unordered_set<std::string> low_keys_;
  std::unordered_map<std::string, double> low_last_written_;  // key -> 上次写入的 since_begin

  void init();
  void save_to_file();

  // 就地剔除该帧中"还没到写入时刻"的低频 key
  void apply_rate_limit(nlohmann::json & signals, double since_begin);
};

}  // namespace tools

#endif  // TOOLS__SIGNAL_RECORDER_HPP
