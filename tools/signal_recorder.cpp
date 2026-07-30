#include "signal_recorder.hpp"

#include <fmt/chrono.h>

#include <filesystem>

#include "math_tools.hpp"
#include "tools/logger.hpp"
#include "tools/record_session.hpp"

namespace tools
{
SignalRecorder::SignalRecorder(double low_rate_hz, const std::vector<std::string> & low_keys)
: init_(false),
  stop_thread_(false),
  // 队列满时 ThreadSafeQueue 默认静默丢弃；这里挂个 handler 把丢弃记下来，
  // 否则 jsonl 出现空洞而离线分析仍以为"每帧一行"，会算出错误的频率/抖动。
  queue_(1024, [this] { dropped_.fetch_add(1, std::memory_order_relaxed); })
{
  start_time_ = std::chrono::steady_clock::now();

  text_path_ = fmt::format("{}/{}_signals.jsonl", records_dir(), session_stamp());
  std::filesystem::create_directory(records_dir());

  low_period_ = (low_rate_hz > 0.0) ? (1.0 / low_rate_hz) : 0.0;
  low_keys_.insert(low_keys.begin(), low_keys.end());
}

SignalRecorder::~SignalRecorder()
{
  if (!init_) return;  // 从未 record 过：没有线程也没有文件，直接返回

  // 先让保存线程把队列里剩下的行全部写完再退出，否则崩溃/Ctrl-C 时丢掉的正好
  // 是最后那几十帧——而复盘要看的往往就是这段。用一个空 json 作哨兵唤醒可能
  // 正阻塞在 pop() 上的线程；save_to_file 看到"stop_ 已置位且队列空"才收工。
  stop_thread_ = true;
  queue_.push({0.0, nlohmann::json::object()});
  if (saving_thread_.joinable()) saving_thread_.join();

  text_writer_.close();

  auto lost = dropped_.load();
  if (lost > 0)
    logger()->warn(
      "[SignalRecorder] {} 行因队列满被丢弃，{} 不再是每帧一行。", lost, text_path_);
}

void SignalRecorder::record(
  std::chrono::steady_clock::time_point t, const nlohmann::json & signals)
{
  if (!init_) init();
  auto since_begin = tools::delta_time(t, start_time_);

  // 分频：把这一帧里"还没到点"的低频 key 删掉。必须在主线程做——依赖
  // low_last_written_ 这个顺序状态，扔到后台线程会因为乱序而失效。
  if (!low_keys_.empty() && signals.is_object()) {
    nlohmann::json filtered = signals;
    apply_rate_limit(filtered, since_begin);
    // 不在主线程做 dump，把 json 对象推进队列让保存线程序列化
    queue_.push({since_begin, std::move(filtered)});
    return;
  }

  queue_.push({since_begin, signals});
}

void SignalRecorder::apply_rate_limit(nlohmann::json & signals, double since_begin)
{
  for (const auto & key : low_keys_) {
    auto it = signals.find(key);
    if (it == signals.end()) continue;  // 这一帧本来就没这个信号

    auto last = low_last_written_.find(key);
    if (last != low_last_written_.end()) {
      // low_period_ == 0 表示"只写一次"：写过就永远不再写
      bool due = (low_period_ > 0.0) && (since_begin - last->second >= low_period_);
      if (!due) {
        signals.erase(it);  // 未到点 → 这一行不带这个 key（离线 ffill 还原）
        continue;
      }
    }
    low_last_written_[key] = since_begin;
  }
}

void SignalRecorder::init()
{
  text_writer_.open(text_path_);
  saving_thread_ = std::thread(&SignalRecorder::save_to_file, this);
  init_ = true;
}

void SignalRecorder::save_to_file()
{
  // 注意循环条件：不能写成 while (!stop_thread_)，那样析构时会把队列里已经排好
  // 队的行直接扔掉。这里要求"停止标志已置位 且 队列已空"才退出。
  while (true) {
    FrameData frame;
    queue_.pop(frame);

    // 跳过析构时塞进来的"哨兵"空 json
    if (frame.signals.empty()) {
      if (stop_thread_ && queue_.empty()) break;
      continue;
    }

    // 合并 t 和 signals 写一行 JSON。signals 已经是 object 时直接把 t 塞进去，
    // 否则用 {"t": ..., "value": signals} 兜底（虽然按约定调用方都传 object）。
    nlohmann::json line;
    if (frame.signals.is_object()) {
      line = std::move(frame.signals);
      line["t"] = frame.since_begin;
    } else {
      line["t"] = frame.since_begin;
      line["value"] = std::move(frame.signals);
    }

    // 每行写完立刻 flush，断电不丢数据
    text_writer_ << line.dump() << std::endl;

    if (stop_thread_ && queue_.empty()) break;
  }
}

}  // namespace tools
