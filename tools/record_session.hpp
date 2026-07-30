#ifndef TOOLS__RECORD_SESSION_HPP
#define TOOLS__RECORD_SESSION_HPP

#include <fmt/chrono.h>
#include <fmt/core.h>

#include <chrono>
#include <string>

namespace tools
{
// 本次进程的"录制会话时间戳"，格式 %Y-%m-%d_%H-%M-%S。
//
// 为什么需要它：Recorder / SignalRecorder / RawSerialRecorder 三者以前各自调
// system_clock::now() 拼文件名，如果它们的构造跨越了一个整秒，就会得到不同的
// 前缀（例如 ...04.avi 和 ...05_signals.jsonl），而 data_recording_guide.md
// 承诺"三者时间戳同一格式便于对齐"——一旦前缀不同，对齐脚本就找不到同伴文件。
// 这里用 inline 函数的局部 static 保证整个进程只算一次（C++11 起线程安全，且
// inline 函数的 static 在所有 TU 间共享）。
// 另外注意 time_point_cast<seconds>：fmt 10 的 %S 会把亚秒也打出来（system_clock
// 精度是纳秒），于是文件名变成 ...14.532394080_signals.jsonl，不符合
// data_recording_guide.md 承诺的 %Y-%m-%d_%H-%M-%S 格式。fmt 9（车上的 22.04）
// 不会。这里先截到整秒，两个 fmt 版本下输出一致。
inline const std::string & session_stamp()
{
  static const std::string stamp = fmt::format(
    "{:%Y-%m-%d_%H-%M-%S}",
    std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()));
  return stamp;
}

// 所有录制文件的落盘目录（相对路径，故二进制必须从仓库根目录运行）
inline const char * records_dir() { return "records"; }

}  // namespace tools

#endif  // TOOLS__RECORD_SESSION_HPP
