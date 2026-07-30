#ifndef TOOLS__RECORD_CONFIG_HPP
#define TOOLS__RECORD_CONFIG_HPP

#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <vector>

#include "tools/logger.hpp"
#include "tools/recorder.hpp"
#include "tools/signal_recorder.hpp"

namespace tools
{
// 三类录制的统一开关解析 + 构造。
//
// 以前每个 main 各自 YAML::LoadFile 再手写一遍
//   bool record_video = !yaml["record_video"] || yaml["record_video"].as<bool>();
// 三份 main 抄三遍，而 standard.cpp 干脆没抄（无条件录像、完全没有 signals）。
// 这里收成一处，新增机器人只需要动 yaml。
//
// yaml 键（全部可缺省，缺省值 = 分频改动之前的行为）：
//
//   record_video:   true    # 视频+四元数 → records/<ts>.avi + .txt（缺省 true）
//   record_signals: false   # 调试量 JSONL → records/<ts>_signals.jsonl（缺省 false）
//   record_serial_raw: false# 串口字节流 → records/<ts>_rx.bin/_tx.bin（缺省 false）
//                           # 注：这一项由 io::Gimbal 自己读，不在本结构里
//   record_video_fps: 30    # 视频抽帧率；tools::Recorder 会丢掉超出的帧
//   signals_low_rate_hz: 5  # 低频档目标频率；<=0 表示低频 key 只写一次
//   signals_low_keys:       # 归入低频档的 key，其余 key 每帧都写
//     - tracker_state
//     - bullet_count
//
// 之所以把 record_video_fps 也暴露出来：它以前硬编码在 Recorder 的默认参数里
// （fps=30），而所有 main 都无参构造，于是 50fps 的相机被静默丢帧到 30fps，
// 导致 signals 行数和视频帧数对不上（pitfalls.md §7 记了这个坑）。现在它至少
// 是可见、可配的。
struct RecordConfig
{
  bool video = true;
  bool signals = false;
  double video_fps = 30.0;
  double signals_low_rate_hz = 0.0;
  std::vector<std::string> signals_low_keys;

  static RecordConfig from_yaml(const YAML::Node & yaml)
  {
    RecordConfig c;
    // 全部用"缺 key 就取默认值"的读法，不能用 tools::read()——那个缺 key 会
    // exit(1)，会让所有没同步改 yaml 的机器人开不了机（pitfalls.md §5）。
    if (yaml["record_video"]) c.video = yaml["record_video"].as<bool>();
    if (yaml["record_signals"]) c.signals = yaml["record_signals"].as<bool>();
    if (yaml["record_video_fps"]) c.video_fps = yaml["record_video_fps"].as<double>();
    if (yaml["signals_low_rate_hz"])
      c.signals_low_rate_hz = yaml["signals_low_rate_hz"].as<double>();
    if (yaml["signals_low_keys"])
      c.signals_low_keys = yaml["signals_low_keys"].as<std::vector<std::string>>();
    return c;
  }

  static RecordConfig from_file(const std::string & config_path)
  {
    return from_yaml(YAML::LoadFile(config_path));
  }
};

// 按配置构造两个录制器（未启用则返回 nullptr），并把结果打进日志——开机时
// 一眼能看出这台车在录什么、分频多少。
struct Recorders
{
  std::unique_ptr<Recorder> video;
  std::unique_ptr<SignalRecorder> signals;
};

inline Recorders make_recorders(const RecordConfig & c)
{
  Recorders r;

  if (c.video) {
    r.video = std::make_unique<Recorder>(c.video_fps);
    logger()->info("[Record] video+quat 已启用，抽帧 {:.0f}fps", c.video_fps);
  }

  if (c.signals) {
    r.signals = std::make_unique<SignalRecorder>(c.signals_low_rate_hz, c.signals_low_keys);
    if (c.signals_low_keys.empty()) {
      logger()->info("[Record] signals 已启用（全部信号全速）：{}", r.signals->path());
    } else if (c.signals_low_rate_hz > 0.0) {
      logger()->info(
        "[Record] signals 已启用：{} 个 key 降频到 {:.1f}Hz，其余全速 → {}",
        c.signals_low_keys.size(), c.signals_low_rate_hz, r.signals->path());
    } else {
      logger()->info(
        "[Record] signals 已启用：{} 个 key 只写一次，其余全速 → {}", c.signals_low_keys.size(),
        r.signals->path());
    }
  }

  return r;
}

inline Recorders make_recorders(const std::string & config_path)
{
  return make_recorders(RecordConfig::from_file(config_path));
}

}  // namespace tools

#endif  // TOOLS__RECORD_CONFIG_HPP
