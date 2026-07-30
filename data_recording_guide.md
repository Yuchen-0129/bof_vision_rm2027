# 数据录制 / 复盘指南

仓库提供三种持久化机制，覆盖从相机原始帧到上下位机字节流再到算法中间量的完整数据链路。所有录制数据落在 `records/` 目录下（被 `.gitignore` 忽略）。

## 一、三类录制对照

| 类 | 内容 | 文件 | 默认 | yaml 开关 |
|---|---|---|---|---|
| `tools::Recorder` | 视频 + 四元数 | `<时间戳>.avi` + `<时间戳>.txt` | 开 | `record_video` |
| `tools::RawSerialRecorder` | 上下位机串口字节流 | `<时间戳>_rx.bin` + `<时间戳>_tx.bin` | 关 | `record_serial_raw` |
| `tools::SignalRecorder` | EKF / cmd / Tracker 等中间量（JSONL） | `<时间戳>_signals.jsonl` | 关 | `record_signals` |

三者共用**同一个进程级时间戳**（`tools::session_stamp()`，`tools/record_session.hpp`），
所以四类文件前缀必然一致，`records/<同一前缀>*` 就能捞出一次会话的全部数据。
以前三者各自调 `system_clock::now()`，构造跨整秒时前缀会差一秒，对齐脚本配不上对。

yaml 缺失 `record_video` 时按"开"处理；其余开关缺失按"关"，向后兼容。

三个 main（`auto_aim_debug_aimer` / `auto_aim_debug_hero_aimer` / `standard`）现在都走
同一套开关解析（`tools::make_recorders(config_path)`，见 `tools/record_config.hpp`）。
`standard` 以前是无条件录像且完全没有 signals，现已统一。
`auto_aim_test`（离线回放）也支持 `record_signals`，用于"改算法 → 回放 → diff 两次 jsonl"。

## 二、Recorder（视频 + 四元数）

每帧把 cv::Mat 和云台四元数推入异步队列，写出 `.avi`（MJPG）和 `.txt`（每行 `<since_begin> <qw> <qx> <qy> <qz>`）。fps 默认 30，构造时可改。

### 启用方法
对应机器人 yaml 里：

```yaml
record_video: true   # 默认开（缺失也按 true 处理）
```

main 端读 yaml 决定是否构造 `tools::Recorder`。关掉这个开关只省视频文件，不影响别的录制。

## 三、RawSerialRecorder（串口字节流）

### 用途
比赛复盘时回答"是不是协议层出错了"。包含全部 RX（CRC 通过的帧）和 TX 字节，可与 vision 端日志对照。

### 文件格式
两个二进制文件，每条记录定长头 + 变长 payload：

```
[uint64_t ts_ns][uint8_t size][uint8_t bytes...]
```

`ts_ns` 是 `steady_clock::now().time_since_epoch()` 的纳秒数（小端），`size` 是 payload 字节数。

### 启用方法
对应机器人 yaml 里：

```yaml
record_serial_raw: true
```

`io::Gimbal` 构造时检查这个开关，开启则构造 `RawSerialRecorder`。生产环境建议关闭（IO 开销虽小，但避免硬盘磨损）。

### 复盘
```bash
./build/recorder_dump records/2026-05-23_17-00-00_rx.bin
./build/recorder_dump records/2026-05-23_17-00-00_tx.bin
```

输出每条已解码字段（mode / yaw / yaw_vel / bullet_speed / 四元数 / CRC 等）。

## 四、SignalRecorder（结构化中间量）

### 用途
PlotJuggler 是实时调试用的，断电就没了。SignalRecorder 把每帧的关键信号合并成一行 JSON 写盘，便于赛后离线分析。和文本 logger 互补：logger 是事件型（"开火了"、"lost 了"），SignalRecorder 是连续时间序列。

### 接口
```cpp
namespace tools {
class SignalRecorder {
public:
  SignalRecorder();
  ~SignalRecorder();
  void record(std::chrono::steady_clock::time_point t,
              const nlohmann::json & signals);
};
}
```

每行格式：

```json
{"t": 1.234, "ekf_x": [...], "ekf_w": 2.51, "cmd_yaw": 0.1, "cmd_pitch": -0.05, "fire": false, "tracker_state": "tracking"}
```

`t` 是相对构造时刻的秒数。JSON 序列化在内部线程完成，主循环只入队不阻塞。每行写完立即 flush，断电不丢数据。

### 启用方法
对应机器人 yaml 里：

```yaml
record_signals: true
```

main 端读 yaml 决定是否构造 `SignalRecorder`，并在每帧调一次 `record(t, j)`。

### 分频：部分信号高频、部分降频

signals 是除视频外体积最大的一项，但不是所有信号都值得每帧存：EKF 状态、`cmd_*`、
`gimbal_*`、`residual_*` 要和视频逐帧对齐、要算抖动 p99，必须全速；而 `tracker_state`、
`bullet_count`、协方差对角线这类慢变量或诊断辅助量，每帧存只是放大文件。

```yaml
signals_low_rate_hz: 5   # 低频档目标频率；设 0 表示低频 key 只写一次
signals_low_keys:        # 列进来的降频，其余全速
  - tracker_state
  - bullet_speed
  - bullet_count
  - P_x
  - P_y
  - P_z
  - P_r
```

**分频是按 key 而不是按行**——这是关键设计。一行 JSON 里既有全速档也有低频档的键，
低频键到点了才带上：

```jsonl
{"w":1.40,"cmd_yaw":-88.6,"tracker_state":"tracking","P_x":0.001,"t":0.00}   ← 低频键在场
{"w":1.41,"cmd_yaw":-88.1,"t":0.02}                                          ← 低频键缺席
{"w":1.39,"cmd_yaw":-88.3,"t":0.04}
...
{"w":1.53,"cmd_yaw":-88.2,"tracker_state":"tracking","P_x":0.001,"t":0.22}   ← 到点，再写一次
```

这样时间轴仍是每帧一行，逐帧对齐视频 / `_tx.bin` 的逻辑完全不变。
如果按行分频（整行丢掉），全速档信号会跟着一起没，起不到降频作用。

**离线读的时候必须先 ffill**，否则低频列大部分是 NaN：

```python
import pandas as pd
df = pd.read_json("records/<ts>_signals.jsonl", lines=True).ffill()
```

实测（`assets/demo/demo` 前 400 帧，13 个诊断辅助量降到 5Hz）：
**体积降到 66%（省 34%），行数不变（400 行），全速档 key 逐值零变化。**

不配 `signals_low_keys` = 全部信号全速 = 与分频功能加入之前完全一致。

> ⚠️ 回放**不是逐位确定**的，diff 两次 jsonl 之前先确认检测序列同组，
> 否则会把识别抖动误判成自己改动的锅。见 `pitfalls.md` 第六又二分之一章。

### 推荐 hook 位置
每帧把 Tracker / Aimer / Gimbal 三个阶段的关键信号合并成同一个 json：

```cpp
nlohmann::json j;
// Tracker.track 后
j["state"] = tracker.state();
auto x = target.ekf_x();
j["ekf_x"] = {x[0], x[2], x[4], x[6], x[7], x[8]};  // x, y, z, a, w, r

// Aimer.aim 后
j["cmd_yaw"] = cmd.yaw;
j["cmd_pitch"] = cmd.pitch;
j["aim_valid"] = aimer.debug_aim_point.valid;

// Gimbal 状态
auto gs = gimbal.state();
j["gimbal_yaw"] = gs.yaw;
j["gimbal_pitch"] = gs.pitch;
j["fire"] = cmd.shoot;

signal_recorder.record(timestamp, j);
```

### 复盘
```bash
./build/recorder_dump records/2026-05-23_17-00-00_signals.jsonl
```

直接逐行打印 JSON。如果只想看某几列，用 `jq` 简单过滤：

```bash
./build/recorder_dump records/...signals.jsonl | jq -c '{t, cmd_yaw, gimbal_yaw, fire}'
```

## 五、`recorder_dump` 工具

统一解析三种录制文件，按扩展名自动选格式：

```bash
./build/recorder_dump <file>
  file ending in _rx.bin / _tx.bin / _signals.jsonl
```

实现见 `tools/recorder_dump.cpp`，复用 `io::GimbalToVision / VisionToGimbal` 协议结构体直接 memcpy 解码，无需重复定义字段。

## 六、典型工作流

### 比赛后复盘"为什么打偏"
1. 找到事故时刻，看 records/ 目录里对应时间戳的所有文件。
2. `./build/recorder_dump <ts>_signals.jsonl` 看 EKF 是否发散、cmd 是否合理。
3. 如果 cmd 不对，看 `<ts>_tx.bin` 里 vision 实际发出的字节。
4. 如果云台没响应，看 `<ts>_rx.bin` 里下位机回传的 yaw / pitch。
5. 视频 `.avi` 提供视觉真相做交叉验证。

### 比赛前调参
1. yaml 里只开 `record_signals: true`（数据量小、覆盖关键算法量）。
2. 跑一段时间后用 `recorder_dump | jq` 简单过滤想看的列，配合 awk/sort 找异常帧。
3. 实测 hero `gimbal_yaw_tau` 时同时启用 `record_signals` 和 `record_serial_raw`，能从字节级回溯响应曲线（cmd 时间戳 vs gimbal 实测 yaw 时间戳）。

## 七、磁盘占用估算（100Hz 采样、1 小时）

| 数据 | 体积 |
|---|---|
| 视频 1280×1024 MJPG 30fps | 10-30 GB |
| 四元数 .txt | 18 MB |
| RX raw (~30B/帧) | 11 MB |
| TX raw (~30B/帧) | 11 MB |
| Signals JSONL（全速） | 100 MB（信号多则更大） |
| Signals JSONL（分频后） | 约 66%，取决于降频了哪些 key |

视频是绝对大头，其他全部加起来一小时不到 200MB。比赛前清理 `records/` 即可。
分频省下的绝对体积不大，真正的价值是**长时间录制时 signals 不再线性膨胀**，
以及把注意力集中在真正需要逐帧的那几十个信号上。
