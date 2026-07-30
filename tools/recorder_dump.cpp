// 离线复盘工具：把 records/ 下的录制文件 dump 成人可读文本。
// 支持三种文件：
//   - <prefix>_rx.bin / <prefix>_tx.bin   串口字节流（每条 [u64 ts_ns][u8 size][bytes]）
//   - <prefix>_signals.jsonl              关键信号 JSONL
// 用法：
//   ./build/recorder_dump records/2026-05-23_17-00-00_rx.bin
//   ./build/recorder_dump records/2026-05-23_17-00-00_tx.bin
//   ./build/recorder_dump records/2026-05-23_17-00-00_signals.jsonl
// 输出到 stdout，重定向自取。bin 文件按 vision↔gimbal 协议结构解码（GimbalToVision /
// VisionToGimbal），其他长度直接 hex dump。

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "io/gimbal/gimbal.hpp"

namespace
{
std::string ext_of(const std::string & path)
{
  auto dot = path.find_last_of('.');
  if (dot == std::string::npos) return "";
  return path.substr(dot);
}

bool ends_with(const std::string & s, const std::string & suf)
{
  return s.size() >= suf.size() && s.compare(s.size() - suf.size(), suf.size(), suf) == 0;
}

// 把字节流按已知 packed struct 解码并打印
void print_rx_struct(uint64_t ts_ns, const std::vector<uint8_t> & bytes)
{
  std::cout << ts_ns << " RX ";
  if (bytes.size() != sizeof(io::GimbalToVision)) {
    std::cout << "size=" << bytes.size() << " (expect " << sizeof(io::GimbalToVision) << ")\n";
    return;
  }
  io::GimbalToVision rx;
  std::memcpy(&rx, bytes.data(), sizeof(rx));
  std::cout << "head=0x" << std::hex << static_cast<int>(rx.head) << std::dec
            << " mode=" << static_cast<int>(rx.mode)
            << " color=" << static_cast<int>(rx.enemy_color) << " q=[" << rx.q[0] << "," << rx.q[1]
            << "," << rx.q[2] << "," << rx.q[3] << "]"
            << " yaw=" << rx.yaw << " yaw_vel=" << rx.yaw_vel << " pitch=" << rx.pitch
            << " pitch_vel=" << rx.pitch_vel << " bullet_speed=" << rx.bullet_speed
            << " bullet_count=" << rx.bullet_count << " crc=0x" << std::hex << rx.crc16 << std::dec
            << "\n";
}

void print_tx_struct(uint64_t ts_ns, const std::vector<uint8_t> & bytes)
{
  std::cout << ts_ns << " TX ";
  if (bytes.size() != sizeof(io::VisionToGimbal)) {
    std::cout << "size=" << bytes.size() << " (expect " << sizeof(io::VisionToGimbal) << ")\n";
    return;
  }
  io::VisionToGimbal tx;
  std::memcpy(&tx, bytes.data(), sizeof(tx));
  std::cout << "head=0x" << std::hex << static_cast<int>(tx.head) << std::dec
            << " mode=" << static_cast<int>(tx.mode) << " yaw=" << tx.yaw
            << " yaw_vel=" << tx.yaw_vel << " yaw_acc=" << tx.yaw_acc << " pitch=" << tx.pitch
            << " pitch_vel=" << tx.pitch_vel << " pitch_acc=" << tx.pitch_acc << " crc=0x"
            << std::hex << tx.crc16 << std::dec << "\n";
}

int dump_bin(const std::string & path, bool is_rx)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    std::cerr << "open failed: " << path << "\n";
    return 1;
  }
  std::cout << "# file: " << path << "\n";
  std::cout << "# format: <ts_ns> " << (is_rx ? "RX" : "TX") << " <fields...>\n";
  while (true) {
    uint64_t ts_ns = 0;
    if (!in.read(reinterpret_cast<char *>(&ts_ns), sizeof(ts_ns))) break;
    uint8_t size = 0;
    if (!in.read(reinterpret_cast<char *>(&size), sizeof(size))) break;
    std::vector<uint8_t> bytes(size);
    if (size > 0 && !in.read(reinterpret_cast<char *>(bytes.data()), size)) break;
    if (is_rx) {
      print_rx_struct(ts_ns, bytes);
    } else {
      print_tx_struct(ts_ns, bytes);
    }
  }
  return 0;
}

int dump_jsonl(const std::string & path)
{
  // jsonl 本身就是文本，直接逐行 cat 即可，但去掉超长行的展开让人看着舒服
  std::ifstream in(path);
  if (!in) {
    std::cerr << "open failed: " << path << "\n";
    return 1;
  }
  std::cout << "# file: " << path << "\n";
  std::string line;
  while (std::getline(in, line)) {
    std::cout << line << "\n";
  }
  return 0;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "usage: recorder_dump <file>\n"
              << "  file ending in _rx.bin / _tx.bin / _signals.jsonl\n";
    return 1;
  }
  std::string path = argv[1];
  if (ends_with(path, "_rx.bin")) return dump_bin(path, /*is_rx=*/true);
  if (ends_with(path, "_tx.bin")) return dump_bin(path, /*is_rx=*/false);
  if (ends_with(path, ".jsonl")) return dump_jsonl(path);
  std::cerr << "unknown file type: " << ext_of(path) << "\n";
  return 1;
}
