# SP 参考代码归档（`tests/sp_examples`）

本目录收纳原 SP 战队开源仓库中的示例与 ROS2 桥层代码，供后续开发对照阅读。

## 内容说明

- **根目录下若干 `*.cpp`**（如 `standard.cpp`、`sentry*.cpp`、`uav*.cpp` 等）：仅作文本归档，**不参与 CMake 构建**。
- **`io/ros2/`**：ROS2 与导航相关的桥接实现；当系统检测到 ROS2 与 `sp_msgs` 等依赖时，由同目录下的 `CMakeLists.txt` 编译为静态库 `sp_io_ros2`。
- **`tests/`**：三个 ROS2 小测试（`publish_test`、`subscribe_test`、`topic_loop_test`），在 ROS2 可用时一并编译。

## 如何编译 ROS2 部分

在已 `source /opt/ros/<distro>/setup.bash` 且工作空间能找到 `sp_msgs` 的前提下，在项目根目录执行：

```bash
cmake -S . -B build && cmake --build build -j
```

若未配置 ROS2，配置阶段会跳过本目录的 ROS2 目标，不影响主工程其余目标。

## 主工程

主程序入口仅保留 `src/auto_aim_debug_aimer.cpp`；主 `io` 库不再包含 ROS2 源码。
