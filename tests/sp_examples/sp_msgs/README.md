# `sp_msgs` 接口包（自定义 ROS2 消息）

本目录是从 [tests/sp_examples/](../) 中 SP 桥层与测试代码反推出的最小可用 ROS2 接口包，**不参与本仓库的 `cmake -B build` 主构建**，由独立的 colcon 工作区拾起后构建。

## 消息字段（最小集）

- `sp_msgs/msg/EnemyStatusMsg`
  - `builtin_interfaces/Time timestamp`
  - `int8[] invincible_enemy_ids`
- `sp_msgs/msg/AutoaimTargetMsg`
  - `builtin_interfaces/Time timestamp`
  - `int8[] target_ids`

字段来源：参见 [../../io/ros2/subscribe2nav.cpp](../../../io/ros2/subscribe2nav.cpp)、[../../io/ros2/subscribe2nav.hpp](../../../io/ros2/subscribe2nav.hpp)、[../tests/topic_loop_test.cpp](../tests/topic_loop_test.cpp)。

## 构建与使用（colcon 软链工作区）

```bash
# 一次性：建立独立的 colcon 工作区并软链 sp_msgs
mkdir -p ~/sp_ws/src
ln -s /home/kk/workspace/bof_vision/bof_vision_26/tests/sp_examples/sp_msgs ~/sp_ws/src/sp_msgs

# 构建（在 ROS2 已 source 的 shell 里）
source /opt/ros/humble/setup.bash
cd ~/sp_ws && colcon build --packages-select sp_msgs
source ~/sp_ws/install/setup.bash

# 之后回项目根再配置主工程
cd /home/kk/workspace/bof_vision/bof_vision_26
cmake -B build && cmake --build build -j8
```

`source ~/sp_ws/install/setup.bash` 之后，[../CMakeLists.txt](../CMakeLists.txt) 里的 `find_package(sp_msgs QUIET)` 才会命中并启用 SP 的 ROS2 桥层与三个测试。

## 卸载

直接 `rm -rf ~/sp_ws` 即可，本仓库内容不受影响。
