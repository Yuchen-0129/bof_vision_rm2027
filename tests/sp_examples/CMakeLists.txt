cmake_minimum_required(VERSION 3.16)

# 仅编译本目录下的 ROS2 桥层与三个 ROS2 测试。
# 同目录其余 .cpp 为 SP 入口源码的文本归档，不加入构建。

find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(sp_msgs QUIET)

if(NOT (ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND rosidl_typesupport_cpp_FOUND
        AND sp_msgs_FOUND))
  message(STATUS "ROS2 not found, sp_examples ROS2 bridge & tests skipped.")
  return()
endif()

message(STATUS "ROS2 found, building sp_examples ROS2 bridge & tests.")

add_library(
  sp_io_ros2 STATIC
  io/ros2/publish2nav.cpp
  io/ros2/ros2.cpp
  io/ros2/subscribe2nav.cpp)
target_include_directories(sp_io_ros2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
ament_target_dependencies(sp_io_ros2 rclcpp std_msgs sp_msgs)

add_executable(publish_test tests/publish_test.cpp)
add_executable(subscribe_test tests/subscribe_test.cpp)
add_executable(topic_loop_test tests/topic_loop_test.cpp)

target_link_libraries(publish_test ${OpenCV_LIBS} fmt::fmt tools io auto_aim sp_io_ros2)
target_link_libraries(subscribe_test ${OpenCV_LIBS} fmt::fmt tools io sp_io_ros2)
target_link_libraries(topic_loop_test ${OpenCV_LIBS} fmt::fmt tools io sp_io_ros2)
