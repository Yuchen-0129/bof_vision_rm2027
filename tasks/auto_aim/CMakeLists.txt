cmake_minimum_required(VERSION 3.16)

set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_subdirectory(auto_aim_fire/planner/tinympc)

add_library(auto_aim_detect OBJECT
    auto_aim_detect/classifier.cpp
    auto_aim_detect/detector.cpp
    auto_aim_detect/yolo.cpp
    auto_aim_detect/yolos/yolov5.cpp
    auto_aim_detect/yolos/yolov8.cpp
    auto_aim_detect/yolos/yolo11.cpp
)

target_link_libraries(auto_aim_detect io openvino::runtime)

add_library(auto_aim_track OBJECT
    common/armor.cpp
    auto_aim_track/solver.cpp
    auto_aim_track/target.cpp
    auto_aim_track/tracker.cpp
)

target_link_libraries(auto_aim_track io)

add_library(auto_aim_fire OBJECT
    auto_aim_fire/aimer.cpp
    auto_aim_fire/shooter.cpp
    auto_aim_fire/planner/planner.cpp
    multithread/commandgener.cpp
    multithread/mt_detector.cpp
)

target_link_libraries(auto_aim_fire io tinympcstatic openvino::runtime)

add_library(auto_aim STATIC
    $<TARGET_OBJECTS:auto_aim_detect>
    $<TARGET_OBJECTS:auto_aim_track>
    $<TARGET_OBJECTS:auto_aim_fire>
)

target_link_libraries(auto_aim io openvino::runtime tinympcstatic)