cmake_minimum_required(VERSION 3.8)
project(feature_tracker)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_model REQUIRED)
find_package(dvs_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(Ceres REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIR}
)

set(LINE_DESCRIPTOR_SOURCES
    src/line_descriptor/src/binary_descriptor_custom.cpp
    src/line_descriptor/src/binary_descriptor_matcher.cpp
    src/line_descriptor/src/bitarray_custom.hpp
    src/line_descriptor/src/bitops_custom.hpp
    src/line_descriptor/src/draw_custom.cpp
    src/line_descriptor/src/LSDDetector_custom.cpp
    src/line_descriptor/src/precomp_custom.hpp
    src/line_descriptor/src/types_custom.hpp)

set(COMMON_DEPS rclcpp sensor_msgs geometry_msgs std_msgs cv_bridge camera_model dvs_msgs)

# 事件特征
add_executable(feature_tracker
    src/event_tracker_node.cpp
    src/parameters.cpp
    src/feature_tracker.cpp
    src/arc_star/arc_star_detector.cc
    src/utility/visualization.cpp
    ${LINE_DESCRIPTOR_SOURCES})
ament_target_dependencies(feature_tracker ${COMMON_DEPS})
target_link_libraries(feature_tracker ${OpenCV_LIBS})

# 图像特征
# NOTE: deliberately does NOT link src/utility/visualization.cpp -- image_tracker_node.cpp
# defines its own pub_img/pub_match/pub_restart globals (matching visualization.h's extern
# declarations pulled in transitively via feature_tracker.h); linking visualization.cpp here
# too would double-define those symbols. Mirrors the original ROS1 CMakeLists.txt structure.
add_executable(image_feature_tracker
    src/image_tracker_node.cpp
    src/parameters.cpp
    src/feature_tracker.cpp
    src/arc_star/arc_star_detector.cc
    ${LINE_DESCRIPTOR_SOURCES})
ament_target_dependencies(image_feature_tracker ${COMMON_DEPS})
target_link_libraries(image_feature_tracker ${OpenCV_LIBS})

install(TARGETS feature_tracker image_feature_tracker
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
