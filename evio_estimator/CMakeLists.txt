cmake_minimum_required(VERSION 3.8)
project(evio_estimator)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_model REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

set(COMMON_DEPS rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs tf2_ros cv_bridge camera_model)

add_executable(evio_estimator
    src/estimator_node.cpp
    src/parameters.cpp
    src/estimator.cpp
    src/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projection_factor.cpp
    src/factor/projection_td_factor.cpp
    src/factor/marginalization_factor.cpp
    src/factor/sampson_factor.cpp
    src/factor/sampson_jacobian.cpp
    src/factor/projectionOneFrameTwoCamFactor.cpp
    src/factor/projectionTwoFrameOneCamFactor.cpp
    src/factor/projectionTwoFrameTwoCamFactor.cpp
    src/utility/utility.cpp
    src/utility/visualization.cpp
    src/utility/CameraPoseVisualization.cpp
    src/initial/solve_5pts.cpp
    src/initial/solve_opt.cpp
    src/initial/translate_factor.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp

    # 新加入的线特征相关的
    src/utility/line_geometry.cpp
    src/factor/line_parameterization.cpp
    src/factor/line_projection_factor.cpp
    )

ament_target_dependencies(evio_estimator ${COMMON_DEPS})
target_link_libraries(evio_estimator ${OpenCV_LIBS} ${CERES_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS evio_estimator
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
