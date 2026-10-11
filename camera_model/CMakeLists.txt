cmake_minimum_required(VERSION 3.8)
project(camera_model)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS})

set(CAMERA_MODEL_SOURCES
    src/chessboard/Chessboard.cc
    src/calib/CameraCalibration.cc
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/CostFunctionFactory.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/CataCamera.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/sparse_graph/Transform.cc
    src/gpl/gpl.cc
    src/gpl/EigenQuaternionParameterization.cc)

add_library(camera_model ${CAMERA_MODEL_SOURCES})
target_link_libraries(camera_model ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_include_directories(camera_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(Calibration src/intrinsic_calib.cc ${CAMERA_MODEL_SOURCES})
target_link_libraries(Calibration ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

install(TARGETS camera_model
  EXPORT camera_modelTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS Calibration
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(camera_model)
ament_export_targets(camera_modelTargets)
install(EXPORT camera_modelTargets
  DESTINATION share/${PROJECT_NAME}/cmake)

ament_package()
