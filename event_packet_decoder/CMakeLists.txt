cmake_minimum_required(VERSION 3.8)
project(event_packet_decoder)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(event_camera_msgs REQUIRED)
find_package(event_camera_codecs REQUIRED)
find_package(dvs_msgs REQUIRED)

add_library(event_packet_decoder_component SHARED
  src/event_packet_decoder_node.cpp)

target_include_directories(event_packet_decoder_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(event_packet_decoder_component
  rclcpp
  rclcpp_components
  event_camera_msgs
  event_camera_codecs
  dvs_msgs)

rclcpp_components_register_node(event_packet_decoder_component
  PLUGIN "event_packet_decoder::EventPacketDecoderNode"
  EXECUTABLE event_packet_decoder_node)

install(TARGETS event_packet_decoder_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
