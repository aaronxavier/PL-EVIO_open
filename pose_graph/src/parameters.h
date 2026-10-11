#pragma once

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <cv_bridge/cv_bridge.h>

// Shim so the original ROS1 ROS_DEBUG/INFO/WARN/ERROR(_STREAM)/ASSERT/BREAK call sites
// throughout this package's ported algorithm code can stay textually unchanged.
#include <cassert>
#include <cstdlib>
#define ROS_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("pose_graph"), __VA_ARGS__)
#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("pose_graph"), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("pose_graph"), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("pose_graph"), __VA_ARGS__)
#define ROS_DEBUG_STREAM(...) RCLCPP_DEBUG_STREAM(rclcpp::get_logger("pose_graph"), __VA_ARGS__)
#define ROS_INFO_STREAM(...) RCLCPP_INFO_STREAM(rclcpp::get_logger("pose_graph"), __VA_ARGS__)
#define ROS_WARN_STREAM(...) RCLCPP_WARN_STREAM(rclcpp::get_logger("pose_graph"), __VA_ARGS__)
#define ROS_ERROR_STREAM(...) RCLCPP_ERROR_STREAM(rclcpp::get_logger("pose_graph"), __VA_ARGS__)
#define ROS_ASSERT(cond) assert(cond)
#define ROS_BREAK() abort()

// Replaces ROS1's `ros::Time(t)` for a double seconds-since-epoch value.
inline builtin_interfaces::msg::Time secondsToRosTime(double t)
{
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(t);
    stamp.nanosec = static_cast<uint32_t>((t - stamp.sec) * 1e9);
    return stamp;
}

extern camodocal::CameraPtr m_camera;
extern Eigen::Vector3d tic;
extern Eigen::Matrix3d qic;
extern rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match_img;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_match_points;
extern int VISUALIZATION_SHIFT_X;
extern int VISUALIZATION_SHIFT_Y;
extern std::string BRIEF_PATTERN_FILE;
extern std::string POSE_GRAPH_SAVE_PATH;
extern int ROW;
extern int COL;
extern std::string VINS_RESULT_PATH;
extern int DEBUG_IMAGE;
extern int FAST_RELOCALIZATION;
extern int SAVE_LOOP_MATCH;

extern double TS_LK_THRESHOLD;//用于提取time surface不为128的点
