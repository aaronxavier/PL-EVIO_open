#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>

#include <dvs_msgs/msg/event_array.hpp>
#include <builtin_interfaces/msg/time.hpp>

// Replaces ROS1's `header.stamp = ros::Time(t)` for a double seconds-since-epoch value.
inline builtin_interfaces::msg::Time secondsToRosTime(double t)
{
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(t);
    stamp.nanosec = static_cast<uint32_t>((t - stamp.sec) * 1e9);
    return stamp;
}

extern rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_loop_image;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_img;
extern rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match, pub_match_two;
extern rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_time_surface;
extern rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_restart;
extern rclcpp::Publisher<dvs_msgs::msg::EventArray>::SharedPtr corner_pub;//把当前事件帧的所有feature发布出去

extern rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match_two_line;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_feature_line;//线特征相关的

void registerPub(rclcpp::Node::SharedPtr n);
void pubLoopImage(const cv::Mat &imgTrack, const double t);
