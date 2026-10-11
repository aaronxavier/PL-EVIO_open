#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "CameraPoseVisualization.h"
#include <eigen3/Eigen/Dense>
#include "../estimator.h"
#include "../parameters.h"
#include <fstream>

extern rclcpp::Node::SharedPtr g_node;//定义于 estimator_node.cpp，pubTF()里tf2_ros广播器需要

extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry;
extern rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path, pub_relo_path;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_point_cloud, pub_margin_cloud;
extern rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_key_poses;
extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_relo_relative_pose;
extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_camera_pose;
extern rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_camera_pose_visual;
extern nav_msgs::msg::Path path, relo_path;

extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_keyframe_pose;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_keyframe_point;
extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_extrinsic;

extern rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr evio_pose;//把pose以geometry_msgs::PoseStamped格式发布
extern rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr gt_pose;//把ground truth的pose以geometry_msgs::PoseStamped格式发布
extern rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_latest_evio;//插值了IMU的（以足够快的速率来发布）

void registerPub(rclcpp::Node::SharedPtr n);

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, const std_msgs::msg::Header &header);////按照imu频率预测位姿并发送，这样就可以提高里程计频率。此处有两个话题“imu_propagate”与“imu_evio”

void printStatistics(const Estimator &estimator, double t);

void pubOdometry(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubInitialGuess(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubKeyPoses(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubCameraPose(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubPointCloud(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubTF(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubKeyframe(const Estimator &estimator);

void pubRelocalization(const Estimator &estimator);

void pubGT(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);//发布GT
