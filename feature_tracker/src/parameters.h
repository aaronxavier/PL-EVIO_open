#pragma once
#include <rclcpp/rclcpp.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// Shim so the original ROS1 ROS_DEBUG/INFO/WARN/ERROR(_STREAM) call sites throughout
// this package's ported algorithm code (feature_tracker.cpp, arc_star_detector.cc, ...)
// can stay textually unchanged.
#define ROS_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("feature_tracker"), __VA_ARGS__)
#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("feature_tracker"), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("feature_tracker"), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("feature_tracker"), __VA_ARGS__)
#define ROS_DEBUG_STREAM(...) RCLCPP_DEBUG_STREAM(rclcpp::get_logger("feature_tracker"), __VA_ARGS__)
#define ROS_INFO_STREAM(...) RCLCPP_INFO_STREAM(rclcpp::get_logger("feature_tracker"), __VA_ARGS__)
#define ROS_WARN_STREAM(...) RCLCPP_WARN_STREAM(rclcpp::get_logger("feature_tracker"), __VA_ARGS__)
#define ROS_ERROR_STREAM(...) RCLCPP_ERROR_STREAM(rclcpp::get_logger("feature_tracker"), __VA_ARGS__)

extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
const int NUM_OF_CAM = 1; 
const int NUM_OF_CAM_stereo = 2; 
extern int STEREO ;//是否采用双目


extern std::string IMAGE_TOPIC;
extern std::string IMAGE1_TOPIC;//双目时，左相机的topic
extern std::string IMAGE2_TOPIC; //双目时，右相机的topic
extern std::string EVENT_TOPIC;//事件的话题
extern std::string IMU_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;//事件特征的距离
extern int MIN_DIST_IMG;//图像特征的距离
extern int WINDOW_SIZE;
extern int FREQ;//发布事件前端的频率
extern int FREQ_IMG;//发布图像前端的频率
extern double F_THRESHOLD;
extern double TS_LK_THRESHOLD;
extern int para_ignore_polarity;//true;
extern double para_decay_ms;//60;//延迟 30
extern double para_decay_loop_ms;//用于回环的时延迟
extern int para_median_blur_kernel_size;//1;
extern double para_feature_filter_threshold;//处理的间隔时间

//线特征相关的
extern double MIN_length;//提取的线特征最小的长度
extern double Scale_image;//The scale of the image that will be used to find the lines. Range (0..1].
extern double Maximun_lines;//最大数目的线
extern int Do_motion_correction;//是否去除event的运动畸变

extern int SHOW_TRACK;
extern int FLOW_BACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;
extern Eigen::Matrix3d Eeesntial_matrix;// essential matrix from left and right camera

extern int Num_of_thread;//处理大量event数组的时候采用多少个线程

void readParameters(rclcpp::Node::SharedPtr n);
