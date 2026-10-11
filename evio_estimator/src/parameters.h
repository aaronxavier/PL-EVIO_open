#pragma once

#include <cassert>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

// Shim so the original ROS1 ROS_DEBUG/INFO/WARN/ERROR(_STREAM)/ASSERT call sites
// throughout this package's ported algorithm code can stay textually unchanged.
#define ROS_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("evio_estimator"), __VA_ARGS__)
#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("evio_estimator"), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("evio_estimator"), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("evio_estimator"), __VA_ARGS__)
#define ROS_DEBUG_STREAM(...) RCLCPP_DEBUG_STREAM(rclcpp::get_logger("evio_estimator"), __VA_ARGS__)
#define ROS_INFO_STREAM(...) RCLCPP_INFO_STREAM(rclcpp::get_logger("evio_estimator"), __VA_ARGS__)
#define ROS_WARN_STREAM(...) RCLCPP_WARN_STREAM(rclcpp::get_logger("evio_estimator"), __VA_ARGS__)
#define ROS_ERROR_STREAM(...) RCLCPP_ERROR_STREAM(rclcpp::get_logger("evio_estimator"), __VA_ARGS__)
#define ROS_ASSERT(cond) assert(cond)
#define ROS_BREAK() abort()

#define SQ(x) (((x)*(x)))

const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10;
const int NUM_OF_CAM = 1; 
const int NUM_OF_CAM_stereo = 2; 
const int NUM_OF_F = 3000; //1000;
//#define UNIT_SPHERE_ERROR

extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;

//下面是双目版本新增的
extern std::vector<std::string> CAM_NAMES;
extern Eigen::Matrix3d Rlr;     // Tlr 
extern Eigen::Vector3d Tlr; 
extern Eigen::Matrix3d Rrl;     // Trl 
extern Eigen::Vector3d Trl; 

extern int temp_T;//用于查看是否需要转置的

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;
extern std::string IMU_TOPIC;
extern double TD;
extern double TR;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern double ROW, COL;

// 双目的一些参数
extern double nG;
extern bool g_use_sampson_model;
extern bool g_use_stereo_correction; 
extern bool g_opt_verbose; 

// 给线特征用的
const int LINE_MIN_OBS = 2; //2;//最小的线观测需求，原本设置为5，此处为保证给线特征的一致性为2


void readParameters(rclcpp::Node::SharedPtr n);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1,
    SIZE_LINE = 4
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
