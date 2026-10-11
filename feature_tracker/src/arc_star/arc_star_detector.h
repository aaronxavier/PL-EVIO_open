#ifndef ARC_STAR_DETECTOR_H
#define ARC_STAR_DETECTOR_H

#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../parameters.h"
#include <cmath>
#include <thread>
#include "../feature_tracker.h"
#include <mutex>

namespace evio { // Asynchronous Corner Detector

using Motion_correction_value=std::pair<bool, std::pair<std::pair<Eigen::Vector4d, Eigen::Vector3f>, std::pair< Eigen::Vector2d,std::pair<Eigen::Vector3f,Eigen::Vector3f>>>>;

// #####################################################
class ArcStarDetector
{
public:
  ArcStarDetector();
  ArcStarDetector(int col, int row);
  ~ArcStarDetector();

void init(int col, int row);
void init(int col, int row, const double fx, const double fy, const double cx, const double cy);
void createSAE(double et, int ex, int ey, bool ep);//输入数据，产生SAE以及SAE_LAST
void createSAE(double et, int ex, int ey, bool ep, const Motion_correction_value measurements);//输入数据，产生SAE以及SAE_LAST 并且加入运动补偿

cv::Mat SAEtoTimeSurface(const double external_sync_time);//从SAE产生TS，通过参数文件设置是否带极性
cv::Mat SAE_Last_toTimeSurface( const double external_sync_time);//从SAE_LAST产生TS，通过参数文件设置是否带极性
cv::Mat SAE_Last_toTimeSurface_withoutP( const double external_sync_time);//从SAE_LAST产生TS，不带极性

cv::Mat SAE_toTimeSurface_withoutP( const double external_sync_time);//从SAE产生TS，不带极性
cv::Mat SAE_toTimeSurface_withoutP_multi_thread(const double external_sync_time);//(多线程)从SAE产生TS，不带极性

bool isCorner(double et, int ex, int ey, bool ep);//单单提取feature。需要额外产生SAE（基于createSAE）
bool isFeature(double et, int ex, int ey, bool ep);//产生SAE同时提取feature（原来的Arc star）


bool isFeature(double et, int ex, int ey, bool ep, const Motion_correction_value measurements);//输入数据，产生SAE以及SAE_LAST (同时做运动补偿)
Eigen::Vector2d motioncorrection(const double ex,const double ey,const Eigen::Vector3f tmp_v,const Eigen::Vector3f accel_avg_, const Eigen::Vector3f omega_avg_, const double dt);

void motion_compensation_function(double et, int ex, int ey, bool ep, const Motion_correction_value measurements);

double last_event_time;//在createSAE中产生，记录当前最晚/最新的事件的时间

cv::Mat cur_event_mat;//将当前的event数据转换为mat的形式

dvs_msgs::msg::EventArray motion_correct_eventstream;

  // int kSensorWidth_ = 346;
  // int kSensorHeight_= 260;
  //   int kSensorWidth_;
  // int kSensorHeight_;

  // int kSensorWidth_ = 240;
  // int kSensorHeight_= 180;

private:
  
  const double t_motion_compensation_threshold=0.01; //0.01;

  // Circular Breshenham Masks
  const int kSmallCircle_[16][2];
  const int kLargeCircle_[20][2];

  int kSensorWidth_;//记录x
  int kSensorHeight_;//记录y
  Eigen::Matrix3f intrinsics_matrix;//相机内参

  cv::Size sensor_size_;
  double decay_ms_;
  bool ignore_polarity_;
  int median_blur_kernel_size_;
  double decay_ms_for_loop;

  // Parameters
  // constexpr static const double filter_threshold_ = 0.050;//算是设置检测的时间密度？原本值为 0.050 (0.1)
  // “const 和 constexpr 变量之间的主要区别在于:const 变量的初始化可以延迟到运行时,而 constexpr 变量必须在编译时进行初始化
  double filter_threshold_;

  // static const int kSensorWidth_ = 240;
  // static const int kSensorHeight_= 180;
  // static const int kSensorWidth_ = 346;
  // static const int kSensorHeight_= 260;

  // Surface of Active Events
  Eigen::MatrixXd sae_[2];//存放sae
  Eigen::MatrixXd sae_latest_[2];//存放上一时刻的sae
  //定义一个没有极性的SAE？？？？
};
// #####################################################

// class ArcStarDetector
// {
// public:
//   ArcStarDetector();
//   ~ArcStarDetector();

//   bool isCorner(double et, int ex, int ey, bool ep);

// private:
//   // Circular Breshenham Masks
//   const int kSmallCircle_[16][2];
//   const int kLargeCircle_[20][2];

//   // Parameters
//   constexpr static const double filter_threshold_ = 0.050;
//   // static const int kSensorWidth_ = 240;
//   // static const int kSensorHeight_= 180;
//   static const int kSensorWidth_ = 346;
//   static const int kSensorHeight_= 260;

//   // Surface of Active Events
//   Eigen::MatrixXd sae_[2];
//   Eigen::MatrixXd sae_latest_[2];
// };

} // Asynchronous Corner Detector

#endif // ARC_STAR_DETECTOR_H
