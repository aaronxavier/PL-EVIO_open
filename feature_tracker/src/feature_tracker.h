#pragma once

#include <cstdio>
#include <iostream>
#include <iomanip>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <cmath>

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <eigen3/Eigen/Geometry>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "tic_toc.h"

#include <sensor_msgs/msg/imu.hpp>
#include <dvs_msgs/msg/event.hpp>
#include <dvs_msgs/msg/event_array.hpp>
#include "arc_star/arc_star_detector.h"
#include "utility/visualization.h"

#include <opencv2/features2d.hpp>
#include "line_descriptor/include/line_descriptor_custom.hpp"


using namespace std;
using namespace camodocal;
using namespace Eigen;

using Motion_correction_value=std::pair<bool, std::pair<std::pair<Eigen::Vector4d, Eigen::Vector3f>, std::pair< Eigen::Vector2d,std::pair<Eigen::Vector3f,Eigen::Vector3f>>>>;


bool inBorder(const cv::Point2f &pt);

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

//线特征相关的
struct Line
{
	cv::Point2f StartPt;//起始点
	cv::Point2f EndPt;//终止点
	float lineWidth;//线的宽度
	cv::Point2f Vp;

	cv::Point2f Center;
	cv::Point2f unitDir; // [cos(theta), sin(theta)]
	float length;//线的长度
	float theta;

	// para_a * x + para_b * y + c = 0
	float para_a;
	float para_b;
	float para_c;

	float image_dx;
	float image_dy;
    float line_grad_avg;

	float xMin;
	float xMax;
	float yMin;
	float yMax;
	unsigned short id;
	int colorIdx;
};

class FrameLines//每一帧
{
public:
    int frame_id;//帧id
    cv::Mat img;//图像
    cv::Mat event_img;//存放事件帧
    
    vector<Line> vecLine;//线的向量
    vector< int > lineID;//线的id

    // opencv3 lsd+lbd
    std::vector<cv::line_descriptor::KeyLine> keylsd;//线特征
    cv::Mat lbd_descr;//线特征的描述子
};
typedef shared_ptr< FrameLines > FrameLinesPtr;

class FeatureTracker
{
  public:
    FeatureTracker();

    // void save_feature_process();//保存feature 处理过程

    void readImage(const cv::Mat &_img,double _cur_time);//做image的tracking
    void readEvent(const dvs_msgs::msg::EventArray &last_event, double _cur_time);//做event stream的tracking
    void readEvent(const dvs_msgs::msg::EventArray &last_event, double _cur_time, const Motion_correction_value measurements);//做event stream的tracking(加入imu信息做运动补偿用的)

    void trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1); //用于做双目的feature tracking

    void setMask();
    void stereo_setMask();
    void setevent_Mask();

    void addPoints();

    bool updateID(unsigned int i);

    void readIntrinsicParameter(const string &calib_file);
    void stereo_readIntrinsicParameter(vector<string> &calib_file);

    void showUndistortion(const string &name);

    void rejectWithF();//mono版本
    void setreo_rejectWithF();//双目版本
    void rejectWithF_event();//自定义版本（改进fusion）
    void rejectWithF_line(const std::vector<cv::line_descriptor::KeyLine> octave0_1, const std::vector<cv::line_descriptor::KeyLine>octave0_2, std::vector<cv::DMatch> good_matches);//去除线特征的外点

    void undistortedPoints();

    cv::Mat getTrackImage();
    cv::Mat getLoopImage();//返回用于回环检测的图像
    cv::Mat getTrackImage_two();//f返回前后两帧matching的结果
    cv::Mat getTrackImage_two_line();//返回前后两帧 线特征 matching的结果
    cv::Mat gettimesurface();//返回time surface用于可视化

    double distance(cv::Point2f &pt1, cv::Point2f &pt2);

        // ######################################画出跟踪的点
    void event_drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                                   vector<int> &curLeftIds,
                                   vector<cv::Point2f> &curLeftPts, 
                                   vector<cv::Point2f> &curRightPts,
                                   map<int, cv::Point2f> &prevLeftPtsMap);
    
    void event_drawTrack_two(const cv::Mat &imLeft, const cv::Mat &imRight, 
                                vector<int> &curLeftIds,
                                vector<cv::Point2f> &curLeftPts, 
                                vector<cv::Point2f> &curRightPts,
                                map<int, cv::Point2f> &prevLeftPtsMap);
    // ###################################################画出跟踪线
    void event_drawTrack_two_line(const cv::Mat imageMat1, const cv::Mat imageMat2,
                          const std::vector<cv::line_descriptor::KeyLine> octave0_1, const std::vector<cv::line_descriptor::KeyLine>octave0_2,
                          const std::vector<cv::DMatch> good_matches);

    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                            vector<int> &curLeftIds,
                            vector<cv::Point2f> &curLeftPts, 
                            vector<cv::Point2f> &curRightPts,
                            map<int, cv::Point2f> &prevLeftPtsMap);

// 计算没有失真的点
    vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);
    cv::Point2f undistortedPts(cv::Point2f &pts, camodocal::CameraPtr cam);
    // 计算点的速度
    vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                    map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);
    cv::Mat mask;
    cv::Mat mask_arc;//事件数据用的mask
    cv::Mat fisheye_mask;
    cv::Mat prev_img;
    cv::Mat cur_img;//上一帧
    cv::Mat forw_img;//当前的image
    vector<cv::Point2f> n_pts;
    vector<cv::Point2f> prev_pts, cur_pts;//之前的特征点
    vector<cv::Point2f> cur_right_pts;//当前右相机的特征点，当为单目的时候，为空，只是辅助画图
    vector<cv::Point2f>  forw_pts;//当前的特征点
    vector<cv::Point2f> prev_un_pts, cur_un_pts;
    vector<cv::Point2f> cur_un_right_pts;//当前未失真的右相机的特征点
    vector<cv::Point2f> pts_velocity;
    vector<cv::Point2f> right_pts_velocity;//右相机特征点的速度
    vector<int> ids, ids_right;
    vector<int> track_cnt;//一共的跟踪数
    map<int, cv::Point2f> cur_un_pts_map;
    map<int, cv::Point2f> prev_un_pts_map;
    map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;//给右相机用的
    camodocal::CameraPtr m_camera;
    vector<camodocal::CameraPtr> setro_m_camera;//双目的时候用的
    double fx,fy,cx,cy;
    double cur_time;//当前的时间
    double prev_time;

  //跟踪的结果图像
    cv::Mat imTrack;//用于跟踪
    cv::Mat imTrack_two;//可视化前后帧跟踪过程
    cv::Mat time_surface_visualization;//用于可视化time surface map
    cv::Mat imTrack_two_line;
    cv::Mat Image_loop;//回环检测用的图像

    map<int, cv::Point2f> prevLeftPtsMap;//之前的特征点（id与点的集合）

    bool FLAG_DETECTOR_NOSTART=true;//用于初始化角点检测器

    static int n_id;//点特征的id统计

    FrameLinesPtr curframe_;//（当前已有的）线特征的帧
    FrameLinesPtr forwframe_;//（最新的）线特征的帧
    cv::Mat undist_map1_, undist_map2_ , K_;//线特征未失真的帧
    vector<int> ids_line;                     // 每条线特征的id
    vector<int> linetrack_cnt;           // 记录某条线特征已经跟踪多少帧了，即被多少帧看到了
    static int allfeature_cnt;                  // 用来统计整个地图中有了多少条线，它将用来赋值


//下面定义的函数用于保存图片
    void save_event_tracking(const cv::Mat &image, const double t){
        std::stringstream string_timestamp;
        string_timestamp << std::setprecision(15) << t;
        std::string str_timestamp = string_timestamp.str(); 

        ostringstream path;
        path <<  "/home/kwanwaipang/PL-EVIO/Event_tracking/"
                <<"timestamp:"<< str_timestamp << "---"
                << "tracking.jpg";
        cv::imwrite( path.str().c_str(), image);
    }

    void save_line_tracking(const cv::Mat &image, const double t){
        std::stringstream string_timestamp;
        string_timestamp << std::setprecision(15) << t;
        std::string str_timestamp = string_timestamp.str(); 

        ostringstream path;
        path <<  "/home/kwanwaipang/PL-EVIO/Line_tracking/"
                <<"timestamp:"<< str_timestamp << "---"
                << "tracking.jpg";
        cv::imwrite( path.str().c_str(), image);
    }

    void save_image_tracking(const cv::Mat &image, const double t){
        std::stringstream string_timestamp;
        string_timestamp << std::setprecision(15) << t;
        std::string str_timestamp = string_timestamp.str(); 

        ostringstream path;
        path <<  "/home/kwanwaipang/PL-EVIO/Image_tracking/"
                <<"timestamp:"<< str_timestamp << "---"
                << "tracking.jpg";
        cv::imwrite( path.str().c_str(), image);
    }    
};


template <typename T>
using Mat3 = typename Eigen::Matrix<T, 3, 3>;

/**
 * @brief convert a vector to skew matrix
 *
 * @tparam T
 * @param v
 * @return Mat3<typename T::Scalar>
 */
template <typename T>
Mat3<typename T::Scalar> vectorToSkewMat(const Eigen::MatrixBase<T> &v) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                "Must have 3x1 matrix");
  Mat3<typename T::Scalar> m;
  m << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  return m;
}

/**
 * @brief convert a vector to homogeneous coordinates
 *
 * @param v
 */
inline void ConvertToHomogeneous(Eigen::Vector3f *v) {
  (*v)[0] = (*v)[0] / (*v)[2];
  (*v)[1] = (*v)[1] / (*v)[2];
  (*v)[2] = 1;
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::msg::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{//把从imu传感器中的数据转变成ros中的数据
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}

// //排序方法
// // bool cmp_event_time(const double& a, const double& b)
// bool cmp_event_time(const dvs_msgs::Event& a, const dvs_msgs::Event& b)
// {
// 	// return a > b; //从大到小排序
//     // return a < b; //从小到大排序
//     return a.ts < b.ts; //从小到大排序
// }