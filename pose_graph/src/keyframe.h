#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "utility/tic_toc.h"
#include "utility/utility.h"
#include "parameters.h"
#include "ThirdParty/DBoW/DBoW2.h"
#include "ThirdParty/DVision/DVision.h"
#include <dvs_msgs/msg/event.hpp>
#include <dvs_msgs/msg/event_array.hpp>

#define MIN_LOOP_NUM 30 //26 //16 //6 //16 //9 //20 //(回环匹配点的数目要大于25) 判断匹配上的点数目大小够不够

using namespace Eigen;
using namespace std;
using namespace DVision;


class BriefExtractor//描述子的类 (通过Brief模板文件，对图像的关键点计算Brief描述子)
{//构建Brief产生器，用于通过Brief模板文件对图像特征点计算Brief描述子
public:
    //运算符重载了“（）”来计算描述子。
  virtual void operator()(const cv::Mat &im, vector<cv::KeyPoint> &keys, vector<BRIEF::bitset> &descriptors) const;
  //读取 构建字典时使用的相同的Brief模板文件，构造BriefExtractor
  BriefExtractor(const std::string &pattern_file);

  DVision::BRIEF m_brief;
};

class KeyFrame//构建关键帧的类(通过BRIEF描述子匹配关键帧和回环候选帧)
{
public:

	KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, cv::Mat &_image,
			 vector<cv::Point3f> &_point_3d, vector<cv::Point2f> &_point_2d_uv, vector<cv::Point2f> &_point_2d_normal, 
			 vector<double> &_point_id, int _sequence, dvs_msgs::msg::EventArray &_event_feature_point);//第三个构造函数,创建新的关键帧

	KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, cv::Mat &_image,
			 vector<cv::Point3f> &_point_3d, vector<cv::Point2f> &_point_2d_uv, vector<cv::Point2f> &_point_2d_normal, 
			 vector<double> &_point_id, int _sequence);//第一个构造函数,创建新的关键帧
	
	KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, Vector3d &_T_w_i, Matrix3d &_R_w_i,
			 cv::Mat &_image, int _loop_index, Eigen::Matrix<double, 8, 1 > &_loop_info,
			 vector<cv::KeyPoint> &_keypoints, vector<cv::KeyPoint> &_keypoints_norm, vector<BRIEF::bitset> &_brief_descriptors);//第二个构造函数,加载之前的关键帧
	
	bool findConnection(KeyFrame* old_kf);//寻找并建立关键帧与回环帧之间的匹配关系
	void computeWindowBRIEFPoint();//计算窗口中所有特征点的描述子
	void computeBRIEFPoint();//额外检测500个新的特征点并计算所有特征点的描述子,为了回环检测

	void savecomputeWindowBRIEFPoint();//保留当前窗口中所有的特征点与描述子
	void computeBRIEFPointfromevent(const dvs_msgs::msg::EventArray &_event_feature_point);//基于事件特征点来生成feature point
	
	//void extractBrief();
	int HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b);//计算两个描述子之间的汉明距离
	bool searchInAera(const BRIEF::bitset window_descriptor,
	                  const std::vector<BRIEF::bitset> &descriptors_old,
	                  const std::vector<cv::KeyPoint> &keypoints_old,
	                  const std::vector<cv::KeyPoint> &keypoints_old_norm,
	                  cv::Point2f &best_match,
	                  cv::Point2f &best_match_norm);//关键帧中某个特征点的描述子与回环帧的所有描述子匹配
	void searchByBRIEFDes(std::vector<cv::Point2f> &matched_2d_old,
						  std::vector<cv::Point2f> &matched_2d_old_norm,
                          std::vector<uchar> &status,
                          const std::vector<BRIEF::bitset> &descriptors_old,
                          const std::vector<cv::KeyPoint> &keypoints_old,
                          const std::vector<cv::KeyPoint> &keypoints_old_norm);//将关键帧与回环帧进行BRIEF描述子匹配
	void FundmantalMatrixRANSAC(const std::vector<cv::Point2f> &matched_2d_cur_norm,
                                const std::vector<cv::Point2f> &matched_2d_old_norm,
                                vector<uchar> &status);//通过RANSAC的基本矩阵校验,去除匹配异常的点
	void PnPRANSAC(const vector<cv::Point2f> &matched_2d_old_norm,
	               const std::vector<cv::Point3f> &matched_3d,
	               std::vector<uchar> &status,
	               Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old);//通过RANSAC的PNP校验,去除匹配异常的点
	void getVioPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);
	void getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);
	void updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);
	void updateVioPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);
	void updateLoop(Eigen::Matrix<double, 8, 1 > &_loop_info);

	Eigen::Vector3d getLoopRelativeT();
	double getLoopRelativeYaw();
	Eigen::Quaterniond getLoopRelativeQ();



	double time_stamp; 
	int index;
	int local_index;
	Eigen::Vector3d vio_T_w_i; //在VIO坐标系下的位姿
	Eigen::Matrix3d vio_R_w_i; 
	Eigen::Vector3d T_w_i;//在pose graph下的位姿
	Eigen::Matrix3d R_w_i;
	Eigen::Vector3d origin_vio_T;//原始的vio坐标系下的位姿（由于在多个轨迹下可能会更新）		
	Eigen::Matrix3d origin_vio_R;
	cv::Mat image;//这是用于计算回环检测的图片，可以用time surface也可以用Eent map
	cv::Mat thumbnail;//用于可视化而已
	vector<cv::Point3f> point_3d; //关键帧中每一个地图点的世界坐标系3D点
	vector<cv::Point2f> point_2d_uv;//关键帧中每一个地图点的像素坐标
	vector<cv::Point2f> point_2d_norm;//关键帧中每一个地图点的相机坐标系下的归一化坐标
	vector<double> point_id;//关键帧中每一个地图点的ID
	vector<cv::KeyPoint> keypoints;//匹配图像中的关键点（额外通过FAST检测来提取的）
	vector<cv::KeyPoint> keypoints_norm;
	vector<cv::KeyPoint> window_keypoints;//窗口中的关键点（像素坐标系，这是之前前端提取的）
	vector<BRIEF::bitset> brief_descriptors;//描述子（基于当前匹配的图像通过FAST检测来提取的）
	vector<BRIEF::bitset> window_brief_descriptors;//窗口中的关键点window_keypoints对应的描述子（基于前端提取的特征点的）
	bool has_fast_point;
	int sequence;

	bool has_loop;
	int loop_index;
	Eigen::Matrix<double, 8, 1 > loop_info;//记录两帧之间相对位姿（x,y,z,qw,qx,qy,qz,yaw）
};

