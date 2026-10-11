#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
using namespace std;

#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include "parameters.h"
#include "utility/line_geometry.h"

//双目的情况下，定义的维护双目的图像特征

enum DPT_TYPE
{
    NO_DEPTH =0, DEPTH_MES, DEPTH_TRI, INVALID
} ;

class stereo_FeaturePerFrame
{
  public:
    stereo_FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        ori_point = point; 
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 
        cur_td = td;
        is_stereo = false;
        dpt = -1; 
        gc_succeed =false; 
    }
    void rightObservation(const Eigen::Matrix<double, 7, 1> &_point)
    {
        pointRight.x() = _point(0);
        pointRight.y() = _point(1);
        pointRight.z() = _point(2);
        ori_pointRight = pointRight; 
        uvRight.x() = _point(3);
        uvRight.y() = _point(4);
        velocityRight.x() = _point(5); 
        velocityRight.y() = _point(6); 
        is_stereo = true;

        // call getDepth()
        getDepth();
    }

    // triangulation to compute depth 
    double getDepth(); 

    Eigen::Matrix2d getOmega(); // inverse of covariance matrix
    Eigen::Matrix2d getOmegaRight(); // weighted inverse of covariance matrix

    void print(){
        printf("point: %f %f %f uv: %f %f \n", point.x(), point.y(), point.z(), uv.x(), uv.y());
    }
    double cur_td;
    bool gc_succeed; 
    Vector3d point, pointRight;
    Vector3d ori_point, ori_pointRight; 
    Vector2d uv, uvRight;
    Vector2d velocity, velocityRight;
    double z;
    bool is_used;
    double parallax;
    MatrixXd A;
    VectorXd b;
    double dep_gradient;
    bool is_stereo; 
    double dpt; 
    int feat_id; 
};

class stereo_FeaturePerId
{
  public:
    const int feature_id;
    int start_frame;
    vector<stereo_FeaturePerFrame> feature_per_frame;

    int used_num;
    bool is_outlier;
    bool is_margin;
    double estimated_depth;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    Vector3d gt_p;

    stereo_FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0),dpt_type(NO_DEPTH)
    {
    }

    int endFrame();
    DPT_TYPE dpt_type; 
};


class FeaturePerFrame//特征在每个观测图像中的信息
{
  public:
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 
        cur_td = td;
    }
    double cur_td;
    Vector3d point;
    Vector2d uv;
    Vector2d velocity;
    double z;
    bool is_used;
    double parallax;
    MatrixXd A;
    VectorXd b;
    double dep_gradient;
};

class FeaturePerId//每个特征，在不同观测图像中的信息
{
  public:
    const int feature_id;
    int start_frame;
    vector<FeaturePerFrame> feature_per_frame;

    int used_num;
    bool is_outlier;
    bool is_margin;
    double estimated_depth;
    int solve_flag; //3D点的深度是否被解出来 0 haven't solve yet; 1 solve succ; 2 solve fail;

    Vector3d gt_p;

    FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0)
    {
    }

    int endFrame();
};

// 额外定义的事件feature管理器
class Event_FeaturePerFrame
{
  public:
    Event_FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 
        cur_td = td;
    }
    double cur_td;
    Vector3d point;
    Vector2d uv;
    Vector2d velocity;
    double z;
    bool is_used;
    double parallax;
    MatrixXd A;
    VectorXd b;
    double dep_gradient;
};

class Event_FeaturePerId
{
  public:
    const int feature_id;
    int start_frame;
    vector<Event_FeaturePerFrame> Event_feature_per_frame;

    int used_num;
    bool is_outlier;
    bool is_margin;
    double estimated_depth;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    Vector3d gt_p;

    Event_FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0)
    {
    }

    int endFrame();
};

//加入线特征
class lineFeaturePerFrame
{
public:
    lineFeaturePerFrame(const Vector4d &line)
    {
        lineobs = line;//前面插入的时候为Vector4d(x_startpoint, y_startpoint, x_endpoint, y_endpoint)
    }
    lineFeaturePerFrame(const Vector8d &line)
    {
        lineobs = line.head<4>();
        lineobs_R = line.tail<4>();
    }
    Vector4d lineobs;   // (应该是存放了线对应的四个点的值)每一帧上的观测
    Vector4d lineobs_R;
    double z;
    bool is_used;
    double parallax;
    MatrixXd A;
    VectorXd b;
    double dep_gradient;
};

class lineFeaturePerId
{
public:
    const int feature_id;//线特征的feature id
    int start_frame;//线特征的起始帧

    //  feature_per_frame 是个向量容器，存着这个特征在每一帧上的观测量。
    //                    如：feature_per_frame[0]，存的是ft在start_frame上的观测值; feature_per_frame[1]存的是start_frame+1上的观测
    vector<lineFeaturePerFrame> linefeature_per_frame;

    int used_num;//被跟踪的次数（已经有多少帧看到这个特征）
    bool is_outlier;
    bool is_margin;
    bool is_triangulation;//（这个线特征）是否进行三角化了
    Vector6d line_plucker;

    Vector4d obs_init;
    Vector4d obs_j;
    Vector6d line_plk_init; // used to debug
    Vector3d ptw1;  // used to debug
    Vector3d ptw2;  // used to debug
    Eigen::Vector3d tj_;   // tij
    Eigen::Matrix3d Rj_;
    Eigen::Vector3d ti_;   // tij
    Eigen::Matrix3d Ri_;
    int removed_cnt;
    int all_obs_cnt;    // 总共观测多少次了

    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    lineFeaturePerId(int _feature_id, int _start_frame)
            : feature_id(_feature_id), start_frame(_start_frame),
              used_num(0), solve_flag(0),is_triangulation(false)
    {
        removed_cnt = 0;
        all_obs_cnt = 1;
    }

    int endFrame();
};

class FeatureManager
{
  public:
    FeatureManager(Matrix3d _Rs[]);

    void setRic(Matrix3d _ric[]);

    void clearState();

    int getFeatureCount();//统计点特征
    int stereo_getFeatureCount();
    int getEventFeatureCount();//统计事件点特征
    int getLineFeatureCount();//统计线特征

    MatrixXd getLineOrthVector(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);//获得线特征的方向相邻
    void setLineOrth(MatrixXd x, Vector3d Ps[], Matrix3d Rs[],Vector3d tic[], Matrix3d ric[]);//设置线特征的方向
    MatrixXd getLineOrthVectorInCamera();//获得camera下的线特征的方向向量
    void setLineOrthInCamera(MatrixXd x);//在camera下的线特征设置
    double reprojection_error( Vector4d obs, Matrix3d Rwc, Vector3d twc, Vector6d line_w );//线特征的重投影误差
    void removeLineOutlier(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
    

    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td);//原始版本
    // 结合线特征进行视差检测
    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const map<int, vector<pair<int, Vector4d>>> &lines, double td);
    //图像特征+事件点特征。线特征
    bool addFeatureCheckParallax( int frame_count, 
                                  const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, 
                                  const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &event, 
                                  const map<int, vector<pair<int, Vector4d>>> &lines, 
                                  double td);
    //双目
    bool stereo_addFeatureCheckParallax( int frame_count, 
                                  const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, 
                                  const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &event, 
                                  const map<int, vector<pair<int, Vector4d>>> &lines, 
                                  double td);

    void debugShow();
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);//在初始化的时候用的
    vector<pair<Vector3d, Vector3d>> getCorrespondingWithDepth(int frame_count_l, int frame_count_r);//(双目)在初始化的时候用的

    //void updateDepth(const VectorXd &x);
    void setDepth(const VectorXd &x);
    void stereo_setDepth(const VectorXd &x);
    void Event_setDepth(const VectorXd &x);//设置事件角点的深度

    void removeFailures();
    void clearDepth(const VectorXd &x);
    VectorXd getDepthVector();
    VectorXd stereo_getDepthVector();
    VectorXd Event_getDepthVector();
    void triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
    void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d);
    void triangulateStereo();//双目的时候进行三角化
    void stereo_triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);//双目未三角化的点，就采用单目的三角化
    void Event_triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
    void triangulateLine(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);//实现直线三角化函数
    void triangulateLine(double baseline);  // stereo line
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    void removeBack();
    void removeFront(int frame_count);
    void removeOutlier();
    list<FeaturePerId> feature;//点特征
    list<stereo_FeaturePerId> stereo_feature;//双目情况下的图像点特征
    list<Event_FeaturePerId> Event_feature;//事件点特征（如果加入image后，FeaturePerId就为图像的特征，Event_FeaturePerId 为事件特征）
    list<lineFeaturePerId> linefeature;//线特征
    int last_track_num;
    //双目的情况下新增的
    double last_average_parallax;//上一次的平移
    int new_feature_num;//新产生的特征点
    int long_track_num;//跟踪次数大于4

  private:
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);//处理图像特征点
    double compensatedParallax2(const stereo_FeaturePerId &it_per_id, int frame_count);//处理双目的图像点特征
    double compensatedParallax2(const Event_FeaturePerId &it_per_id, int frame_count);//处理事件特征点
    const Matrix3d *Rs;
    Matrix3d ric[NUM_OF_CAM];
};

#endif