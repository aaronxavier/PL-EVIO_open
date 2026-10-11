#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mutex>
#include <queue>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "pose_graph.h"
#include "utility/CameraPoseVisualization.h"
#include "parameters.h"
#include <cmath>
#define SKIP_FIRST_CNT 10
using namespace std;

rclcpp::Node::SharedPtr g_node;//给free function里用到的logger/now()

static inline double toSec(const builtin_interfaces::msg::Time & stamp)
{
    return rclcpp::Time(stamp).seconds();
}

queue<sensor_msgs::msg::Image::ConstSharedPtr> image_buf;//time surface图像数据
queue<sensor_msgs::msg::PointCloud::ConstSharedPtr> point_buf;////世界坐标系下的地图点云
queue<nav_msgs::msg::Odometry::ConstSharedPtr> pose_buf;//当前帧的 pose
queue<Eigen::Vector3d> odometry_buf;
std::mutex m_buf;
std::mutex m_process;
int frame_index  = 0;
int sequence = 1;
PoseGraph posegraph;//pose graph整个类
int skip_first_cnt = 0;
int SKIP_CNT;
int skip_cnt = 0;
bool load_flag = 0;
bool start_flag = 0;
double SKIP_DIS = 0;

int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
int ROW;
int COL;
int DEBUG_IMAGE;//可视化回环参数
int SAVE_LOOP_MATCH;//将所有的回环匹配过程的图像保存出来可视化
int VISUALIZE_IMU_FORWARD;
int LOOP_CLOSURE;
int FAST_RELOCALIZATION;
double TS_LK_THRESHOLD;

camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match_img;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_match_points;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_camera_pose_visual;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_key_odometrys;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_vio_path;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_latest_loop_evio;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_flight_evio;
nav_msgs::msg::Path no_loop_path;

std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
std::string VINS_RESULT_PATH;
CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
Eigen::Vector3d last_t(-100, -100, -100);
double last_image_time = -1;

using EventQueue = std::queue<dvs_msgs::msg::EventArray>;
EventQueue events_buf;//event_corner_buf


void new_sequence()//开始一个新的图像序列sequence（地图合并功能）
{
    printf("new sequence\n");
    sequence++;
    printf("sequence cnt %d \n", sequence);
    if (sequence > 5)
    {
        ROS_WARN("only support 5 sequences since it's boring to copy code for more sequences.");
        ROS_BREAK();
    }
    posegraph.posegraph_visualization->reset();
    posegraph.publish();
    m_buf.lock();
    while(!image_buf.empty())
        image_buf.pop();
    while(!point_buf.empty())
        point_buf.pop();
    while(!pose_buf.empty())
        pose_buf.pop();
    while(!odometry_buf.empty())
        odometry_buf.pop();
    while(!events_buf.empty())//补充将event-corner feature buf清空
        events_buf.pop();
    m_buf.unlock();
}

void eventsfeatureCallback(const dvs_msgs::msg::EventArray &event_feature_msg){//回调获得event-corner feature，并放入buf中
    m_buf.lock();
    events_buf.push(event_feature_msg);//放入buf中
    m_buf.unlock();
}

void loop_callback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg)//回环检测的图像调回函数
{//根据时间戳检验是否是新的图像序列
    if(!LOOP_CLOSURE)// 不检测回环，原图也没有意义
        return;
    m_buf.lock();
    image_buf.push(image_msg);//将图像（time surface）放入buf中
    m_buf.unlock();
    //printf(" image time %f \n", toSec(image_msg->header.stamp));

    // detect unstable camera stream
    if (last_image_time == -1)
        last_image_time = toSec(image_msg->header.stamp);
    else if (toSec(image_msg->header.stamp) - last_image_time > 1.0 || toSec(image_msg->header.stamp) < last_image_time) // 检查时间戳是否错乱以及延时过大
    {
        // ROS_WARN("image discontinue! detect a new sequence!");
        // new_sequence(); // 如果发生了就新建一个序列

        ROS_WARN("the event camera has stoped, waiting for the restart!");
        //改为重启。此处不需要发布，只需要等待feature tracker
    }
    last_image_time = toSec(image_msg->header.stamp);
}


// 地图点云回调函数，把point_msg放入point_buf
void point_callback(const sensor_msgs::msg::PointCloud::ConstSharedPtr point_msg)// VIO中KF关于地图点的信息（关键帧中的地图点）
{//地图点云回调函数
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    point_buf.push(point_msg);//把关键帧中的地图点，放入buf中
    m_buf.unlock();
}

// 图像帧位姿回调函数，把pose_msg放入pose_buf
void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)//VIO结点KF的信息（关键帧的pose）
{//关键帧位姿回调函数
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
}


bool davis240c_flag=false;
//为了可视化，跟回环无关
void imu_forward_callback(const nav_msgs::msg::Odometry::ConstSharedPtr forward_msg)//发布基于IMU的更高频率的VIO的位姿
{//IMU前向递推的回调函数，从IMU预积分的位姿得到IMU位姿和cam位姿。得到低延时和高频率的结果
    if (VISUALIZE_IMU_FORWARD)
    {
        Vector3d vio_t(forward_msg->pose.pose.position.x, forward_msg->pose.pose.position.y, forward_msg->pose.pose.position.z);
        Quaterniond vio_q;
        vio_q.w() = forward_msg->pose.pose.orientation.w;
        vio_q.x() = forward_msg->pose.pose.orientation.x;
        vio_q.y() = forward_msg->pose.pose.orientation.y;
        vio_q.z() = forward_msg->pose.pose.orientation.z;
        //获取当前最新的结果

        vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
        vio_q = posegraph.w_r_vio *  vio_q;

        vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
        vio_q = posegraph.r_drift * vio_q;

        //发布odometry
        nav_msgs::msg::Odometry odometry;
        odometry.header = forward_msg->header;
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = vio_t.x();
        odometry.pose.pose.position.y = vio_t.y();
        odometry.pose.pose.position.z = vio_t.z();
        odometry.pose.pose.orientation.x = vio_q.x();
        odometry.pose.pose.orientation.y = vio_q.y();
        odometry.pose.pose.orientation.z = vio_q.z();
        odometry.pose.pose.orientation.w = vio_q.w();
        odometry.twist.twist.linear.x = forward_msg->twist.twist.linear.x;
        odometry.twist.twist.linear.y = forward_msg->twist.twist.linear.y;
        odometry.twist.twist.linear.z = forward_msg->twist.twist.linear.z;
        pub_flight_evio->publish(odometry);

        Vector3d vio_t_cam;
        Quaterniond vio_q_cam;
        vio_t_cam = vio_t + vio_q * tic;
        vio_q_cam = vio_q * qic;

        cameraposevisual.reset();
        cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
        cameraposevisual.publish_by(pub_camera_pose_visual, forward_msg->header);

        //此处定义一个发布者，发布vio_t_cam以及vio_q_cam
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = forward_msg->header;
        pose_stamped.pose.position.x=vio_t_cam.x();
        pose_stamped.pose.position.y=vio_t_cam.y();
        pose_stamped.pose.position.z=vio_t_cam.z();
        pose_stamped.pose.orientation.w=vio_q_cam.w();
        pose_stamped.pose.orientation.x=vio_q_cam.x();
        pose_stamped.pose.orientation.y=vio_q_cam.y();
        pose_stamped.pose.orientation.z=vio_q_cam.z();
        //给一个flag，当z的绝对值少于1的时候，开始可以发布
        if(!davis240c_flag && abs(pose_stamped.pose.position.z)<=1.0 && abs(pose_stamped.pose.position.x)<=1.0 && abs(pose_stamped.pose.position.y)<=1.0){
            davis240c_flag=true;
        }
        if (davis240c_flag)
            pub_latest_loop_evio->publish(pose_stamped);//以足够快的速率来发布（以geometry_msgs::PoseStamped格式）
    }
}


void relo_relative_pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)// 利用VIO进行重定位的结果进行修正
{//从estimator中获取了回环重定位的结果
//重定位回调函数，将重定位帧的相对位姿放入loop_info中
// 其中的updateKeyFrameLoop()进行回环更新
    Vector3d relative_t = Vector3d(pose_msg->pose.pose.position.x,
                                   pose_msg->pose.pose.position.y,
                                   pose_msg->pose.pose.position.z);
    Quaterniond relative_q;
    relative_q.w() = pose_msg->pose.pose.orientation.w;
    relative_q.x() = pose_msg->pose.pose.orientation.x;
    relative_q.y() = pose_msg->pose.pose.orientation.y;
    relative_q.z() = pose_msg->pose.pose.orientation.z;
    double relative_yaw = pose_msg->twist.twist.linear.x;
    int index = pose_msg->twist.twist.linear.y;
    Eigen::Matrix<double, 8, 1 > loop_info;
    loop_info << relative_t.x(), relative_t.y(), relative_t.z(),
                 relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                 relative_yaw;//把回环检测的R与T放入loop_info
    posegraph.updateKeyFrameLoop(index, loop_info);//进行回环更新

}


// 接受的VIO滑窗中最新的位姿，不一定是KF
// 这里了可视化相关的内容
void vio_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)//，获取IMU与Cam的pose
{
    Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;

    vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
    vio_q = posegraph.w_r_vio *  vio_q;

    vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
    vio_q = posegraph.r_drift * vio_q;

    Vector3d vio_t_cam;
    Quaterniond vio_q_cam;
    vio_t_cam = vio_t + vio_q * tic;//转换到camera下
    vio_q_cam = vio_q * qic;

    if (!VISUALIZE_IMU_FORWARD)
    {
        cameraposevisual.reset();
        cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
        cameraposevisual.publish_by(pub_camera_pose_visual, pose_msg->header);
    }

    odometry_buf.push(vio_t_cam);
    if (odometry_buf.size() > 10)
    {
        odometry_buf.pop();
    }

    visualization_msgs::msg::Marker key_odometrys;//将最新10frame的结果显示出来
    key_odometrys.header = pose_msg->header;
    key_odometrys.header.frame_id = "world";
    key_odometrys.ns = "key_odometrys";
    key_odometrys.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    key_odometrys.action = visualization_msgs::msg::Marker::ADD;
    key_odometrys.pose.orientation.w = 1.0;
    key_odometrys.lifetime = builtin_interfaces::msg::Duration();

    key_odometrys.id = 0; //key_odometrys_id++;
    key_odometrys.scale.x = 0.1;
    key_odometrys.scale.y = 0.1;
    key_odometrys.scale.z = 0.1;
    key_odometrys.color.r = 1.0;
    key_odometrys.color.a = 1.0;

    for (unsigned int i = 0; i < odometry_buf.size(); i++)
    {
        geometry_msgs::msg::Point pose_marker;
        Vector3d vio_t;
        vio_t = odometry_buf.front();
        odometry_buf.pop();
        pose_marker.x = vio_t.x();
        pose_marker.y = vio_t.y();
        pose_marker.z = vio_t.z();
        key_odometrys.points.push_back(pose_marker);
        odometry_buf.push(vio_t);
    }
    pub_key_odometrys->publish(key_odometrys);

    if (!LOOP_CLOSURE)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = pose_msg->header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = vio_t.x();
        pose_stamped.pose.position.y = vio_t.y();
        pose_stamped.pose.position.z = vio_t.z();
        no_loop_path.header = pose_msg->header;
        no_loop_path.header.frame_id = "world";
        no_loop_path.poses.push_back(pose_stamped);
        pub_vio_path->publish(no_loop_path);//没有回环的轨迹发布出来
    }
}

void extrinsic_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)//实时更新外参
{
    m_process.lock();
    tic = Vector3d(pose_msg->pose.pose.position.x,
                   pose_msg->pose.pose.position.y,
                   pose_msg->pose.pose.position.z);
    qic = Quaterniond(pose_msg->pose.pose.orientation.w,
                      pose_msg->pose.pose.orientation.x,
                      pose_msg->pose.pose.orientation.y,
                      pose_msg->pose.pose.orientation.z).toRotationMatrix();
    m_process.unlock();
}

void process()// 回环检测主要处理函数，也是需要重点关注改进的地方
{
    if (!LOOP_CLOSURE) // 不检测回环就啥都不干
        return;
    while (true)
    {        // 三个参数图像、点云、VIO（关键帧/窗口）位姿
        sensor_msgs::msg::Image::ConstSharedPtr image_msg = nullptr;//图像信息（tracking图像的信息，目前先采用time surface）
        sensor_msgs::msg::PointCloud::ConstSharedPtr point_msg = nullptr;//关键帧中的地图点（里面包括了，关键帧中，每一个地图点的世界坐标系3D点，相机坐标系下的归一化坐标以及像素坐标+特征点的id）
        nav_msgs::msg::Odometry::ConstSharedPtr pose_msg = nullptr;//关键帧的pose

        // find out the messages with same time stamp（得到具有相同时间戳的pose_msg、image_msg、point_msg）
        m_buf.lock();
        // 做一个时间戳对齐，涉及到原图，KF位姿以及KF对应地图点
        if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            if (toSec(image_buf.front()->header.stamp) > toSec(pose_buf.front()->header.stamp))// 原图时间戳比另外两个晚，只能扔掉早于第一个原图的消息
            {
                pose_buf.pop();
                printf("throw pose at beginning\n");
            }
            else if (toSec(image_buf.front()->header.stamp) > toSec(point_buf.front()->header.stamp))
            {
                point_buf.pop();
                printf("throw point at beginning\n");
            }
            // 上面确保了image_buf <= point_buf && image_buf <= pose_buf
             // 下面根据pose时间找时间戳同步的原图和地图点
            else if (toSec(image_buf.back()->header.stamp) >= toSec(pose_buf.front()->header.stamp)
                && toSec(point_buf.back()->header.stamp) >= toSec(pose_buf.front()->header.stamp))
            {
                pose_msg = pose_buf.front();// 取出来pose
                pose_buf.pop();
                while (!pose_buf.empty()) // 清空所有的pose，回环的帧率慢一些没关系
                    pose_buf.pop();
                while (toSec(image_buf.front()->header.stamp) < toSec(pose_msg->header.stamp))
                    image_buf.pop();
                image_msg = image_buf.front(); // 找到对应pose的原图
                image_buf.pop();

                while (toSec(point_buf.front()->header.stamp) < toSec(pose_msg->header.stamp))
                    point_buf.pop();
                point_msg = point_buf.front();// 找到对应的地图点
                point_buf.pop();
            }
        }
        m_buf.unlock();
        // 至此取出了时间戳同步的原图，KF和地图点信息

        // 构建pose_graph中用到的关键帧，然后每隔SKIP_CNT，将将距上一关键帧距离（平移向量的模）超过SKIP_DIS的图像创建为关键帧。
        // 其中，最核心的函数是KeyFrame类以及addKeyFrame（）函数。
        if (pose_msg != nullptr)
        {
            // skip fisrt few
            if (skip_first_cnt < SKIP_FIRST_CNT)//跳过最开始的SKIP_FIRST_CNT(10)帧（剔除最开始的SKIP_FIRST_CNT帧）
            {
                skip_first_cnt++;
                continue;
            }

            if (skip_cnt < SKIP_CNT) // 降频，每隔SKIP_CNT帧处理一次(roslaunch参数设置为0)
            {
                skip_cnt++;
                continue;
            }
            else
            {
                skip_cnt = 0;
            }

            cv_bridge::CvImageConstPtr ptr;// 通过cvbridge得到opencv格式的图像
            if (image_msg->encoding == "8UC1")
            {
                sensor_msgs::msg::Image img;
                img.header = image_msg->header;
                img.height = image_msg->height;
                img.width = image_msg->width;
                img.is_bigendian = image_msg->is_bigendian;
                img.step = image_msg->step;
                img.data = image_msg->data;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            }
            else
                ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);

            cv::Mat image = ptr->image;//这个为tracking的图像，也就是回环匹配的图像

            //加一个判决，以防图片尺寸变化
            if(!( image.rows==ROW &&  image.cols==COL)){//对付image有时大小不一致的问题
                resize(image,image, cv::Size(COL, ROW));
            }

            // build keyframe
            // 得到KF的位姿，转成eigen格式
            Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                                  pose_msg->pose.pose.position.y,
                                  pose_msg->pose.pose.position.z);//关键帧的T
            Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                                     pose_msg->pose.pose.orientation.x,
                                     pose_msg->pose.pose.orientation.y,
                                     pose_msg->pose.pose.orientation.z).toRotationMatrix();//关键帧的R
            if((T - last_t).norm() > SKIP_DIS)  // 要求KF相隔必要的平移距离（roslaunch中设置为0）
            {//将距上一关键帧距离（平移向量的模）超过SKIP_DIS的图像创建为关键帧
                vector<cv::Point3f> point_3d; //// VIO世界坐标系下的地图点坐标
                vector<cv::Point2f> point_2d_uv; //// 归一化相机坐标系的坐标
                vector<cv::Point2f> point_2d_normal;//// 像素坐标
                vector<double> point_id;// // 地图点的idx

                for (unsigned int i = 0; i < point_msg->points.size(); i++)// 遍历所有的地图点
                {
                    cv::Point3f p_3d;
                    p_3d.x = point_msg->points[i].x;
                    p_3d.y = point_msg->points[i].y;
                    p_3d.z = point_msg->points[i].z;
                    point_3d.push_back(p_3d);

                    cv::Point2f p_2d_uv, p_2d_normal;
                    double p_id;
                    p_2d_normal.x = point_msg->channels[i].values[0];
                    p_2d_normal.y = point_msg->channels[i].values[1];
                    p_2d_uv.x = point_msg->channels[i].values[2];
                    p_2d_uv.y = point_msg->channels[i].values[3];
                    p_id = point_msg->channels[i].values[4];
                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                    point_id.push_back(p_id);
                }

                // 创建为关键帧类型并加入到posegraph中(通过函数addKeyFrame)
                KeyFrame* keyframe = new KeyFrame(toSec(pose_msg->header.stamp), frame_index, T, R, image,
                                   point_3d, point_2d_uv, point_2d_normal, point_id, sequence);   // 创建回环检测节点的KF
                //时间，第几个窗口，当前关键帧的T与R，跟踪的图片，当前关键帧的地图点（每一个地图点的世界坐标系3D点，相机坐标系下的归一化坐标以及像素坐标+特征点的id），序列

                m_process.lock();
                start_flag = 1;
                //在posegraph中添加关键帧，flag_detect_loop=1回环检测
                posegraph.addKeyFrame(keyframe, 1); // 回环检测核心入口函数（传入的为回环的关键帧）
                m_process.unlock();
                frame_index++;
                last_t = T;
            }
        }

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);//休眠5ms
    }
}


void process_event()// 回环检测主要处理函数，也是需要重点关注改进的地方
{
    if (!LOOP_CLOSURE) // 不检测回环就啥都不干
        return;
    while (true)
    {        // 三个参数图像、点云、VIO（关键帧/窗口）位姿
        sensor_msgs::msg::Image::ConstSharedPtr image_msg = nullptr;//图像信息（tracking图像的信息，目前先采用time surface）
        sensor_msgs::msg::PointCloud::ConstSharedPtr point_msg = nullptr;//关键帧中的地图点（里面包括了，关键帧中，每一个地图点的世界坐标系3D点，相机坐标系下的归一化坐标以及像素坐标+特征点的id）
        nav_msgs::msg::Odometry::ConstSharedPtr pose_msg = nullptr;//关键帧的pose
        dvs_msgs::msg::EventArray event_corner_msg;//存放来自events_buf中的事件特征点（处理过程跟image_msg或者image_buf一样）

        // find out the messages with same time stamp（得到具有相同时间戳的pose_msg、image_msg、point_msg）
        m_buf.lock();
        // 做一个时间戳对齐，涉及到原图，KF位姿以及KF对应地图点
        if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            if (toSec(image_buf.front()->header.stamp) > toSec(pose_buf.front()->header.stamp))// 原图时间戳比另外两个晚，只能扔掉早于第一个原图的消息
            {
                pose_buf.pop();
                printf("throw pose at beginning\n");
            }
            else if (toSec(image_buf.front()->header.stamp) > toSec(point_buf.front()->header.stamp))//image发生在点云的后面
            {
                point_buf.pop();
                printf("throw point at beginning\n");//把前面的点去掉
            }
            // 上面确保了image_buf <= point_buf && image_buf <= pose_buf
             // 下面根据pose时间找时间戳同步的原图和地图点
            else if (toSec(image_buf.back()->header.stamp) >= toSec(pose_buf.front()->header.stamp)
                && toSec(point_buf.back()->header.stamp) >= toSec(pose_buf.front()->header.stamp))
            {
                pose_msg = pose_buf.front();// 取出来pose
                pose_buf.pop();
                while (!pose_buf.empty()) // 清空所有的pose，回环的帧率慢一些没关系
                    pose_buf.pop();
                while (toSec(image_buf.front()->header.stamp) < toSec(pose_msg->header.stamp))//清空在此之前的imagebuf与event buf
                {
                    image_buf.pop();
                    events_buf.pop();
                }
                image_msg = image_buf.front(); // 找到对应pose的原图
                image_buf.pop();
                while(toSec(events_buf.front().header.stamp)<toSec(image_msg->header.stamp))//当事件特征点的最前一个时间少于image_msg时，就是在前面发生的，可以丢掉
                    events_buf.pop();
                event_corner_msg=events_buf.front();//找到对应pose的event feature
                events_buf.pop();

                while (toSec(point_buf.front()->header.stamp) < toSec(pose_msg->header.stamp))
                    point_buf.pop();
                point_msg = point_buf.front();// 找到对应的地图点
                point_buf.pop();
            }
        }
        m_buf.unlock();
        // 至此取出了时间戳同步的原图，KF和地图点信息

        // 构建pose_graph中用到的关键帧，然后每隔SKIP_CNT，将将距上一关键帧距离（平移向量的模）超过SKIP_DIS的图像创建为关键帧。
        // 其中，最核心的函数是KeyFrame类以及addKeyFrame（）函数。
        if (pose_msg != nullptr)
        {
            // skip fisrt few
            if (skip_first_cnt < SKIP_FIRST_CNT)//跳过最开始的SKIP_FIRST_CNT(10)帧（剔除最开始的SKIP_FIRST_CNT帧）
            {
                skip_first_cnt++;
                continue;
            }

            if (skip_cnt < SKIP_CNT) // 降频，每隔SKIP_CNT帧处理一次(roslaunch参数设置为0)
            {
                skip_cnt++;
                continue;
            }
            else
            {
                skip_cnt = 0;
            }

            cv_bridge::CvImageConstPtr ptr;// 通过cvbridge得到opencv格式的图像
            if (image_msg->encoding == "8UC1")
            {
                sensor_msgs::msg::Image img;
                img.header = image_msg->header;
                img.height = image_msg->height;
                img.width = image_msg->width;
                img.is_bigendian = image_msg->is_bigendian;
                img.step = image_msg->step;
                img.data = image_msg->data;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            }
            else
                ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);

            cv::Mat image = ptr->image;//这个为tracking的图像，也就是回环匹配的图像

            // build keyframe
            // 得到KF的位姿，转成eigen格式
            Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                                  pose_msg->pose.pose.position.y,
                                  pose_msg->pose.pose.position.z);//关键帧的T
            Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                                     pose_msg->pose.pose.orientation.x,
                                     pose_msg->pose.pose.orientation.y,
                                     pose_msg->pose.pose.orientation.z).toRotationMatrix();//关键帧的R

            if((T - last_t).norm() > SKIP_DIS)  // 要求KF相隔必要的平移距离（roslaunch中设置为0）
            {//将距上一关键帧距离（平移向量的模）超过SKIP_DIS的图像创建为关键帧
                vector<cv::Point3f> point_3d; //// VIO世界坐标系下的地图点坐标
                vector<cv::Point2f> point_2d_uv; //// 归一化相机坐标系的坐标
                vector<cv::Point2f> point_2d_normal;//// 像素坐标
                vector<double> point_id;// // 地图点的idx

                for (unsigned int i = 0; i < point_msg->points.size(); i++)// 遍历所有的地图点
                {
                    cv::Point3f p_3d;
                    p_3d.x = point_msg->points[i].x;
                    p_3d.y = point_msg->points[i].y;
                    p_3d.z = point_msg->points[i].z;
                    point_3d.push_back(p_3d);

                    cv::Point2f p_2d_uv, p_2d_normal;
                    double p_id;
                    p_2d_normal.x = point_msg->channels[i].values[0];
                    p_2d_normal.y = point_msg->channels[i].values[1];
                    p_2d_uv.x = point_msg->channels[i].values[2];
                    p_2d_uv.y = point_msg->channels[i].values[3];
                    p_id = point_msg->channels[i].values[4];
                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                    point_id.push_back(p_id);
                }

                // 创建为关键帧类型并加入到posegraph中(通过函数addKeyFrame)
                KeyFrame* keyframe = new KeyFrame(toSec(pose_msg->header.stamp), frame_index, T, R, image,
                                   point_3d, point_2d_uv, point_2d_normal, point_id, sequence,event_corner_msg);   // 创建回环检测节点的KF
                //时间，第几个窗口，当前关键帧的T与R，跟踪的图片，当前关键帧的地图点（每一个地图点的世界坐标系3D点，相机坐标系下的归一化坐标以及像素坐标+特征点的id），序列

                m_process.lock();
                start_flag = 1;
                //在posegraph中添加关键帧，flag_detect_loop=1回环检测
                posegraph.addKeyFrame(keyframe, 1); // 回环检测核心入口函数（传入的为回环的关键帧）将flag_detect_loop=1即设置回环检测。
                m_process.unlock();
                frame_index++;
                last_t = T;
            }
        }

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);//休眠5ms
    }
}


void command()//接受县官的键盘指令
{
    ROS_INFO("press the s button to save the pose graph");
    if (!LOOP_CLOSURE)
        return;
    while(1)
    {
        char c = getchar();
        if (c == 's') // s就是存储地图（注意，要按下s才会保存pose graph）
        {
            m_process.lock();
            posegraph.savePoseGraph();
            m_process.unlock();
            printf("save pose graph finish\nyou can set 'load_previous_pose_graph' to 1 in the config file to reuse it next time\n");
        }

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("pose_graph");
    posegraph.registerPub(g_node);//定义一系列的发布者

    // read param（读取参数）
    g_node->declare_parameter<int>("visualization_shift_x", VISUALIZATION_SHIFT_X);
    g_node->get_parameter("visualization_shift_x", VISUALIZATION_SHIFT_X); // 这两个shift基本都是0
    g_node->declare_parameter<int>("visualization_shift_y", VISUALIZATION_SHIFT_Y);
    g_node->get_parameter("visualization_shift_y", VISUALIZATION_SHIFT_Y);
    g_node->declare_parameter<int>("skip_cnt", SKIP_CNT);
    g_node->get_parameter("skip_cnt", SKIP_CNT);// 跳过前SKIP_CNT帧  （传入设置了0）
    g_node->declare_parameter<double>("skip_dis", SKIP_DIS);
    g_node->get_parameter("skip_dis", SKIP_DIS); // 两帧距离门限（也是设置了0）
    std::string config_file;
    g_node->declare_parameter<std::string>("config_file", config_file);
    g_node->get_parameter("config_file", config_file);//获取参数
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

// 可视化的参数
    double camera_visual_size = fsSettings["visualize_camera_size"];
    cameraposevisual.setScale(camera_visual_size);
    cameraposevisual.setLineWidth(camera_visual_size / 10.0);

//后面提取event feature需要用
TS_LK_THRESHOLD= fsSettings["TS_LK_threshold"];

// 是否进行回环检测的标识
    LOOP_CLOSURE = fsSettings["loop_closure"];//开启回环的参数
    std::string LOOP_TOPIC;//回环检测用的图像的topic
    int LOAD_PREVIOUS_POSE_GRAPH;
    if (LOOP_CLOSURE)//如果需要进行回环检测则读取词典和BRIEF描述子的模板文件，同时读取config中的其他参数、设置带回环的结果输出路径。
    {
        // 图片分辨率
        ROW = fsSettings["image_height"];
        COL = fsSettings["image_width"];

        // / 读取字典
        std::string pkg_path = ament_index_cpp::get_package_share_directory("pose_graph");
        string vocabulary_file = pkg_path + "/support_files/brief_k10L6.bin"; // 训练好的二进制词袋的路径（应该是需要有这个才有回环）
        cout << "vocabulary_file" << vocabulary_file << endl;
        posegraph.loadVocabulary(vocabulary_file); // 加载二进制词袋

        // 读取BRIEF描述子的模板文件
        BRIEF_PATTERN_FILE = pkg_path + "/support_files/brief_pattern.yml"; // 计算描述子pattern的文件
        cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << endl;
        m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str());// 和前面一样，生成一个相机模型

        fsSettings["loop_closure_topic"] >> LOOP_TOPIC;    // 回环检测基于time surface map(或者读入event，然后用事件累积的图？)
        fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
        fsSettings["output_path"] >> VINS_RESULT_PATH;//把结果输出
        fsSettings["save_image"] >> DEBUG_IMAGE;//这个应该是可视化回环的参数
        fsSettings["save_loop_match"]>>SAVE_LOOP_MATCH;//可视化所有回环检测的图像

        // create folder if not exists
        FileSystemHelper::createDirectoryIfNotExists(POSE_GRAPH_SAVE_PATH.c_str());
        FileSystemHelper::createDirectoryIfNotExists(VINS_RESULT_PATH.c_str());

        VISUALIZE_IMU_FORWARD = fsSettings["visualize_imu_forward"];// 可视化是否使用imu进行前推
        LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"]; // 是否加载已有地图
        FAST_RELOCALIZATION = fsSettings["fast_relocalization"];// 是否快速重定位，这个和VIO结点有交互

        //下面是用于记录有了回环之后的结果的
        VINS_RESULT_PATH = VINS_RESULT_PATH + "/pl_evio_result_loop.txt";
        std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
        fout.close();

        std::ofstream fout1(VINS_RESULT_PATH, std::ios::app);
            fout.setf(std::ios::fixed, std::ios::floatfield);
            fout << "timestamp" << ",";
            fout << "tx" << ","
                    << "ty" << ","
                    << "tz" << ","
                    << "qx" << ","
                    << "qy" << ","
                    << "qz" << ","
                    << "qw" << ","<< std::endl;
        fout1.close();
        fsSettings.release();

        // 加载先前的位姿图
        if (LOAD_PREVIOUS_POSE_GRAPH)//如果有previous graph
        {
            printf("load pose graph\n");
            m_process.lock();
            posegraph.loadPoseGraph();//加载先前位姿图
            m_process.unlock();
            printf("load pose graph finish\n");
            load_flag = 1;
        }
        else
        {
            printf("no previous pose graph\n");
            load_flag = 1;
        }
    }

    fsSettings.release();

// 订阅话题topic并执行各自回调函数
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(2000));
    auto sub_imu_forward = g_node->create_subscription<nav_msgs::msg::Odometry>("/evio_estimator/imu_propagate", qos, imu_forward_callback);//这个就是基于IMU以更高频率发布的VIO消息
    auto sub_vio = g_node->create_subscription<nav_msgs::msg::Odometry>("/evio_estimator/odometry", qos, vio_callback);//获取当前的VIO的结果（用于可视化0
    auto sub_image = g_node->create_subscription<sensor_msgs::msg::Image>(LOOP_TOPIC, rclcpp::QoS(rclcpp::KeepLast(2000)).best_effort(), loop_callback);
    auto sub_pose = g_node->create_subscription<nav_msgs::msg::Odometry>("/evio_estimator/keyframe_pose", qos, pose_callback);//关键点的pose
    auto sub_extrinsic = g_node->create_subscription<nav_msgs::msg::Odometry>("/evio_estimator/extrinsic", qos, extrinsic_callback);//IMU与camera的外参
    auto sub_point = g_node->create_subscription<sensor_msgs::msg::PointCloud>("/evio_estimator/keyframe_point", qos, point_callback);//关键点（关键帧中的地图点）
    auto sub_relo_relative_pose = g_node->create_subscription<nav_msgs::msg::Odometry>("/evio_estimator/relo_relative_pose", qos, relo_relative_pose_callback);//重定位的结果

    pub_latest_loop_evio = g_node->create_publisher<geometry_msgs::msg::PoseStamped>("~/imu_evio_loop", 1000);//发布最新的evio——imu-loop的结果
    pub_flight_evio = g_node->create_publisher<nav_msgs::msg::Odometry>("~/evio_odometry", 1000);//发布odometry（给飞机用）

    auto sub_event_feature = g_node->create_subscription<dvs_msgs::msg::EventArray>(
        "/feature_tracker/event_feature_loop", qos,
        [](const dvs_msgs::msg::EventArray::SharedPtr msg) { eventsfeatureCallback(*msg); });//获取event-corner feature
// 发布的话题topic
    pub_match_img = g_node->create_publisher<sensor_msgs::msg::Image>("~/match_image", 1000);
    pub_camera_pose_visual = g_node->create_publisher<visualization_msgs::msg::MarkerArray>("~/camera_pose_visual", 1000);
    pub_key_odometrys = g_node->create_publisher<visualization_msgs::msg::Marker>("~/key_odometrys", 1000);
    pub_vio_path = g_node->create_publisher<nav_msgs::msg::Path>("~/no_loop_path", 1000);
    pub_match_points = g_node->create_publisher<sensor_msgs::msg::PointCloud>("~/match_points", 100);//发布回环

// 创建两个线程，process 和 command
    std::thread measurement_process= std::thread(process);//回环检测最主要处理的函数（pose graph主线程）
    // std::thread measurement_process= std::thread(process_event);
    // std::thread keyboard_command_process = std::thread(command);/// 接受相关键盘指令 (关键：按下s才会保存pose graph)


    rclcpp::spin(g_node);
    rclcpp::shutdown();

    return 0;
}
