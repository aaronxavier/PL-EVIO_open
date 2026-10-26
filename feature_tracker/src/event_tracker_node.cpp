#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>

#include "feature_tracker.h"
#include "utility/visualization.h"

#include <queue>
#include <thread>
#include <mutex>
#include <tuple>

#define SHOW_UNDISTORTION 0
// Max gap between event packets before declaring a stream discontinuity. Upstream used 1.0 s,
// which fires spuriously when the camera is still (few events); a true stall is still caught.
static const double EVENT_GAP_RESET_S = 5.0;

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::msg::Image::ConstSharedPtr> img_buf;

rclcpp::Node::SharedPtr g_node;//给free function里用到的ros::Time::now()/logger

FeatureTracker trackerData[NUM_OF_CAM];//定义了跟踪类（每个相机都定义了其跟踪的对象）
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;

evio::ArcStarDetector detector_666 = evio::ArcStarDetector();//额外进行声明检测器
using EventQueue = std::queue<dvs_msgs::msg::EventArray>;
EventQueue events_buf;
std::mutex m_buf;//相当于加了一个锁（互斥锁）。每次调用消息前，先锁上，把消息放到buf里面后再解锁
std::mutex m_buf_feature;
bool detector_666_flag=false;


void pubTrackImage(const cv::Mat &imgTrack, const cv::Mat &imgTrack_two, const cv::Mat &imgTrack_two_line, const cv::Mat &time_surface_map, const double t)
{//发布跟踪的image
    std_msgs::msg::Header header;
    header.frame_id = "world";
    header.stamp = secondsToRosTime(t);

    if(!imgTrack.empty())
    {
        auto imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
        pub_match->publish(*imgTrackMsg);//可视化单帧的跟踪效果

        // trackerData[0].save_event_tracking(imgTrack,t);
    }

    if(!imgTrack_two.empty())
    {
        auto imgTrackMsg_two = cv_bridge::CvImage(header, "bgr8", imgTrack_two).toImageMsg();
        pub_match_two->publish(*imgTrackMsg_two);//可视化两帧的跟踪效果
    }

    if(!imgTrack_two_line.empty())
    {
        auto imgTrackMsg_two_line = cv_bridge::CvImage(header, "bgr8", imgTrack_two_line).toImageMsg();
        pub_match_two_line->publish(*imgTrackMsg_two_line);//可视化两帧的线特征匹配的效果

        // trackerData[0].save_line_tracking(imgTrack_two_line,t);
    }

    if(!time_surface_map.empty())
    {
        auto TimeSurfaceImg = cv_bridge::CvImage(header, "mono8", time_surface_map).toImageMsg();
        pub_time_surface->publish(*TimeSurfaceImg);//可视化time surface
    }

    //保存图片
    // trackerData[0].save_event_tracking(imgTrack,t);
    // trackerData[0].save_line_tracking(imgTrack_two_line,t);
}


dvs_msgs::msg::EventArray corner_msg;//要变为全局变量
void multi_thread_feature(std::vector<dvs_msgs::msg::Event> e, int beginIndex, int length);//函数声明
void sync_process_event_corner_multi_thread(){//给回环发布event-corner用的
    while(1)
    {
        // TicToc t_this;
        if(!detector_666_flag){
            detector_666_flag=true;
            detector_666.init(COL, ROW);
        }

        // m_buf.lock();
        if(!(events_buf.empty())){
            corner_msg.header = events_buf.front().header;
            corner_msg.width = events_buf.front().width;
            corner_msg.height = events_buf.front().height;

            // 这段若单线程处理会很耗时间，改为4线程处理
            if(!corner_msg.events.empty()){
                corner_msg.events.clear();//进程开始前先把之前的清空一下
            }

            int threadCount = Num_of_thread;//dvxplorer四个线程，davis346 2个就够了吧
            std::thread threads_feature[threadCount];
            for (int i = 0; i < threadCount; i++)
            {
                //为每个线程分配任务
                int beginIndex = i*events_buf.front().events.size()/threadCount;
                int length=events_buf.front().events.size()/threadCount;
                // auto param = new std::tuple<std::vector<dvs_msgs::msg::Event>, int, int>(events_buf.front().events, beginIndex, length);
                threads_feature[i] = std::thread(multi_thread_feature,events_buf.front().events,beginIndex,length);
            }
            //等待所有线程结束
            for(auto& thread_f:threads_feature){
                if(thread_f.joinable())
                    thread_f.join();
            }

            // ROS_INFO("corner size %d,event size%d,Percetange of corners: %.1f%%",corner_msg.events.size(), events_buf.front().events.size(),(double(corner_msg.events.size())/events_buf.front().events.size())*100);
            // if(t_this.toc()>15)
            //     ROS_INFO("whole time costs of extracting the feature is: %f ms", t_this.toc());


            // if(PUB_THIS_FRAME){//只有当给后端喂数据的时候才会发给回环
                corner_pub->publish(corner_msg);//发布出去
                // pubLoopImage(detector_666.SAE_toTimeSurface_withoutP(corner_msg.header.stamp.toSec()),corner_msg.header.stamp.toSec());//用作回环检测的图像一起发布出去
                pubLoopImage(detector_666.SAE_toTimeSurface_withoutP_multi_thread(rclcpp::Time(corner_msg.header.stamp).seconds()),rclcpp::Time(corner_msg.header.stamp).seconds());//用作回环检测的图像一起发布出去
            // }

            events_buf.pop();//删除
            // ROS_WARN("the number of the corner-event buf for loop is:%d",events_buf.size());

            // if(t_this.toc()>15)
            //     ROS_INFO("whole time costs of sync_process_event_corner is: %f ms", t_this.toc());
        }
        //  m_buf.unlock();

        std::chrono::milliseconds dura(2);//等待2ms
        std::this_thread::sleep_for(dura);
    }
}
//工作线程
void multi_thread_feature(std::vector<dvs_msgs::msg::Event> e, int beginIndex, int length)
{
    for(int i=beginIndex;i<beginIndex+length;i++){
        if(detector_666.isFeature(e[i].ts, e[i].x, e[i].y, e[i].polarity)){//检测是否角点并且push进去corner_msg中
            m_buf_feature.lock();
            corner_msg.events.push_back(e[i]);
            m_buf_feature.unlock();
        }
    }
}



void eventsCallback(const dvs_msgs::msg::EventArray &event_msg)//img_msg没有了
{
    // TicToc t_whole;
    //检测有没有事件
    const int n_event =event_msg.events.size();
    // ROS_INFO("THE SIZE OF EVENT:%d",n_event);
    if (n_event ==0) {
        ROS_WARN("not event, please move the event camera or check whether connecting");
        return;
    }
    // else if (n_event <=3000) {//少于一定阈值的时候,copy一下（此处目前是关键！！！！！）
    // // else if (n_event <=300) {//少于一定阈值的时候,copy一下（此处目前是关键！！！！！）
    // // when the number of events at the interval is less than the threshold, the old event frame will copy to the new event frame,
    // // which means that the camera keeps still at this interval.
    //     // return;
    //     // 如果当前的事件很少，那么就copy上一次的特征点，并且将特征点的速率都改为0
    // }


    if(first_image_flag)// 对第一帧图像的基本操作
    {
        first_image_flag = false;
        first_image_time = event_msg.events[0].ts;
        last_image_time = event_msg.events[0].ts;
        return;
    }
    // detect unstable camera stream
     // 检查时间戳是否正常，这里认为超过一秒或者错乱就异常
    // 图像时间差太多光流追踪就会失败，这里没有描述子匹配，因此对时间戳要求就高
    if (event_msg.events[0].ts - last_image_time > EVENT_GAP_RESET_S || event_msg.events[0].ts < last_image_time)
    {
        ROS_WARN("event stream discontinue! reset the feature tracker!");
        // 一些常规的reset操作
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;
        std_msgs::msg::Bool restart_flag;
        restart_flag.data = true;
        pub_restart->publish(restart_flag);// 告诉其他模块要重启了 (进行重启)
        return;
    }
    last_image_time = event_msg.events[0].ts;//也是创建time surface的时间

    // m_buf.lock();
    // events_buf.push(event_msg);
    // m_buf.unlock();

    // frequency control（// 控制一下发给后端的频率）
    if (round(1.0 * pub_count / (event_msg.events[0].ts - first_image_time)) <= FREQ)  // 保证发给后端的不超过这个频率
    {
        PUB_THIS_FRAME = true;//发布这帧
        // reset the frequency control
        // 这段时间的频率和预设频率十分接近，就认为这段时间很棒，重启一下，避免delta t太大
        if (abs(1.0 * pub_count / (event_msg.events[0].ts - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = event_msg.events[0].ts;
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;//不发布这帧

 // 即使不发布也是正常做光流追踪的！光流对图像的变化要求尽可能小
    // TicToc t_r;
    trackerData[0].readEvent(event_msg,event_msg.events[0].ts);//开始进行event的tracking处理
    // printf("featureTracker time: %f ms \n", t_r.toc());//看看特征检测需要的时间（ms）
    // if (t_r.toc()>18)
    //     std::cout<<"featureTracker time!!!!!!!!!!!!!!!!!!!!!!!!:"<<t_r.toc()<<std::endl;

    //此处是更新ID
    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);  // 单目的情况下可以直接用=号
        if (!completed)
            break;
    }

    //给回环发数据用的（发回环的图像以及event-corner）
    // events_buf.push(event_msg);
    // pubLoopImage(trackerData[0].getLoopImage(),event_msg.header.stamp.toSec());//发布用作回环检测的图像


   if (PUB_THIS_FRAME)// 若频率满足，发布当前帧。给后端喂数据
   {
        pub_count++;//计数，用于控制发送给后端的频率

        //下面是事件点特征
        {
            auto feature_points = std::make_shared<sensor_msgs::msg::PointCloud>();
            sensor_msgs::msg::ChannelFloat32 id_of_point;//特征点的id
            sensor_msgs::msg::ChannelFloat32 u_of_point;//特征点在图像中的uv位置
            sensor_msgs::msg::ChannelFloat32 v_of_point;
            sensor_msgs::msg::ChannelFloat32 velocity_x_of_point;//特征点在图像中的速度信息
            sensor_msgs::msg::ChannelFloat32 velocity_y_of_point;

            feature_points->header = event_msg.header;
            feature_points->header.frame_id = "world";

            vector<set<int>> hash_ids(NUM_OF_CAM);
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                auto &un_pts = trackerData[i].cur_un_pts; // 去畸变的归一化相机坐标系
                auto &cur_pts = trackerData[i].cur_pts;// 像素坐标
                auto &ids = trackerData[i].ids;//获得特征点的id值
                auto &pts_velocity = trackerData[i].pts_velocity; // 归一化坐标下的速度

                // ROS_ERROR("The number of the point:%d",ids.size());

                for (unsigned int j = 0; j < ids.size(); j++)
                {
                    if (trackerData[i].track_cnt[j] > 1)// 只发布追踪大于1的，因为等于1没法构成重投影约束，也没法三角化
                    {
                        int p_id = ids[j];
                        hash_ids[i].insert(p_id);
                        geometry_msgs::msg::Point32 p;
                        p.x = un_pts[j].x;
                        p.y = un_pts[j].y;
                        p.z = 1;//归一化平面上的点
                        // 利用这个ros消息的格式进行信息存储
                        feature_points->points.push_back(p);
                        id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                        u_of_point.values.push_back(cur_pts[j].x);
                        v_of_point.values.push_back(cur_pts[j].y);
                        velocity_x_of_point.values.push_back(pts_velocity[j].x);
                        velocity_y_of_point.values.push_back(pts_velocity[j].y);
                    }
                }
            }

            //再push到每一个通道中
            feature_points->channels.push_back(id_of_point);
            feature_points->channels.push_back(u_of_point);
            feature_points->channels.push_back(v_of_point);
            feature_points->channels.push_back(velocity_x_of_point);
            feature_points->channels.push_back(velocity_y_of_point);
            ROS_DEBUG("publish %f, at %f", rclcpp::Time(feature_points->header.stamp).seconds(), g_node->now().seconds());
            // skip the first image; since no optical speed on frist image
            if (!init_pub)
            {
                init_pub = 1;
            }
            else
                pub_img->publish(*feature_points);//发布特征点给后端

        }

         //下面是提取事件线特征的
        // if(trackerData[0].curframe_->vecLine.size()>=10)//大于十条线特征才会输出
        {
            auto feature_lines = std::make_shared<sensor_msgs::msg::PointCloud>();//定义线特征
            sensor_msgs::msg::ChannelFloat32 id_of_line;   //  线特征的id feature id
            sensor_msgs::msg::ChannelFloat32 u_of_endpoint;    //  u 线特征的终点
            sensor_msgs::msg::ChannelFloat32 v_of_endpoint;    //  v

            feature_lines->header = event_msg.header;
            feature_lines->header.frame_id = "world";

            vector<set<int>> hash_ids_line(NUM_OF_CAM);
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                auto un_lines = trackerData[0].curframe_->vecLine;

                auto &ids_line = trackerData[0].curframe_->lineID;//获取当前所有线的id

                for (unsigned int j = 0; j < ids_line.size(); j++)
                {

                    int p_id = ids_line[j];//当前线的id
                    hash_ids_line[i].insert(p_id);
                    geometry_msgs::msg::Point32 p;//起点的xy
                    p.x = un_lines[j].StartPt.x;//起点的xy
                    p.y = un_lines[j].StartPt.y;
                    p.z = 1;

                    feature_lines->points.push_back(p);//放入线的起点
                    id_of_line.values.push_back(p_id * NUM_OF_CAM + i);//线的id
                    // std::cout<< "feature tracking id: " <<p_id * NUM_OF_CAM + i<<" "<<p_id<<"\n";
                    u_of_endpoint.values.push_back(un_lines[j].EndPt.x);//放入线的终点
                    v_of_endpoint.values.push_back(un_lines[j].EndPt.y);
                    //ROS_ASSERT(inBorder(cur_pts[j]));
                }

            }
            feature_lines->channels.push_back(id_of_line);
            feature_lines->channels.push_back(u_of_endpoint);
            feature_lines->channels.push_back(v_of_endpoint);
            pub_feature_line->publish(*feature_lines);//把线特征发布出去
        }


         if (SHOW_TRACK)//需要show跟踪的过程
        {
            // 额外写发布的函数
            cv::Mat imageTrack=trackerData[0].getTrackImage();
            cv::Mat imgTrack_two =trackerData[0].getTrackImage_two();
            cv::Mat imgTrack_two_line =trackerData[0].getTrackImage_two_line();//线特征的匹配结果
            cv::Mat Time_surface_map =trackerData[0].gettimesurface();
            pubTrackImage(imageTrack,imgTrack_two,imgTrack_two_line,Time_surface_map,last_image_time);
        }

    }
    // if(t_whole.toc()>20)
    //     ROS_INFO("whole feature tracker processing costs!!!!!!!!!!!!!!!!!!!!: %f", t_whole.toc());
    // std::cout<<"feed to the estimator?"<<PUB_THIS_FRAME<<std::endl;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("feature_tracker");
    ROS_WARN("into event point and line feature detection and tracking");
    readParameters(g_node); // 读取配置文件

    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);  // 获得每个相机的内参

    // 事件的订阅者
    auto event_sub = g_node->create_subscription<dvs_msgs::msg::EventArray>(
        EVENT_TOPIC, rclcpp::QoS(rclcpp::KeepLast(20)).best_effort(),
        [](const dvs_msgs::msg::EventArray::SharedPtr msg) { eventsCallback(*msg); });

    // 注册一些publisher
    registerPub(g_node);//注册一些发布者(自定义的)

    //额外开一个线程，额外的将event-corner发布给pose_graph
    // std::thread sync_thread{sync_process_event_corner_multi_thread};//多线程来提取特征点

    rclcpp::spin(g_node); // spin代表这个节点开始循环查询topic是否接收
    rclcpp::shutdown();
    return 0;
}
