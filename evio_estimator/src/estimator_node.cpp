#include <chrono>
#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"


Estimator estimator;
rclcpp::Node::SharedPtr g_node;//给free function里用到的logger
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr g_diag_pub;

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buf;
queue<sensor_msgs::msg::PointCloud::ConstSharedPtr> feature_buf;//事件点特征
queue<sensor_msgs::msg::PointCloud::ConstSharedPtr> imagefeature_buf;//图像点特征
queue<sensor_msgs::msg::PointCloud::ConstSharedPtr> linefeature_buf;//线特征
queue<sensor_msgs::msg::PointCloud::ConstSharedPtr> relo_buf;
int sum_of_wait = 0;

// Bounded-queue caps. When VIO cannot initialize (e.g. the image-feature stream is
// starved, as happens in low light), getMeasurements_pl_image() returns empty without
// draining these buffers, so they would otherwise grow without limit at IMU rate
// (~1kHz) and eventually OOM the machine. Capping to drop the oldest message once a
// buffer exceeds its limit bounds memory while keeping the most-recent (and only
// usable) data. During normal operation these caps are never reached because the
// processing thread drains every frame.
static const size_t MAX_IMU_BUF = 20000;      // ~20-40 s of IMU history
static const size_t MAX_FEATURE_BUF = 2000;   // ~100 s at 20 Hz
static const size_t MAX_RELO_BUF = 100;

template <typename Q>
static void push_capped(Q & q, const typename Q::value_type & msg, size_t cap)
{
    q.push(msg);
    while (q.size() > cap)
        q.pop();  // drop oldest
}

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_estimator;

double latest_time;//当前帧的时间
//状态变量，窗口中的PQV以及Ba与Bg
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
//上一时刻的加速度与角速度
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0;

static inline double toSec(const builtin_interfaces::msg::Time & stamp)
{
    return rclcpp::Time(stamp).seconds();
}

/**
 * @brief 根据当前imu数据预测当前位姿
 *
 * @param[in] imu_msg
 */
void predict(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)//根据当前imu数据预测当前位姿
{
    double t = toSec(imu_msg->header.stamp);
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

  // 得到加速度
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    // double dx = imu_msg->linear_acceleration.z;
    // double dy = -imu_msg->linear_acceleration.x;
    // double dz = -imu_msg->linear_acceleration.y;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    // 得到角速度
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    // double rx = imu_msg->angular_velocity.z;
    // double ry = -imu_msg->angular_velocity.x;
    // double rz = -imu_msg->angular_velocity.y;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    //下面通过中值积分获得加速度与角速度的平均值
    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;   // 上一时刻世界坐标系下加速度值
    // 当前IMU坐标系下的加速度-加速度的bias，然后乘以tmp_Q转换到世界坐标系下，再把重力加速度g减去。得到上一时刻世界坐标系下的加速度

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;    // 中值陀螺仪的结果
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);// 更新姿态（更新Q）  根据角度度，获得当前时刻最新的旋转，然用该值求当前时刻世界坐标系下的加速度

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;// 当前时刻世界坐标系下的加速度值（与上面同理）

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);// 加速度中值积分的值

    // 经典物理中位置，速度更新方程（更新P与V）
    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;//把当前值保存下来，下次用
    gyr_0 = angular_velocity;
}


void update()// 用最新VIO结果更新最新imu对应的位姿
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];//窗口中的P
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::msg::Imu::ConstSharedPtr> tmp_imu_buf = imu_buf;// 遗留的imu的buffer，因为下面需要pop，所以copy了一份
    for (sensor_msgs::msg::Imu::ConstSharedPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());// 得到最新imu时刻的位姿

}

// 获得匹配好的图像imu组
std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>, sensor_msgs::msg::PointCloud::ConstSharedPtr>>
getMeasurements()//实际上就是为了获取当前特征那段的imu数据（时间的软同步）
{
    std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>, sensor_msgs::msg::PointCloud::ConstSharedPtr>> measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())//IMU与图像都没来
            return measurements;

        if (!(toSec(imu_buf.back()->header.stamp) > toSec(feature_buf.front()->header.stamp) + estimator.td))//最新的IMU的时间小于等于当前图像的时间+td（那么就是IMU没到）
        {
            //ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }

        if (!(toSec(imu_buf.front()->header.stamp) < toSec(feature_buf.front()->header.stamp) + estimator.td))//当最早的IMU的时间>=最早的图像的时间+td，扔掉在IMU前面发生的一下image
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        } // 此时就保证了图像前一定有imu数据

        //时间戳检查完毕，凯撒获取IMU与image feature信息
        sensor_msgs::msg::PointCloud::ConstSharedPtr img_msg = feature_buf.front();//拿到最前的feature
        feature_buf.pop();//删掉最前的feature
        // 一般第一帧不会严格对齐，但是后面就都会对齐，当然第一帧也不会用到
        std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> IMUs;
        while (toSec(imu_buf.front()->header.stamp) < toSec(img_msg->header.stamp) + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }

        IMUs.emplace_back(imu_buf.front());// 保留图像时间戳后一个imu数据，但不会从buffer中扔掉
        if (IMUs.empty())
            ROS_WARN("no imu between two image");
        measurements.emplace_back(IMUs, img_msg);//放进来进行处理
    }
    return measurements;
}

// 加入了线特征
std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>,
        std::pair<sensor_msgs::msg::PointCloud::ConstSharedPtr,sensor_msgs::msg::PointCloud::ConstSharedPtr> >>
getMeasurements_pl()//实际上就是为了获取当前特征那段的imu数据（时间的软同步）
{
    // std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>, sensor_msgs::msg::PointCloud::ConstSharedPtr>> measurements;
    std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>,
            std::pair<sensor_msgs::msg::PointCloud::ConstSharedPtr,sensor_msgs::msg::PointCloud::ConstSharedPtr> >> measurements;

    while (true)
    {
        // if (imu_buf.empty() || feature_buf.empty()|| linefeature_buf.empty())//IMU与图像都没来 (是否需要管line buffer？)
        if (imu_buf.empty() || feature_buf.empty())//IMU与图像都没来 (是否需要管line buffer？)
            return measurements;

        if (!(toSec(imu_buf.back()->header.stamp) > toSec(feature_buf.front()->header.stamp) + estimator.td))//最新的IMU的时间小于等于当前图像的时间+td（那么就是IMU没到）
        {
            //ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }

        if (!(toSec(imu_buf.front()->header.stamp) < toSec(feature_buf.front()->header.stamp) + estimator.td))//当最早的IMU的时间>=最早的图像的时间+td，扔掉在IMU前面发生的一下image
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            linefeature_buf.pop();//线特征
            continue;
        } // 此时就保证了图像前一定有imu数据

        //时间戳检查完毕，凯撒获取IMU与image feature信息
        sensor_msgs::msg::PointCloud::ConstSharedPtr img_msg = feature_buf.front();//拿到最前的feature
        feature_buf.pop();//删掉最前的feature
        sensor_msgs::msg::PointCloud::ConstSharedPtr linefeature_msg = linefeature_buf.front();//获取线特征
        linefeature_buf.pop();

        // ROS_ERROR("The number of the line:%d",linefeature_msg->points.size());
        // ROS_ERROR("The number of the point:%d",img_msg->points.size());

        // 一般第一帧不会严格对齐，但是后面就都会对齐，当然第一帧也不会用到
        std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> IMUs;
        while (toSec(imu_buf.front()->header.stamp) < toSec(img_msg->header.stamp) + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }

        IMUs.emplace_back(imu_buf.front());// 保留图像时间戳后一个imu数据，但不会从buffer中扔掉
        if (IMUs.empty())
            ROS_WARN("no imu between two image");

        // measurements.emplace_back(IMUs, img_msg);//放进来进行处理
        measurements.emplace_back(IMUs, std::make_pair(img_msg,linefeature_msg));
    }
    return measurements;
}


//事件点线特征+图像特征（getMeasurements_pl_image）
std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>,
                std::pair<sensor_msgs::msg::PointCloud::ConstSharedPtr,std::pair<std::vector<sensor_msgs::msg::PointCloud::ConstSharedPtr>,std::vector<sensor_msgs::msg::PointCloud::ConstSharedPtr>> > > >
getMeasurements_pl_image()//实际上就是为了获取当前特征那段的imu数据（时间的软同步）
{
    // std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>, sensor_msgs::msg::PointCloud::ConstSharedPtr>> measurements;
    // std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>,
    //         std::pair<sensor_msgs::msg::PointCloud::ConstSharedPtr,sensor_msgs::msg::PointCloud::ConstSharedPtr> >> measurements;
    std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>,
                std::pair<sensor_msgs::msg::PointCloud::ConstSharedPtr,
                    std::pair<std::vector<sensor_msgs::msg::PointCloud::ConstSharedPtr>,std::vector<sensor_msgs::msg::PointCloud::ConstSharedPtr>> > > > measurements;

    // imagefeature_buf 图像特征
    //feature_buf 事件点特征
    //linefeature_buf 事件线特征
    // imu_buf imu

    while (true)
    {
        // if (imu_buf.empty() || feature_buf.empty()|| linefeature_buf.empty())//IMU与图像都没来 (是否需要管line buffer？)
        if (imu_buf.empty() || imagefeature_buf.empty())//IMU与图像都没来 (那就返回空的)
            return measurements;

        if (!(toSec(imu_buf.back()->header.stamp) > toSec(imagefeature_buf.front()->header.stamp) + estimator.td))//最新的IMU的时间小于等于当前图像的时间+td（那么就是IMU没到）
        {
            //ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }

        if (!(toSec(imu_buf.front()->header.stamp) < toSec(imagefeature_buf.front()->header.stamp) + estimator.td))//当最早的IMU的时间>=最早的图像的时间+td，扔掉在IMU前面发生的一下image
        {
            ROS_WARN("throw img, only should happen at the beginning");
            if(feature_buf.size()!=0)
                feature_buf.pop();//事件点特征
            if(linefeature_buf.size()!=0)
                linefeature_buf.pop();//线特征
            if(imagefeature_buf.size()!=0)
                imagefeature_buf.pop();//图像特征
            continue;
        } // 此时就保证了图像前一定有imu数据

        //时间戳检查完毕，凯撒获取IMU与image feature信息
        std::vector<sensor_msgs::msg::PointCloud::ConstSharedPtr> event_msg;
        while(feature_buf.size()!=0){
        // if(feature_buf.size()!=0){
            event_msg.emplace_back(feature_buf.front());//拿到最前的feature
            feature_buf.pop();//删掉最前的feature
        }
        std::vector<sensor_msgs::msg::PointCloud::ConstSharedPtr> linefeature_msg;
        while(linefeature_buf.size()!=0){
        // if(linefeature_buf.size()!=0){
            linefeature_msg.emplace_back(linefeature_buf.front());//获取线特征
            linefeature_buf.pop();
        }
        sensor_msgs::msg::PointCloud::ConstSharedPtr image_msg;
        if(imagefeature_buf.size()!=0){
            image_msg = imagefeature_buf.front();//拿到最前的图像feature
            imagefeature_buf.pop();//删掉最前的feature
        }

        // ROS_ERROR("image_buf: %d, event_point_buf: %d, event_line_buf:%d",imagefeature_buf.size(),feature_buf.size(),linefeature_buf.size());


        // 一般第一帧不会严格对齐，但是后面就都会对齐，当然第一帧也不会用到
        std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> IMUs;
        while (toSec(imu_buf.front()->header.stamp) < toSec(image_msg->header.stamp) + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }

        IMUs.emplace_back(imu_buf.front());// 保留图像时间戳后一个imu数据，但不会从buffer中扔掉
        if (IMUs.empty())
            ROS_WARN("no imu between two image");

        // measurements.emplace_back(IMUs, img_msg);//放进来进行处理
        measurements.emplace_back(IMUs, std::make_pair(image_msg,std::make_pair(event_msg,linefeature_msg)));//IMU+（图像+（事件点特征，事件线特征））
    }
    return measurements;
}

/**
 * @brief imu消息存进buffer，同时按照imu频率预测位姿并发送，这样就可以提高里程计频率
 *
 * @param[in] imu_msg
 */
void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
    if (toSec(imu_msg->header.stamp) <= last_imu_t)
    {
        // Measured on the DAVIS346 setup: every one of these is an exact duplicate stamp (dt == 0) of the
        // previous sample, never a backwards step; the check drops it, so only report a throttled summary.
        static size_t n_dup = 0, n_back = 0;
        static auto t_last_report = std::chrono::steady_clock::now();
        if (toSec(imu_msg->header.stamp) == last_imu_t) ++n_dup; else ++n_back;
        auto now = std::chrono::steady_clock::now();
        if (now - t_last_report > std::chrono::seconds(5))
        {
            ROS_WARN("dropped IMU samples in last 5 s: %zu duplicate-stamp, %zu backwards (last: stamp - last = %.3f ms)",
                     n_dup, n_back, (toSec(imu_msg->header.stamp) - last_imu_t) * 1e3);
            n_dup = n_back = 0;
            t_last_report = now;
        }
        return;
    }
    last_imu_t = toSec(imu_msg->header.stamp);

    // double a=0,b=0,c=0;
    // sensor_msgs::msg::Imu::ConstSharedPtr flight_imu
    // a = imu_msg->linear_acceleration.x;
    // b = imu_msg->linear_acceleration.y;
    // c = imu_msg->linear_acceleration.z;
    // flight_imu->linear_acceleration.x = a;
    // imu_msg->linear_acceleration.x = c;
    // imu_msg->linear_acceleration.y = -a;
    // imu_msg->linear_acceleration.z = -b;

    m_buf.lock();
    push_capped(imu_buf, imu_msg, MAX_IMU_BUF);//将IMU放入buf中（有上限，防止未初始化时无限增长）
    m_buf.unlock();
    con.notify_one();

    last_imu_t = toSec(imu_msg->header.stamp);

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);//预测最新的位置
        std_msgs::msg::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR){// 只有初始化完成后才发送当前结果
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);//基于IMU做快速的预测，发布最新的odometry，保证了里程计的发布频率（此处有两个话题“imu_propagate”与“imu_evio”）
        }
    }


}


/**
 * @brief 将前端信息送进buffer
 *
 * @param[in] feature_msg
 */
void feature_callback(const sensor_msgs::msg::PointCloud::ConstSharedPtr feature_msg)//订阅来自前端的消息（特征点）
{
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    // if( feature_msg->points.size()>6){//为了防止event太少，起码要有一个数量的feature point出来才可以
        m_buf.lock();
        push_capped(feature_buf, feature_msg, MAX_FEATURE_BUF);//将feature放入feature buf中（有上限）
        m_buf.unlock();
        con.notify_one();
    // }
    // else{ROS_INFO("no enough event feature, please move the camera"); }
}

void linefeature_callback(const sensor_msgs::msg::PointCloud::ConstSharedPtr feature_msg)//线特征
{
    // if(feature_msg->points.size()>3){
        m_buf.lock();
        push_capped(linefeature_buf, feature_msg, MAX_FEATURE_BUF);
        m_buf.unlock();
        con.notify_one();
    // }
}

void imagefeature_callback(const sensor_msgs::msg::PointCloud::ConstSharedPtr feature_msg)
{
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    m_buf.lock();
    push_capped(imagefeature_buf, feature_msg, MAX_FEATURE_BUF);
    m_buf.unlock();
    con.notify_one();
}

/**
 * @brief 将vins估计器复位
 *
 * @param[in] restart_msg (重启的消息，一个bool值)
 */
void restart_callback(const std_msgs::msg::Bool::ConstSharedPtr restart_msg)//复位的时候能否改成下一个在某个位置下重启，而不是00？
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the EVIO estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();//全部删除
        while(!imu_buf.empty())
            imu_buf.pop();//全部删除
        while(!imagefeature_buf.empty())
            imagefeature_buf.pop();//全部删除
        while(!linefeature_buf.empty())
            linefeature_buf.pop();//全部删除
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();//清空
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

void relocalization_callback(const sensor_msgs::msg::PointCloud::ConstSharedPtr points_msg)//检测到回环的点
{
    //printf("relocalization callback! \n");
    m_buf.lock();
    push_capped(relo_buf, points_msg, MAX_RELO_BUF);
    m_buf.unlock();
}

// thread: visual-inertial odometry
void process()
{
    while (true)  // 这个线程是会一直循环下去
    {
        std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>, sensor_msgs::msg::PointCloud::ConstSharedPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);//先上锁，等到有数据后再解开
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;//一直等到获得IMU与feature的measurement为止
                 });
        lk.unlock();// 数据buffer的锁解锁，回调可以继续塞数据了

        m_estimator.lock();// 进行后端求解，不能和复位重启冲突
        for (auto &measurement : measurements) // 给予范围的for循环，这里就是遍历每组image imu组合
        {
            auto img_msg = measurement.second;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first)// 遍历imu
            {
                double t = toSec(imu_msg->header.stamp);//imu的时间
                double img_t = toSec(img_msg->header.stamp) + estimator.td;//img_t=img的时间+td
                if (t <= img_t)//如果imu的时间<=img的时间+td（预测的imu时间）
                {
                    if (current_time < 0)//初始的时候
                        current_time = t;
                    double dt = t - current_time;//实际IMU的时间跟预测的IMU的时间之差
                    ROS_ASSERT(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    //计算当前图像帧间的imu预积分值，即帧间评议、旋转、速度以及bias
                    // 并利用imu对系统最新状态进行传播，为视觉三角化及重投影提供位姿初值
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));// 时间差和imu数据送进去
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);

                }
                else // 这就是针对最后一个imu数据，需要做一个简单的线性插值
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }

            // set relocalization frame（// 回环相关部分,设置重定位帧）
            sensor_msgs::msg::PointCloud::ConstSharedPtr relo_msg = nullptr;
            while (!relo_buf.empty())   // 取出最新的回环帧
            {
                relo_msg = relo_buf.front();// 返回队首元素的值
                relo_buf.pop();//删除队列首元素
            }
            if (relo_msg != nullptr)  // 有效回环信息
            {
                vector<Vector3d> match_points; // 回环帧的归一化坐标和地图点idx
                double frame_stamp = toSec(relo_msg->header.stamp);
                for (unsigned int i = 0; i < relo_msg->points.size(); i++)//遍历relo_msg中的points特征点
                {
                    Vector3d u_v_id;
                    u_v_id.x() = relo_msg->points[i].x;
                    u_v_id.y() = relo_msg->points[i].y;
                    u_v_id.z() = relo_msg->points[i].z;
                    match_points.push_back(u_v_id);
                }

                // 回环帧的位姿( [重定位帧的平移向量T的x,y,z，旋转四元数w,x,y,z和索引值])
                Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];//索引
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);//设置回环
            }

            ROS_DEBUG("processing vision data with stamp %f \n", toSec(img_msg->header.stamp));

//下面开始处理feature
            TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;// 特征点id->特征点信息
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;//特征点的id
                int camera_id = v % NUM_OF_CAM;//camera的id
                double x = img_msg->points[i].x;// 去畸变后归一滑像素坐标
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];// 特征点像素坐标
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];// 特征点速度
                double velocity_y = img_msg->channels[4].values[i];
                ROS_ASSERT(z == 1); // 检查是不是归一化
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            }
            estimator.processImage(image, img_msg->header);//主要的处理feature的函数


             // 一些打印以及topic的发送
            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);
            std_msgs::msg::Header header = img_msg->header;
            header.frame_id = "world";

            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);//发布点云(基于estimator,而里面包含了pose)
            pubTF(estimator, header);//发布TF，camera与imu的外参
            pubKeyframe(estimator);//把关键帧的pose以及地图点发布出来
            if (relo_msg != nullptr)
                pubRelocalization(estimator);//发布回环重定位的结果
        }
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}

void process_pointline()
{
    while (true)  // 这个线程是会一直循环下去
    {
        std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>,
                std::pair<sensor_msgs::msg::PointCloud::ConstSharedPtr,sensor_msgs::msg::PointCloud::ConstSharedPtr> >> measurements; //现在的measurement包含了点特征与线特征

        std::unique_lock<std::mutex> lk(m_buf);//先上锁，等到有数据后再解开
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements_pl()).size() != 0;
                 });
        lk.unlock();// 数据buffer的锁解锁，回调可以继续塞数据了

        m_estimator.lock();// 进行后端求解，不能和复位重启冲突
        for (auto &measurement : measurements) // 给予范围的for循环，这里就是遍历每组image imu组合
        {
            auto point_and_line_msg = measurement.second;//包含线特征与点特征
            auto img_msg = point_and_line_msg.first;//点状特征数据
            auto line_msg = point_and_line_msg.second;//线状特征数据

            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first)// 遍历imu
            {
                double t = toSec(imu_msg->header.stamp);//imu的时间
                double img_t = toSec(img_msg->header.stamp) + estimator.td;//img_t=img的时间+td
                if (t <= img_t)//如果imu的时间<=img的时间+td（预测的imu时间）
                {
                    if (current_time < 0)//初始的时候
                        current_time = t;
                    double dt = t - current_time;//实际IMU的时间跟预测的IMU的时间之差
                    ROS_ASSERT(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));// 时间差和imu数据送进去
                }
                else // 这就是针对最后一个imu数据，需要做一个简单的线性插值
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                }
            }

            // set relocalization frame（// 回环相关部分,设置重定位帧）
            sensor_msgs::msg::PointCloud::ConstSharedPtr relo_msg = nullptr;
            while (!relo_buf.empty())   // 取出最新的回环帧
            {
                relo_msg = relo_buf.front();// 返回队首元素的值
                relo_buf.pop();//删除队列首元素
            }
            if (relo_msg != nullptr)  // 有效回环信息
            {
                vector<Vector3d> match_points; // 回环帧的归一化坐标和地图点idx
                double frame_stamp = toSec(relo_msg->header.stamp);
                for (unsigned int i = 0; i < relo_msg->points.size(); i++)//遍历relo_msg中的points特征点
                {
                    Vector3d u_v_id;
                    u_v_id.x() = relo_msg->points[i].x;
                    u_v_id.y() = relo_msg->points[i].y;
                    u_v_id.z() = relo_msg->points[i].z;
                    match_points.push_back(u_v_id);
                }

                Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];//索引
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);//设置回环
            }

            ROS_DEBUG("processing vision data with stamp %f \n", toSec(img_msg->header.stamp));

//下面开始处理feature
            TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;// 特征点id->特征点信息
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];
                ROS_ASSERT(z == 1); // 检查是不是归一化
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            }

             //提取线状特征
            map<int, vector<pair<int, Vector4d>>> lines;
            for (unsigned int i = 0; i < line_msg->points.size(); i++)
            {
                int v = line_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;        // 被几号相机观测到的，如果是单目，camera_id = 0
                double x_startpoint = line_msg->points[i].x;
                double y_startpoint = line_msg->points[i].y;
                double x_endpoint = line_msg->channels[1].values[i];
                double y_endpoint = line_msg->channels[2].values[i];
                lines[feature_id].emplace_back(camera_id, Vector4d(x_startpoint, y_startpoint, x_endpoint, y_endpoint));//起始点、终止点
            }

            estimator.processImage(image, lines, img_msg->header); // 把点状特征也加入进行处理。


             // 一些打印以及topic的发送
            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);
            std_msgs::msg::Header header = img_msg->header;
            header.frame_id = "world";

            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);//发布点云(基于estimator,而里面包含了pose)
            pubTF(estimator, header);//发布TF，camera与imu的外参
            pubKeyframe(estimator);//把关键帧的pose以及地图点发布出来
            if (relo_msg != nullptr)
                pubRelocalization(estimator);//发布回环重定位的结果
        }
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}

// process_pointline_image(事件点与线特征+图像特征)

void process_pointline_image()
{
    while (true)  // 这个线程是会一直循环下去
    {
        std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>,
                std::pair<sensor_msgs::msg::PointCloud::ConstSharedPtr,std::pair<std::vector<sensor_msgs::msg::PointCloud::ConstSharedPtr>,std::vector<sensor_msgs::msg::PointCloud::ConstSharedPtr>> > > > measurements; //现在的measurement包含了点特征与线特征+图像点特征
        // IMU+（图像+（事件点特征，事件线特征））

        std::unique_lock<std::mutex> lk(m_buf);//先上锁，等到有数据后再解开
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements_pl_image()).size() != 0;
                 });
        lk.unlock();// 数据buffer的锁解锁，回调可以继续塞数据了

        m_estimator.lock();// 进行后端求解，不能和复位重启冲突
        for (auto &measurement : measurements) // 给予范围的for循环，这里就是遍历每组image imu组合
        {
            // IMU+（图像+（事件点特征，事件线特征））
            auto img_msg=measurement.second.first;//图像特征
            auto point_and_line_msg = measurement.second.second;//包含线特征与点特征
            auto event_msg = point_and_line_msg.first;//点状特征数据
            auto line_msg = point_and_line_msg.second;//线状特征数据

            // 先处理IMU部分
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first)// 遍历imu
            {
                double t = toSec(imu_msg->header.stamp);//imu的时间
                double img_t = toSec(img_msg->header.stamp) + estimator.td;//img_t=img的时间+td
                if (t <= img_t)//如果imu的时间<=img的时间+td（预测的imu时间）
                {
                    if (current_time < 0)//初始的时候
                        current_time = t;
                    double dt = t - current_time;//实际IMU的时间跟预测的IMU的时间之差
                    ROS_ASSERT(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    //计算当前图像帧间的imu预积分值，即帧间评议、旋转、速度以及bias
                    // 并利用imu对系统最新状态进行传播，为视觉三角化及重投影提供位姿初值
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));// 时间差和imu数据送进去
                }
                else // 这就是针对最后一个imu数据，需要做一个简单的线性插值
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                }
            }

            // set relocalization frame（// 回环相关部分,设置重定位帧）
            sensor_msgs::msg::PointCloud::ConstSharedPtr relo_msg = nullptr;
            while (!relo_buf.empty())   // 取出最新的回环帧
            {
                relo_msg = relo_buf.front();// 返回队首元素的值
                relo_buf.pop();//删除队列首元素
            }
            if (relo_msg != nullptr)  // 有效回环信息
            {
                vector<Vector3d> match_points; // 回环帧的归一化坐标和地图点idx
                double frame_stamp = toSec(relo_msg->header.stamp);
                for (unsigned int i = 0; i < relo_msg->points.size(); i++)//遍历relo_msg中的points特征点
                {
                    Vector3d u_v_id;
                    u_v_id.x() = relo_msg->points[i].x;
                    u_v_id.y() = relo_msg->points[i].y;
                    u_v_id.z() = relo_msg->points[i].z;
                    match_points.push_back(u_v_id);
                }

                // 回环帧的位姿( [重定位帧的平移向量T的x,y,z，旋转四元数w,x,y,z和索引值])
                Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];//索引
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);//设置回环
            }

            ROS_DEBUG("processing vision data with stamp %f \n", toSec(img_msg->header.stamp));

            //下面开始处理视觉feature
            // （提取当前事件帧的特征点数据，提取点状特征）
            TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> event;// 特征点id->特征点信息
            if(!event_msg.empty()){
                for (auto event_point_msg:event_msg){
                    for (unsigned int i = 0; i < event_point_msg->points.size(); i++)
                    {
                        int v = event_point_msg->channels[0].values[i] + 0.5;
                        int feature_id = v / NUM_OF_CAM;//特征点的id
                        int camera_id = v % NUM_OF_CAM;//camera的id
                        double x = event_point_msg->points[i].x;// 去畸变后归一滑像素坐标
                        double y = event_point_msg->points[i].y;
                        double z = event_point_msg->points[i].z;
                        double p_u = event_point_msg->channels[1].values[i];// 特征点像素坐标
                        double p_v = event_point_msg->channels[2].values[i];
                        double velocity_x = event_point_msg->channels[3].values[i];// 特征点速度
                        double velocity_y = event_point_msg->channels[4].values[i];
                        ROS_ASSERT(z == 1); // 检查是不是归一化
                        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                        event[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
                    }
                }
            }

            // 提取图像角点特征
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;// 特征点id->特征点信息
            if(img_msg!=nullptr){
                for (unsigned int i = 0; i < img_msg->points.size(); i++)
                {
                    int v = img_msg->channels[0].values[i] + 0.5;
                    int feature_id = v / NUM_OF_CAM;
                    int camera_id = v % NUM_OF_CAM;
                    double x = img_msg->points[i].x;
                    double y = img_msg->points[i].y;
                    double z = img_msg->points[i].z;
                    double p_u = img_msg->channels[1].values[i];
                    double p_v = img_msg->channels[2].values[i];
                    double velocity_x = img_msg->channels[3].values[i];
                    double velocity_y = img_msg->channels[4].values[i];
                    ROS_ASSERT(z == 1);
                    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                    xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                    image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
                }
            }

             //提取线状特征
            map<int, vector<pair<int, Vector4d>>> lines;
            if(!line_msg.empty()){
                for(auto event_line_msg:line_msg){
                    for (unsigned int i = 0; i < event_line_msg->points.size(); i++)
                    {
                        int v = event_line_msg->channels[0].values[i] + 0.5;
                        int feature_id = v / NUM_OF_CAM;
                        int camera_id = v % NUM_OF_CAM;        // 被几号相机观测到的，如果是单目，camera_id = 0
                        double x_startpoint = event_line_msg->points[i].x;
                        double y_startpoint = event_line_msg->points[i].y;
                        double x_endpoint = event_line_msg->channels[1].values[i];
                        double y_endpoint = event_line_msg->channels[2].values[i];
                        lines[feature_id].emplace_back(camera_id, Vector4d(x_startpoint, y_startpoint, x_endpoint, y_endpoint));//起始点、终止点
                    }
                }
            }

            estimator.processImage(image,event,lines, img_msg->header); // 把点状特征也加入进行处理。
            {   // diagnostics: [solver_flag, td, ba(3), bg(3), ric ypr(3), tic(3)] of the newest window frame
                std_msgs::msg::Float64MultiArray d;
                const Eigen::Vector3d ypr = Utility::R2ypr(estimator.ric[0]);
                d.data = {(double)estimator.solver_flag, estimator.td,
                          estimator.Bas[WINDOW_SIZE].x(), estimator.Bas[WINDOW_SIZE].y(), estimator.Bas[WINDOW_SIZE].z(),
                          estimator.Bgs[WINDOW_SIZE].x(), estimator.Bgs[WINDOW_SIZE].y(), estimator.Bgs[WINDOW_SIZE].z(),
                          ypr.x(), ypr.y(), ypr.z(), estimator.tic[0].x(), estimator.tic[0].y(), estimator.tic[0].z()};
                g_diag_pub->publish(d);
            }


             // 一些打印以及topic的发送
            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);
            std_msgs::msg::Header header = img_msg->header;
            header.frame_id = "world";

            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);//发布点云(基于estimator,而里面包含了pose)
            pubTF(estimator, header);//发布TF，camera与imu的外参
            pubKeyframe(estimator);//把关键帧的pose以及地图点发布出来
            if (relo_msg != nullptr)
                pubRelocalization(estimator);//发布回环重定位的结果
        }
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}


//发布GT
void GTCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg){
    pubGT(msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("evio_estimator");
    readParameters(g_node);
    estimator.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ROS_WARN("waiting for image and imu...");

    registerPub(g_node);//注册一些发布者（这个做法跟vins-fusion很像）

    const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(2000));
    const auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(2000)).best_effort();

    auto sub_imu = g_node->create_subscription<sensor_msgs::msg::Imu>(IMU_TOPIC, imu_qos, imu_callback);//订阅IMU
    auto sub_feature = g_node->create_subscription<sensor_msgs::msg::PointCloud>("/feature_tracker/feature", reliable_qos, feature_callback);//订阅feature（前端的结果）
    auto sub_linefeature = g_node->create_subscription<sensor_msgs::msg::PointCloud>("/feature_tracker/linefeature", reliable_qos, linefeature_callback);//线特征回调
    auto sub_imagefeature = g_node->create_subscription<sensor_msgs::msg::PointCloud>("/image_feature_tracker/feature", reliable_qos, imagefeature_callback);//图像特征回调
    auto sub_restart = g_node->create_subscription<std_msgs::msg::Bool>("/feature_tracker/restart", reliable_qos, restart_callback);// 接受前端重启命令
    auto sub_relo_points = g_node->create_subscription<sensor_msgs::msg::PointCloud>("/pose_graph/match_points", reliable_qos, relocalization_callback);//回环检测由pose_graph发过来的。重定位（ 回环检测的fast relocalization响应）

    auto gt_pose_sub_ = g_node->create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", rclcpp::QoS(1), GTCallback);//订阅GT然后发布
    g_diag_pub = g_node->create_publisher<std_msgs::msg::Float64MultiArray>("~/diag", 10);//诊断：bias/td/外参

    // std::thread measurement_process{process};//额外开了一个线程（核心处理线程）
    // std::thread measurement_process{process_pointline};//点特征与线特征的版本
    std::thread measurement_process{process_pointline_image};//事件点特征与线特征+图像点特征的版本
    rclcpp::spin(g_node);
    rclcpp::shutdown();

    return 0;
}
