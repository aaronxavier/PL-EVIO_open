#include "arc_star_detector.h"

std::mutex events_mutex_;//相当于加了一个锁（互斥锁）。每次调用消息前，先锁上，把消息放到buf里面后再解锁

namespace evio  { // acd : Asynchronous Corner Detector

// ArcStarDetector::ArcStarDetector() :
//     kSmallCircle_{{0, 3}, {1, 3}, {2, 2}, {3, 1},
//               {3, 0}, {3, -1}, {2, -2}, {1, -3},
//               {0, -3}, {-1, -3}, {-2, -2}, {-3, -1},
//               {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}},
//     kLargeCircle_{{0, 4}, {1, 4}, {2, 3}, {3, 2},
//               {4, 1}, {4, 0}, {4, -1}, {3, -2},
//               {2, -3}, {1, -4}, {0, -4}, {-1, -4},
//               {-2, -3}, {-3, -2}, {-4, -1}, {-4, 0},
//               {-4, 1}, {-3, 2}, {-2, 3}, {-1, 4}} {

//     // Initialize Surface of Active Events to 0-timestamp
//     //初始化SAE为0时刻。SAE中的元素为时刻
//     //sae_与sae_latest_为当前时刻与上一时刻的SAE
//     ROS_INFO("66666666666666");
//     sae_[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
//     sae_[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
//     sae_latest_[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
//     sae_latest_[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
// }

ArcStarDetector::ArcStarDetector() :
    kSmallCircle_{{0, 3}, {1, 3}, {2, 2}, {3, 1},
              {3, 0}, {3, -1}, {2, -2}, {1, -3},
              {0, -3}, {-1, -3}, {-2, -2}, {-3, -1},
              {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}},
    kLargeCircle_{{0, 4}, {1, 4}, {2, 3}, {3, 2},
              {4, 1}, {4, 0}, {4, -1}, {3, -2},
              {2, -3}, {1, -4}, {0, -4}, {-1, -4},
              {-2, -3}, {-3, -2}, {-4, -1}, {-4, 0},
              {-4, 1}, {-3, 2}, {-2, 3}, {-1, 4}} {

    // Initialize Surface of Active Events to 0-timestamp
    //初始化SAE为0时刻。SAE中的元素为时刻
    //sae_与sae_latest_为当前时刻与上一时刻的SAE
    ROS_INFO("self-define");
}

//自定义了一个新的构造函数
ArcStarDetector::ArcStarDetector(int col, int row):
    kSmallCircle_{{0, 3}, {1, 3}, {2, 2}, {3, 1},
              {3, 0}, {3, -1}, {2, -2}, {1, -3},
              {0, -3}, {-1, -3}, {-2, -2}, {-3, -1},
              {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}},
    kLargeCircle_{{0, 4}, {1, 4}, {2, 3}, {3, 2},
              {4, 1}, {4, 0}, {4, -1}, {3, -2},
              {2, -3}, {1, -4}, {0, -4}, {-1, -4},
              {-2, -3}, {-3, -2}, {-4, -1}, {-4, 0},
              {-4, 1}, {-3, 2}, {-2, 3}, {-1, 4}},
               kSensorWidth_{col},
              kSensorHeight_{row}{

    // Initialize Surface of Active Events to 0-timestamp
    ROS_INFO("fuck!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11");
    ROS_INFO("kSensorWidth_=%d",kSensorWidth_);
    ROS_INFO("kSensorHeight_=%d",kSensorHeight_);

    // kSensorWidth_=col;
    // kSensorHeight_=row;
    sae_[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_latest_[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_latest_[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
}

ArcStarDetector::~ArcStarDetector() {
}

void ArcStarDetector::init(int col, int row){

    kSensorWidth_=col;
    kSensorHeight_=row;
    sensor_size_ = cv::Size(kSensorWidth_, kSensorHeight_);//用于timesurface
    sae_[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_latest_[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_latest_[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);

    decay_ms_=para_decay_ms;//延迟 30
    // decay_ms_for_loop=para_decay_loop_ms;//先不使用
    ignore_polarity_= (bool) para_ignore_polarity;//true;
    median_blur_kernel_size_=para_median_blur_kernel_size;//1;
    filter_threshold_=para_feature_filter_threshold;//处理的间隔时间

    // cur_event_mat=cv::Mat::zeros(sensor_size_, CV_8UC3);//当前的event转换为map(在调用处清空一下)

}

void ArcStarDetector::init(int col, int row, const double fx, const double fy, const double cx, const double cy){

    kSensorWidth_=col;
    kSensorHeight_=row;
    sensor_size_ = cv::Size(kSensorWidth_, kSensorHeight_);//用于timesurface
    sae_[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_latest_[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_latest_[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);

    decay_ms_=para_decay_ms;//延迟 30
    // decay_ms_for_loop=para_decay_loop_ms;//先不使用
    ignore_polarity_= (bool) para_ignore_polarity;//true;
    median_blur_kernel_size_=para_median_blur_kernel_size;//1;
    filter_threshold_=para_feature_filter_threshold;//处理的间隔时间

    // cur_event_mat=cv::Mat::zeros(sensor_size_, CV_8UC3);//当前的event转换为map(在调用处清空一下)
    // 内参矩阵
    intrinsics_matrix << float (fx), 0.,         float (cx),
                         0.,         float (fy), float (cy),
                         0.,         0.,          1.;
}

void ArcStarDetector::createSAE(double et, int ex, int ey, bool ep, const Motion_correction_value measurements){

      // 加入运动补偿的操作
      const double const_et=et;
      const double const_ex=ex;
      const double const_ey=ey;

      Eigen::Vector4d State=measurements.second.first.first;
      Eigen::Vector3f temp_v_a=measurements.second.first.second;//状态的加速度
      Eigen::Vector3f tmp_v;//状态的速度
      tmp_v[0]=State[0];
      tmp_v[1]=State[1];
      tmp_v[2]=State[2];
      double time_temp_v=State[3];

      Eigen::Vector3f accel_avg_=measurements.second.second.second.first;//当前IMU的加速度
      Eigen::Vector3f omega_avg_=measurements.second.second.second.second;//当前imu的角速度

      double t_0_event=measurements.second.second.first[0];//第一个事件的时间
      double t_0_eventstream=measurements.second.second.first[1];//当前事件流的时间
      double t_0=t_0_eventstream;//采用当前的事件信息流来进行运动补偿
      const double dt=et-t_0;

      // if(dt<0){
      //     std::cout<<"（measurements）比第一个事件要早？？？:"<<dt<<std::endl;
      // }
      // if(dt>0.03){
      //     std::cout<<"（measurements）比第一个事件要晚大于0.03:"<<dt<<std::endl;
      // }

      if(dt>t_motion_compensation_threshold){//大于0.01才进行运动补偿
        Eigen::Vector2d correct_coordinate= motioncorrection(const_ex,const_ey,tmp_v,accel_avg_,omega_avg_,dt);
        ex=correct_coordinate[0];
        ey=correct_coordinate[1];
      }
      // Eigen::Vector2d correct_coordinate= motioncorrection(const_ex,const_ey,tmp_v,accel_avg_,omega_avg_,dt);
      // ex=correct_coordinate[0];
      // ey=correct_coordinate[1];



      // Update Surface of Active Events  (更新SAE放此处)
      const int pol = ep ? 1 : 0;//若为正极性则为1，反之为0
      const int pol_inv = (!ep) ? 1 : 0;//若为逆极性则为1，反之为0（刚好是上面的逆）
                  //举例子：当前输入为正极性，那么pol=1；pol_inv为0
                  //当前输入为负极性，那么pol_inv=1；pol为0
      double & t_last = sae_latest_[pol](ex,ey);//存放当前极性的时间
      double & t_last_inv = sae_latest_[pol_inv](ex, ey);//存放当前极性的逆的时间

      // Filter blocks redundant spikes (consecutive and in short time) of the same polarity
      // This filter is required if the detector is to operate with corners with a majority of newest elements in the circles
      if ((et > t_last + filter_threshold_) || (t_last_inv > t_last) ) {//若当前事件的时间大于上一次同极性的输入时间+阈值 或者 上一次相同极性的时间少于当前极性逆极性的时间
        // 进行赋值（貌似只有sae_latest_才是保证一直被赋值）
        t_last = et;//当前event的时间为t_last.这个操作是不是也是相当于对sae_latest_赋值了???
        sae_[pol](ex, ey) = et;//给当前的SAE赋值               sae【极性】在位置ex与ey处的值=时间
      } else {//不满足条件就单纯只更新sae_latest_.   sae_latest_的更新只是为了选择而已,故此对于time surface,理论上应该选择sae_
        t_last = et;//保证对sae_latest_赋值
      }

      // last_event_time=last_event_time>et ? last_event_time : et;
      //若last_event_time>et，那么当前的et不是最新的，选last_event_time
      //若last_event_time<et，那么当前的et是最新的，选et给last_event_time

      //将当前event转换为cv::Mat
      cur_event_mat.at<cv::Vec3b>(cv::Point(ex,ey)) = (
            ep == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));//正极性为蓝色，负极性为红色
}

void ArcStarDetector::createSAE(double et, int ex, int ey, bool ep){

      // Update Surface of Active Events  (更新SAE放此处)
      const int pol = ep ? 1 : 0;//若为正极性则为1，反之为0
      const int pol_inv = (!ep) ? 1 : 0;//若为逆极性则为1，反之为0（刚好是上面的逆）
                  //举例子：当前输入为正极性，那么pol=1；pol_inv为0
                  //当前输入为负极性，那么pol_inv=1；pol为0
      double & t_last = sae_latest_[pol](ex,ey);//存放当前极性的时间
      double & t_last_inv = sae_latest_[pol_inv](ex, ey);//存放当前极性的逆的时间

      // Filter blocks redundant spikes (consecutive and in short time) of the same polarity
      // This filter is required if the detector is to operate with corners with a majority of newest elements in the circles
      if ((et > t_last + filter_threshold_) || (t_last_inv > t_last) ) {//若当前事件的时间大于上一次同极性的输入时间+阈值 或者 上一次相同极性的时间少于当前极性逆极性的时间
        // 进行赋值（貌似只有sae_latest_才是保证一直被赋值）
        t_last = et;//当前event的时间为t_last.这个操作是不是也是相当于对sae_latest_赋值了???
        sae_[pol](ex, ey) = et;//给当前的SAE赋值               sae【极性】在位置ex与ey处的值=时间
      } else {//不满足条件就单纯只更新sae_latest_.   sae_latest_的更新只是为了选择而已,故此对于time surface,理论上应该选择sae_
        t_last = et;//保证对sae_latest_赋值
      }

      // last_event_time=last_event_time>et ? last_event_time : et;
      //若last_event_time>et，那么当前的et不是最新的，选last_event_time
      //若last_event_time<et，那么当前的et是最新的，选et给last_event_time

      //将当前event转换为cv::Mat
      cur_event_mat.at<cv::Vec3b>(cv::Point(ex,ey)) = (
            ep == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));//正极性为蓝色，负极性为红色
}

cv::Mat ArcStarDetector::SAEtoTimeSurface( const double external_sync_time){//注意必须要在初始化之后！
//external_sync_time为产生timesurface的时间
//而SAE(sae_)中的时间,已经是double(输入参数为e.ts.toSec())

    // create exponential-decayed Time Surface map.
    const double decay_sec = decay_ms_ / 1000.0;//将毫秒转换为秒
    cv::Mat time_surface_map;
    time_surface_map=cv::Mat::zeros(sensor_size_, CV_64F);

      // Loop through all coordinates
      //对于cv::Mat先从y开始赋值
      for (int y=0;y<sensor_size_.height;++y){
              for(int x=0;x<sensor_size_.width;++x){

                    double most_recent_stamp_at_coordXY= (sae_[1](x,y)>sae_[0](x,y)) ? sae_[1](x,y) : sae_[0](x,y);//从SAE中取值
                    // most_recent_stamp_at_coordXY为sae_[1](x,y)与sae_[0](x,y)两者中最大的一个
                    //若当前像素最近发生的是正极性(1),那么sae_[1](x,y)>sae_[0](x,y)成立,那么选择sae_[1](x,y)

                    if(most_recent_stamp_at_coordXY > 0){//确认一下SAE中这个时间是否大于0
                          const double dt = (external_sync_time-most_recent_stamp_at_coordXY);
                          double expVal =std::exp(-dt / decay_sec);

                          if(!ignore_polarity_)//若不忽略极性，则会与极性的正负相乘
                            {
                                // /获取当前极性
                                //sae_中正极性则为1，反之为0
                                //若sae_[1](x,y)>sae_[0](x,y)成立,即当前像素点最晚发生的是正,若不成立,就是负
                                double polarity = (sae_[1](x,y)>sae_[0](x,y)) ? 1.0 : -1.0;  

                                expVal *= polarity;
                            }
                            time_surface_map.at<double>(y,x) = expVal;//给time_surface_map赋值
                    }
              }
      }

       // polarity
      if(!ignore_polarity_)//若不忽略极性，则执行，加上1这个偏置，起码不为0
        time_surface_map = 255.0 * (time_surface_map + 1.0) / 2.0;
      else
        time_surface_map = 255.0 * time_surface_map;
      time_surface_map.convertTo(time_surface_map, CV_8U);//转换格式

      // median blur
      if(median_blur_kernel_size_ > 0){
          cv::medianBlur(time_surface_map, time_surface_map, 2 * median_blur_kernel_size_ + 1);
      }

    return time_surface_map;
}

cv::Mat ArcStarDetector::SAE_Last_toTimeSurface( const double external_sync_time){//注意必须要在初始化之后！
//external_sync_time为产生timesurface的时间
    // create exponential-decayed Time Surface map.
    const double decay_sec = decay_ms_ / 1000.0;//将毫秒转换为秒
    cv::Mat time_surface_map;
    time_surface_map=cv::Mat::zeros(sensor_size_, CV_64F);

      // Loop through all coordinates
      //对于cv::Mat先从y开始赋值
      for (int y=0;y<sensor_size_.height;++y){
              for(int x=0;x<sensor_size_.width;++x){
                    //采用sae_latest_
                    double most_recent_stamp_at_coordXY= (sae_latest_[1](x,y)>sae_latest_[0](x,y)) ? sae_latest_[1](x,y) : sae_latest_[0](x,y);//从SAE中取值
                    // most_recent_stamp_at_coordXY为sae_[1](x,y)与sae_[0](x,y)两者中最大的一个
                    //若当前像素最近发生的是正极性(1),那么sae_[1](x,y)>sae_[0](x,y)成立,那么选择sae_[1](x,y)

                    if(most_recent_stamp_at_coordXY > 0){//确认一下SAE中这个时间是否大于0
                          const double dt = (external_sync_time-most_recent_stamp_at_coordXY);
                          double expVal =std::exp(-dt / decay_sec);
                          if(!ignore_polarity_)//若不忽略极性，则会与极性的正负相乘
                            {
                                // /获取当前极性
                                //sae_中正极性则为1，反之为0
                                //若sae_[1](x,y)>sae_[0](x,y)成立,即当前像素点最晚发生的是正,若不成立,就是负
                                double polarity = (sae_latest_[1](x,y)>sae_latest_[0](x,y)) ? 1.0 : -1.0;  //1为正，0为负
                                expVal *= polarity;
                            }
                            time_surface_map.at<double>(y,x) = expVal;//给time_surface_map赋值
                    }
              }
      }

       // polarity
      if(!ignore_polarity_)//若不忽略极性，则执行，加上1这个偏置，起码不为0
        time_surface_map = 255.0 * (time_surface_map + 1.0) / 2.0;
      else
        time_surface_map = 255.0 * time_surface_map;
      time_surface_map.convertTo(time_surface_map, CV_8U);//转换格式

      // median blur
      if(median_blur_kernel_size_ > 0){
          cv::medianBlur(time_surface_map, time_surface_map, 2 * median_blur_kernel_size_ + 1);
      }

    return time_surface_map;
}


cv::Mat ArcStarDetector::SAE_Last_toTimeSurface_withoutP( const double external_sync_time){//注意必须要在初始化之后！
//external_sync_time为产生timesurface的时间
    // create exponential-decayed Time Surface map.
    const double decay_sec = decay_ms_ / 1000.0;//将毫秒转换为秒
    cv::Mat time_surface_map;
    time_surface_map=cv::Mat::zeros(sensor_size_, CV_64F);

      // Loop through all coordinates
      //对于cv::Mat先从y开始赋值
      for (int y=0;y<sensor_size_.height;++y){
              for(int x=0;x<sensor_size_.width;++x){
                    //采用sae_latest_
                    double most_recent_stamp_at_coordXY= (sae_latest_[1](x,y)>sae_latest_[0](x,y)) ? sae_latest_[1](x,y) : sae_latest_[0](x,y);//从SAE中取值,获取时间最大的(就是最新的了)
                    // most_recent_stamp_at_coordXY为sae_[1](x,y)与sae_[0](x,y)两者中最大的一个
                    //若当前像素最近发生的是正极性(1),那么sae_[1](x,y)>sae_[0](x,y)成立,那么选择sae_[1](x,y)

                    if(most_recent_stamp_at_coordXY > 0){//确认一下SAE中这个时间是否大于0
                          const double dt = (external_sync_time-most_recent_stamp_at_coordXY);
                          double expVal =std::exp(-dt / decay_sec);
                          time_surface_map.at<double>(y,x) = expVal;//给time_surface_map赋值(忽略了极性)
                    }
              }
      }
      time_surface_map = 255.0 * time_surface_map;
      time_surface_map.convertTo(time_surface_map, CV_8U);//转换格式

      // median blur
      if(median_blur_kernel_size_ > 0){
          cv::medianBlur(time_surface_map, time_surface_map, 2 * median_blur_kernel_size_ + 1);
      }

    return time_surface_map;
}

cv::Mat ArcStarDetector::SAE_toTimeSurface_withoutP( const double external_sync_time){//注意必须要在初始化之后！
//external_sync_time为产生timesurface的时间
    // create exponential-decayed Time Surface map.
    const double decay_sec = decay_ms_ / 1000.0;//将毫秒转换为秒
    cv::Mat time_surface_map;
    time_surface_map=cv::Mat::zeros(sensor_size_, CV_64F);

      // Loop through all coordinates
      //对于cv::Mat先从y开始赋值
      for (int y=0;y<sensor_size_.height;++y){
              for(int x=0;x<sensor_size_.width;++x){
                    //采用sae_
                    double most_recent_stamp_at_coordXY= (sae_[1](x,y)>sae_[0](x,y)) ? sae_[1](x,y) : sae_[0](x,y);//从SAE中取值,获取时间最大的(就是最新的了)
                    // most_recent_stamp_at_coordXY为sae_[1](x,y)与sae_[0](x,y)两者中最大的一个
                    //若当前像素最近发生的是正极性(1),那么sae_[1](x,y)>sae_[0](x,y)成立,那么选择sae_[1](x,y)

                    if(most_recent_stamp_at_coordXY > 0){//确认一下SAE中这个时间是否大于0
                          const double dt = (external_sync_time-most_recent_stamp_at_coordXY);
                          double expVal =std::exp(-dt / decay_sec);
                          time_surface_map.at<double>(y,x) = expVal;//给time_surface_map赋值(忽略了极性)
                    }
              }
      }
      time_surface_map = 255.0 * time_surface_map;
      time_surface_map.convertTo(time_surface_map, CV_8U);//转换格式

      // median blur
      if(median_blur_kernel_size_ > 0){
          cv::medianBlur(time_surface_map, time_surface_map, 2 * median_blur_kernel_size_ + 1);
      }
      
      //保证像素强度的一致性
      cv::medianBlur(time_surface_map, time_surface_map, 3);//进行模糊处理很有必要
      cv::normalize(time_surface_map, time_surface_map, 0, 255, cv::NORM_MINMAX);//cv::NORM_MINMAX   cv::NORM_INF

      //加入轮廓提取，是否有利于回环？
      // cv::medianBlur(time_surface_map, time_surface_map, 3);//提取轮廓前需要滤波
      // cv::Mat h1_kernel = (cv::Mat_<char>(3, 3) << -1, -1, -1, -1, 8, -1, -1, -1, -1);
      // cv::filter2D(time_surface_map, time_surface_map, time_surface_map.type(), h1_kernel);

      //canny边缘检测
      // cv::blur(time_surface_map, time_surface_map, cv::Size(3, 3));
      // Canny(time_surface_map, time_surface_map, 3, 9, 3);

    return time_surface_map;
}


cv::Mat time_surface_map_multi_thread;
void time_surface_multi_thread(double _external_sync_time,Eigen::MatrixXd _sae_[2],double _decay_sec,int beginindex,int length){
  for(int y=0;y<ROW;++y){
    for(int x=beginindex;x<beginindex+length;++x){
      double most_recent_stamp_at_coordXY= (_sae_[1](x,y)>_sae_[0](x,y)) ? _sae_[1](x,y) : _sae_[0](x,y);
      if(most_recent_stamp_at_coordXY > 0){//确认一下SAE中这个时间是否大于0
        const double dt = (_external_sync_time-most_recent_stamp_at_coordXY);
        double expVal =std::exp(-dt / _decay_sec);
        time_surface_map_multi_thread.at<double>(y,x) = expVal;//给time_surface_map赋值(忽略了极性)
      }
    }
  }
}

cv::Mat ArcStarDetector::SAE_toTimeSurface_withoutP_multi_thread( const double external_sync_time){//注意必须要在初始化之后！
//external_sync_time为产生timesurface的时间
    // create exponential-decayed Time Surface map.
    const double decay_sec = decay_ms_ / 1000.0;//将毫秒转换为秒
    // const double decay_sec = (decay_ms_/4) / 1000.0;//将毫秒转换为秒（让拖尾少一些）
    cv::Mat time_surface_map;
    time_surface_map_multi_thread=cv::Mat::zeros(sensor_size_, CV_64F);//每次调用先清空

    if(Num_of_thread>=4){//当设置>=4,此处才会使用多线程
      int threadCount = Num_of_thread/2;//2个线程
      std::thread threads_timesurface[threadCount];   
      for (int i = 0; i < threadCount; i++)
      {
        //为每个线程分配任务
        int beginIndex = i*sensor_size_.width/threadCount;
        int length=sensor_size_.width/threadCount;
// void time_surface_multi_thread(const double _external_sync_time,const Eigen::MatrixXd _sae_0,const Eigen::MatrixXd _sae_1,cv::Mat &_time_surface_map,const double _decay_sec,const int beginindex, const int length)
        threads_timesurface[i] = std::thread(time_surface_multi_thread,external_sync_time,sae_,decay_sec,beginIndex,length);
      }
      //等待所有线程结束
      for(auto& thread_f:threads_timesurface)
          if(thread_f.joinable())
              thread_f.join();// openmp
    }
    else{
      for (int y=0;y<sensor_size_.height;++y){
        for(int x=0;x<sensor_size_.width;++x){
            //采用sae_
            double most_recent_stamp_at_coordXY= (sae_[1](x,y)>sae_[0](x,y)) ? sae_[1](x,y) : sae_[0](x,y);//从SAE中取值,获取时间最大的(就是最新的了)
            // most_recent_stamp_at_coordXY为sae_[1](x,y)与sae_[0](x,y)两者中最大的一个
            //若当前像素最近发生的是正极性(1),那么sae_[1](x,y)>sae_[0](x,y)成立,那么选择sae_[1](x,y)

            if(most_recent_stamp_at_coordXY > 0){//确认一下SAE中这个时间是否大于0
                  const double dt = (external_sync_time-most_recent_stamp_at_coordXY);
                  double expVal =std::exp(-dt / decay_sec);
                  time_surface_map_multi_thread.at<double>(y,x) = expVal;//给time_surface_map赋值(忽略了极性)
            }
        }
      }
    }
      //完成线程后，赋值
      time_surface_map = 255.0 * time_surface_map_multi_thread;
      // cv::normalize(time_surface_map, time_surface_map, 0.0, 255.0, cv::NORM_MINMAX);//先归一化再转换图像格式cv::NORM_MINMAX
      time_surface_map.convertTo(time_surface_map, CV_8U);//转换格式

      // median blur
      if(median_blur_kernel_size_ > 0){
          cv::medianBlur(time_surface_map, time_surface_map, 2 * median_blur_kernel_size_ + 1);
      }
      
      //保证像素强度的一致性
      cv::medianBlur(time_surface_map, time_surface_map, 3);//进行模糊处理很有必要

      //做直方图均衡处理
      // cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();//默认参数
      cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));//VINS-MONO用的均衡
      clahe->apply(time_surface_map, time_surface_map);

      //轮廓提取
      // cv::Mat h1_kernel = (cv::Mat_<char>(3, 3) << -1, -1, -1, -1, 8, -1, -1, -1, -1);
      // filter2D(time_surface_map, time_surface_map, time_surface_map.type(), h1_kernel);

      // cv::normalize(time_surface_map, time_surface_map, 0, 255, cv::NORM_MINMAX);//cv::NORM_MINMAX   cv::NORM_INF

    return time_surface_map;
}


bool ArcStarDetector::isCorner(double et, int ex, int ey, bool ep) {//这个是最主要的函数

    //(全部SAE的更新放在了SAEtoTimeSurface中实现) Update Surface of Active Events
    const int pol = ep ? 1 : 0;//若为正极性则为1，反之为0
    const int pol_inv = (!ep) ? 1 : 0;//若为逆极性则为1，反之为0
    //举例子：当前输入为正极性，那么pol=1；pol_inv为0
    //当前输入为负极性，那么pol_inv=1；pol为0
    double & t_last = sae_latest_[pol](ex,ey);//分别存放两种极性，上一时刻的sae在当前点及极性下的时间
    double & t_last_inv = sae_latest_[pol_inv](ex, ey);//分别存放两种极性

    // Filter blocks redundant spikes (consecutive and in short time) of the same polarity
    // This filter is required if the detector is to operate with corners with a majority of newest elements in the circles
    if ((et > t_last + filter_threshold_) || (t_last_inv > t_last) ) {//若当前事件的时间大于上一次的输入时间+阈值或者逆极性的时间大于上一次正极性的时间
      // 进行赋值
      //上面也对sae_latest_进行了更新了,此处就不需要更新了吧 
      // t_last = et;//当前event的时间为t_last

      // 上面更新了sae_,这里就不需要更新了吧
      // sae_[pol](ex, ey) = et;//给当前的SAE赋值               sae【极性】在位置ex与ey处的值=时间
    } else {//否则就记录当前的时间然后返回不是角点，但仍然记录了sae_latest_？

      // t_last = et;//更新sae_latest_,同样的,上面已经进行了更新了,所以不需要了

      return false;//不满足条件则不检测角点
    }

    //多加一个约束

    // Return if too close to the border（边缘点）
    // const int kBorderLimit = 4;//默认
    // const int kBorderLimit = 10;
     const int kBorderLimit = MIN_DIST+1;//选取特征点之间的最小距离
    if (ex < kBorderLimit || ex >= (kSensorWidth_ - kBorderLimit) ||
        ey < kBorderLimit || ey >= (kSensorHeight_ - kBorderLimit)) {
      return false;//太边缘的不选
    }

    // Define constant and thresholds
    //定义大圆与小圆
    const int kSmallCircleSize = 16;
    const int kLargeCircleSize = 20;
    //定义判断是否为角点的阈值
    // const int kSmallMinThresh = 3;//小圆的阈值
    // const int kSmallMaxThresh = 6;//
    // const int kLargeMinThresh = 4;//大圆的阈值
    // const int kLargeMaxThresh = 8;
    const int kSmallMinThresh = 4;//小圆的阈值
    const int kSmallMaxThresh = 6;//
    const int kLargeMinThresh = 5;//大圆的阈值
    const int kLargeMaxThresh = 8;


    bool is_arc_valid = false;//初始化这个值，用于判断是否为角点

    // Small Circle exploration  开始从小圆探索
    // Initialize arc from newest element
    double segment_new_min_t = sae_[pol](ex+kSmallCircle_[0][0], ey+kSmallCircle_[0][1]);// const int kSmallCircle_[16][2]是一个静态二维数组，这里其实就是用了{0, 3}作为起始点

    // Left and Right are equivalent to CW and CCW as in the paper
    int arc_right_idx = 0;//CW
    int arc_left_idx;//CCW

    // Find newest
    for (int i=1; i<kSmallCircleSize; i++) {//遍历16个单元
      const double t =sae_[pol](ex+kSmallCircle_[i][0], ey+kSmallCircle_[i][1]);//每一处的sae的时间t
      if (t > segment_new_min_t) {//若大于第一个的值，相当于最新的时间
        segment_new_min_t = t;//当前最大的
        arc_right_idx = i; // % End up in the maximum value   获取是第几个元素。arc_right_idx为新检索的，连续大于初始点的数目
      }
    }
    // Shift to the sides of the newest element;
    arc_left_idx = (arc_right_idx-1+kSmallCircleSize)%kSmallCircleSize;
    arc_right_idx= (arc_right_idx+1)%kSmallCircleSize;
    double arc_left_value = sae_[pol](ex+kSmallCircle_[arc_left_idx][0], ey+kSmallCircle_[arc_left_idx][1]);
    double arc_right_value = sae_[pol](ex+kSmallCircle_[arc_right_idx][0], ey+kSmallCircle_[arc_right_idx][1]);
    double arc_left_min_t = arc_left_value;
    double arc_right_min_t = arc_right_value;

    // Expand
    // Initial expand does not require checking
    int iteration = 1; // The arc already contain the maximum
    for (; iteration<kSmallMinThresh; iteration++) {
      // Decide the most promising arc
      if (arc_right_value > arc_left_value) { // Right arc
        if (arc_right_min_t < segment_new_min_t) {
            segment_new_min_t = arc_right_min_t;
        }
        // Expand arc
        arc_right_idx= (arc_right_idx+1)%kSmallCircleSize;
        arc_right_value = sae_[pol](ex+kSmallCircle_[arc_right_idx][0], ey+kSmallCircle_[arc_right_idx][1]);
        if (arc_right_value < arc_right_min_t) { // Update minimum of the arc
          arc_right_min_t = arc_right_value;
        }
      } else { // Left arc
        // Include arc in new segment
        if (arc_left_min_t < segment_new_min_t) {
          segment_new_min_t = arc_left_min_t;
        }

        // Expand arc
        arc_left_idx= (arc_left_idx-1+kSmallCircleSize)%kSmallCircleSize;
        arc_left_value = sae_[pol](ex+kSmallCircle_[arc_left_idx][0], ey+kSmallCircle_[arc_left_idx][1]);
        if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
          arc_left_min_t = arc_left_value;
        }
      }
    }
    int newest_segment_size = kSmallMinThresh;

    // Further expand until completion of the circle
    for (; iteration<kSmallCircleSize; iteration++) {
      // Decide the most promising arc
      if (arc_right_value > arc_left_value) { // Right arc
        // Include arc in new segment
        if ((arc_right_value >=  segment_new_min_t)) {
          newest_segment_size = iteration+1; // Check
          if (arc_right_min_t < segment_new_min_t) {
            segment_new_min_t = arc_right_min_t;
          }
        }

        // Expand arc
        arc_right_idx= (arc_right_idx+1)%kSmallCircleSize;
        arc_right_value = sae_[pol](ex+kSmallCircle_[arc_right_idx][0], ey+kSmallCircle_[arc_right_idx][1]);
        if (arc_right_value < arc_right_min_t) { // Update minimum of the arc
          arc_right_min_t = arc_right_value;
        }
      } else { // Left arc
        // Include arc in new segment
        if ((arc_left_value >=  segment_new_min_t)) {
          newest_segment_size = iteration+1;
          if (arc_left_min_t < segment_new_min_t) {
            segment_new_min_t = arc_left_min_t;
          }
        }

        // Expand arc
        arc_left_idx= (arc_left_idx-1+kSmallCircleSize)%kSmallCircleSize;
        arc_left_value = sae_[pol](ex+kSmallCircle_[arc_left_idx][0], ey+kSmallCircle_[arc_left_idx][1]);
        if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
          arc_left_min_t = arc_left_value;
        }
      }
    }

    if (// Corners with newest segment of a minority of elements in the circle
        // These corners are equivalent to those in Mueggler et al. BMVC17
            (newest_segment_size <= kSmallMaxThresh) ||
        // Corners with newest segment of a majority of elements in the circle
        // This can be commented out to decrease noise at expenses of less repeatibility. If you do, DO NOT forget to comment the equilvent line in the large circle
        ((newest_segment_size >= (kSmallCircleSize - kSmallMaxThresh)) && (newest_segment_size <= (kSmallCircleSize - kSmallMinThresh)))) {
      is_arc_valid = true;
    }

    // Large Circle exploration
    if (is_arc_valid) {//若前面小圆已经判断是角点，则进行这里的操作
    is_arc_valid = false;

      segment_new_min_t = sae_[pol](ex+kLargeCircle_[0][0], ey+kLargeCircle_[0][1]);
      arc_right_idx = 0;

      // Initialize in the newest element
      for (int i=1; i<kLargeCircleSize; i++) {
        const double t =sae_[pol](ex+kLargeCircle_[i][0], ey+kLargeCircle_[i][1]);
        if (t > segment_new_min_t) {
          segment_new_min_t = t;
          arc_right_idx = i; // % End up in the maximum value
        }
      }
      // Shift to the sides of the newest elements;
      arc_left_idx = (arc_right_idx-1+kLargeCircleSize)%kLargeCircleSize;
      arc_right_idx= (arc_right_idx+1)%kLargeCircleSize;
      arc_left_value = sae_[pol](ex+kLargeCircle_[arc_left_idx][0],
                                 ey+kLargeCircle_[arc_left_idx][1]);
      arc_right_value = sae_[pol](ex+kLargeCircle_[arc_right_idx][0],
                                  ey+kLargeCircle_[arc_right_idx][1]);
      arc_left_min_t = arc_left_value;
      arc_right_min_t = arc_right_value;

      // Expand
      // Initial expand does not require checking
      iteration = 1;
      for (; iteration<kLargeMinThresh; iteration++) {
        // Decide the most promising arc
        if (arc_right_value > arc_left_value) { // Right arc
          if (arc_right_min_t < segment_new_min_t) {
              segment_new_min_t = arc_right_min_t;
          }
          // Expand arc
          arc_right_idx= (arc_right_idx+1)%kLargeCircleSize;
          arc_right_value = sae_[pol](ex+kLargeCircle_[arc_right_idx][0],
                                      ey+kLargeCircle_[arc_right_idx][1]);
          if (arc_right_value < arc_right_min_t) { // Update minimum of the arc
            arc_right_min_t = arc_right_value;
          }
        } else { // Left arc
          // Include arc in new segment
          if (arc_left_min_t < segment_new_min_t) {
            segment_new_min_t = arc_left_min_t;
          }

          // Expand arc
          arc_left_idx= (arc_left_idx-1+kLargeCircleSize)%kLargeCircleSize;
          arc_left_value = sae_[pol](ex+kLargeCircle_[arc_left_idx][0],
                                     ey+kLargeCircle_[arc_left_idx][1]);
          if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
            arc_left_min_t = arc_left_value;
          }
        }
      }
      newest_segment_size = kLargeMinThresh;

      // Further expand until completion of the circle
      for (; iteration<kLargeCircleSize; iteration++) {
        // Decide the most promising arc
        if (arc_right_value > arc_left_value) { // Right arc
          // Include arc in new segment
          if ((arc_right_value >=  segment_new_min_t)) {
            newest_segment_size = iteration+1;
            if (arc_right_min_t < segment_new_min_t) {
              segment_new_min_t = arc_right_min_t;
            }
          }

          // Expand arc
          arc_right_idx= (arc_right_idx+1)%kLargeCircleSize;
          arc_right_value = sae_[pol](ex+kLargeCircle_[arc_right_idx][0],
                                      ey+kLargeCircle_[arc_right_idx][1]);
          if (arc_right_value < arc_right_min_t) { // Update minimum of the arc
            arc_right_min_t = arc_right_value;
          }
        } else { // Left arc
          // Include arc in new segment
          if ((arc_left_value >=  segment_new_min_t)) {
            newest_segment_size = iteration+1;
            if (arc_left_min_t < segment_new_min_t) {
              segment_new_min_t = arc_left_min_t;
            }
          }

          // Expand arc
          arc_left_idx= (arc_left_idx-1+kLargeCircleSize)%kLargeCircleSize;
          arc_left_value = sae_[pol](ex+kLargeCircle_[arc_left_idx][0],
                                    ey+kLargeCircle_[arc_left_idx][1]);
          if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
            arc_left_min_t = arc_left_value;
          }
        }
      }

      if (// Corners with newest segment of a minority of elements in the circle
          // These corners are equivalent to those in Mueggler et al. BMVC17
              (newest_segment_size <= kLargeMaxThresh) ||
          // Corners with newest segment of a majority of elements in the circle
          // This can be commented out to decrease noise at expenses of less repeatibility. If you do, DO NOT forget to comment the equilvent line in the small circle
          (newest_segment_size >= (kLargeCircleSize - kLargeMaxThresh) && (newest_segment_size <= (kLargeCircleSize - kLargeMinThresh))) ) {
        is_arc_valid = true;
      }
    }

    return is_arc_valid;
}


//用于后端回环检测的特征
bool ArcStarDetector::isFeature(double et, int ex, int ey, bool ep) {
    // Update Surface of Active Events
    const int pol = ep ? 1 : 0;
    const int pol_inv = (!ep) ? 1 : 0;
    double & t_last = sae_latest_[pol](ex,ey);
    double & t_last_inv = sae_latest_[pol_inv](ex, ey);

    // Filter blocks redundant spikes (consecutive and in short time) of the same polarity
    // This filter is required if the detector is to operate with corners with a majority of newest elements in the circles
    if ((et > t_last + filter_threshold_) || (t_last_inv > t_last) ) {
      t_last = et;
      sae_[pol](ex, ey) = et;
    } else {
      t_last = et;
      return false;
    }

    // Return if too close to the border
    const int kBorderLimit = 10+2;//默认为4
    if (ex < kBorderLimit || ex >= (kSensorWidth_ - kBorderLimit) ||
        ey < kBorderLimit || ey >= (kSensorHeight_ - kBorderLimit)) {
      return false;
    }

    // Define constant and thresholds
    const int kSmallCircleSize = 16;
    const int kLargeCircleSize = 20;
    const int kSmallMinThresh = 3;
    const int kSmallMaxThresh = 6;
    const int kLargeMinThresh = 4;
    const int kLargeMaxThresh = 8;


    bool is_arc_valid = false;
    // Small Circle exploration
    // Initialize arc from newest element
    double segment_new_min_t = sae_[pol](ex+kSmallCircle_[0][0], ey+kSmallCircle_[0][1]);

    // Left and Right are equivalent to CW and CCW as in the paper
    int arc_right_idx = 0;
    int arc_left_idx;

    // Find newest
    for (int i=1; i<kSmallCircleSize; i++) {
      const double t =sae_[pol](ex+kSmallCircle_[i][0], ey+kSmallCircle_[i][1]);
      if (t > segment_new_min_t) {
        segment_new_min_t = t;
        arc_right_idx = i; // % End up in the maximum value
      }
    }
    // Shift to the sides of the newest element;
    arc_left_idx = (arc_right_idx-1+kSmallCircleSize)%kSmallCircleSize;
    arc_right_idx= (arc_right_idx+1)%kSmallCircleSize;
    double arc_left_value = sae_[pol](ex+kSmallCircle_[arc_left_idx][0], ey+kSmallCircle_[arc_left_idx][1]);
    double arc_right_value = sae_[pol](ex+kSmallCircle_[arc_right_idx][0], ey+kSmallCircle_[arc_right_idx][1]);
    double arc_left_min_t = arc_left_value;
    double arc_right_min_t = arc_right_value;

    // Expand
    // Initial expand does not require checking
    int iteration = 1; // The arc already contain the maximum
    for (; iteration<kSmallMinThresh; iteration++) {
      // Decide the most promising arc
      if (arc_right_value > arc_left_value) { // Right arc
        if (arc_right_min_t < segment_new_min_t) {
            segment_new_min_t = arc_right_min_t;
        }
        // Expand arc
        arc_right_idx= (arc_right_idx+1)%kSmallCircleSize;
        arc_right_value = sae_[pol](ex+kSmallCircle_[arc_right_idx][0], ey+kSmallCircle_[arc_right_idx][1]);
        if (arc_right_value < arc_right_min_t) { // Update minimum of the arc
          arc_right_min_t = arc_right_value;
        }
      } else { // Left arc
        // Include arc in new segment
        if (arc_left_min_t < segment_new_min_t) {
          segment_new_min_t = arc_left_min_t;
        }

        // Expand arc
        arc_left_idx= (arc_left_idx-1+kSmallCircleSize)%kSmallCircleSize;
        arc_left_value = sae_[pol](ex+kSmallCircle_[arc_left_idx][0], ey+kSmallCircle_[arc_left_idx][1]);
        if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
          arc_left_min_t = arc_left_value;
        }
      }
    }
    int newest_segment_size = kSmallMinThresh;

    // Further expand until completion of the circle
    for (; iteration<kSmallCircleSize; iteration++) {
      // Decide the most promising arc
      if (arc_right_value > arc_left_value) { // Right arc
        // Include arc in new segment
        if ((arc_right_value >=  segment_new_min_t)) {
          newest_segment_size = iteration+1; // Check
          if (arc_right_min_t < segment_new_min_t) {
            segment_new_min_t = arc_right_min_t;
          }
        }

        // Expand arc
        arc_right_idx= (arc_right_idx+1)%kSmallCircleSize;
        arc_right_value = sae_[pol](ex+kSmallCircle_[arc_right_idx][0], ey+kSmallCircle_[arc_right_idx][1]);
        if (arc_right_value < arc_right_min_t) { // Update minimum of the arc
          arc_right_min_t = arc_right_value;
        }
      } else { // Left arc
        // Include arc in new segment
        if ((arc_left_value >=  segment_new_min_t)) {
          newest_segment_size = iteration+1;
          if (arc_left_min_t < segment_new_min_t) {
            segment_new_min_t = arc_left_min_t;
          }
        }

        // Expand arc
        arc_left_idx= (arc_left_idx-1+kSmallCircleSize)%kSmallCircleSize;
        arc_left_value = sae_[pol](ex+kSmallCircle_[arc_left_idx][0], ey+kSmallCircle_[arc_left_idx][1]);
        if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
          arc_left_min_t = arc_left_value;
        }
      }
    }

    if (// Corners with newest segment of a minority of elements in the circle
        // These corners are equivalent to those in Mueggler et al. BMVC17
            (newest_segment_size <= kSmallMaxThresh) ||
        // Corners with newest segment of a majority of elements in the circle
        // This can be commented out to decrease noise at expenses of less repeatibility. If you do, DO NOT forget to comment the equilvent line in the large circle
        ((newest_segment_size >= (kSmallCircleSize - kSmallMaxThresh)) && (newest_segment_size <= (kSmallCircleSize - kSmallMinThresh)))) {
      is_arc_valid = true;
    }

    // Large Circle exploration
    if (is_arc_valid) {
    is_arc_valid = false;

      segment_new_min_t = sae_[pol](ex+kLargeCircle_[0][0], ey+kLargeCircle_[0][1]);
      arc_right_idx = 0;

      // Initialize in the newest element
      for (int i=1; i<kLargeCircleSize; i++) {
        const double t =sae_[pol](ex+kLargeCircle_[i][0], ey+kLargeCircle_[i][1]);
        if (t > segment_new_min_t) {
          segment_new_min_t = t;
          arc_right_idx = i; // % End up in the maximum value
        }
      }
      // Shift to the sides of the newest elements;
      arc_left_idx = (arc_right_idx-1+kLargeCircleSize)%kLargeCircleSize;
      arc_right_idx= (arc_right_idx+1)%kLargeCircleSize;
      arc_left_value = sae_[pol](ex+kLargeCircle_[arc_left_idx][0],
                                 ey+kLargeCircle_[arc_left_idx][1]);
      arc_right_value = sae_[pol](ex+kLargeCircle_[arc_right_idx][0],
                                  ey+kLargeCircle_[arc_right_idx][1]);
      arc_left_min_t = arc_left_value;
      arc_right_min_t = arc_right_value;

      // Expand
      // Initial expand does not require checking
      iteration = 1;
      for (; iteration<kLargeMinThresh; iteration++) {
        // Decide the most promising arc
        if (arc_right_value > arc_left_value) { // Right arc
          if (arc_right_min_t < segment_new_min_t) {
              segment_new_min_t = arc_right_min_t;
          }
          // Expand arc
          arc_right_idx= (arc_right_idx+1)%kLargeCircleSize;
          arc_right_value = sae_[pol](ex+kLargeCircle_[arc_right_idx][0],
                                      ey+kLargeCircle_[arc_right_idx][1]);
          if (arc_right_value < arc_right_min_t) { // Update minimum of the arc
            arc_right_min_t = arc_right_value;
          }
        } else { // Left arc
          // Include arc in new segment
          if (arc_left_min_t < segment_new_min_t) {
            segment_new_min_t = arc_left_min_t;
          }

          // Expand arc
          arc_left_idx= (arc_left_idx-1+kLargeCircleSize)%kLargeCircleSize;
          arc_left_value = sae_[pol](ex+kLargeCircle_[arc_left_idx][0],
                                     ey+kLargeCircle_[arc_left_idx][1]);
          if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
            arc_left_min_t = arc_left_value;
          }
        }
      }
      newest_segment_size = kLargeMinThresh;

      // Further expand until completion of the circle
      for (; iteration<kLargeCircleSize; iteration++) {
        // Decide the most promising arc
        if (arc_right_value > arc_left_value) { // Right arc
          // Include arc in new segment
          if ((arc_right_value >=  segment_new_min_t)) {
            newest_segment_size = iteration+1;
            if (arc_right_min_t < segment_new_min_t) {
              segment_new_min_t = arc_right_min_t;
            }
          }

          // Expand arc
          arc_right_idx= (arc_right_idx+1)%kLargeCircleSize;
          arc_right_value = sae_[pol](ex+kLargeCircle_[arc_right_idx][0],
                                      ey+kLargeCircle_[arc_right_idx][1]);
          if (arc_right_value < arc_right_min_t) { // Update minimum of the arc
            arc_right_min_t = arc_right_value;
          }
        } else { // Left arc
          // Include arc in new segment
          if ((arc_left_value >=  segment_new_min_t)) {
            newest_segment_size = iteration+1;
            if (arc_left_min_t < segment_new_min_t) {
              segment_new_min_t = arc_left_min_t;
            }
          }

          // Expand arc
          arc_left_idx= (arc_left_idx-1+kLargeCircleSize)%kLargeCircleSize;
          arc_left_value = sae_[pol](ex+kLargeCircle_[arc_left_idx][0],
                                    ey+kLargeCircle_[arc_left_idx][1]);
          if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
            arc_left_min_t = arc_left_value;
          }
        }
      }

      if (// Corners with newest segment of a minority of elements in the circle
          // These corners are equivalent to those in Mueggler et al. BMVC17
              (newest_segment_size <= kLargeMaxThresh) ||
          // Corners with newest segment of a majority of elements in the circle
          // This can be commented out to decrease noise at expenses of less repeatibility. If you do, DO NOT forget to comment the equilvent line in the small circle
          (newest_segment_size >= (kLargeCircleSize - kLargeMaxThresh) && (newest_segment_size <= (kLargeCircleSize - kLargeMinThresh))) ) {
        is_arc_valid = true;
      }
    }

    return is_arc_valid;
}


Eigen::Vector2d ArcStarDetector::motioncorrection(const double ex,const double ey,const Eigen::Vector3f tmp_v,const Eigen::Vector3f accel_avg_, const Eigen::Vector3f omega_avg_,const double dt)
{
  Eigen::Vector2d correct_coordinate;
  const int kBorder = 6;

  if(ex > kBorder && ex <= (kSensorWidth_ - kBorder) &&
      ey > kBorder && ey <= (kSensorHeight_ - kBorder) ){//只对非边界点进行处理

      Eigen::Vector3f rotation_vector=omega_avg_ * dt;//从t_0到et的旋转向量
      Eigen::Matrix3f rot_skew_mat = vectorToSkewMat(rotation_vector);
      Eigen::Matrix3f rotation_matrix_ = rot_skew_mat.exp();  // vector space to Lee spin space
      //当前仍然是camera坐标系的
      // 变换到像素坐标系
      // Eigen::Matrix3f rot_K =K.inverse() * rotation_matrix_* K;//pixel 从t_0到et的旋转向量
      //若要将et转换到t_0则为：
      // rot_K=rot_K.transpose();
      Eigen::Matrix3f rot_K =intrinsics_matrix *rotation_matrix_.transpose()* intrinsics_matrix.inverse();//pixel 从et到t_0的旋转向量

      // ROS_WARN_STREAM("intrinsics_matrix : " << std::endl << intrinsics_matrix);

      // s=vt+0.5*a*t*t;
      Eigen::Vector3f trans_K=dt*tmp_v+0.5*dt*dt*accel_avg_;
      trans_K=-rot_K*(intrinsics_matrix.inverse()*trans_K);      

      //变换前的点
      Eigen::Vector3f eventVec;
      eventVec[0] = ex;
      eventVec[1] = ey;
      eventVec[2] = 1;
      eventVec = rot_K * eventVec+trans_K;  // event warp  转换回t_0
      // eventVec = rot_K * eventVec;  // 瞬间的位移量很少
      ConvertToHomogeneous(&eventVec);//要重新归一化

      int x_coordinate = std::floor(eventVec[0]);
      int y_coordinate = std::floor(eventVec[1]);

      // std::cout<<"x_coordinate="<<x_coordinate<<"y_coordinate="<<y_coordinate<<std::endl;

      if(x_coordinate>0 && x_coordinate< kSensorWidth_ -1&& y_coordinate >0 && y_coordinate <kSensorHeight_-1)//防止越界
      {
        correct_coordinate[0]=x_coordinate;
        correct_coordinate[1]=y_coordinate;
      }    
      else{//越界点赋予边界值
        // std::cout<<"x_raw="<<ex<<", "<<"y_raw="<<ey<<std::endl;
        // std::cout<<"x_coordinate="<<x_coordinate<<", "<<"y_coordinate="<<y_coordinate<<std::endl;
        // std::cout<<"rotation_vector="<<rotation_vector[0]<<", "<<rotation_vector[1]<<", "<<rotation_vector[2]<<std::endl;
        // std::cout<<"-----------------------------------------------------------------------"<<std::endl;
        // return false;
        if(ex<kSensorWidth_/2)
          correct_coordinate[0]=0;
        else
          correct_coordinate[0]=kSensorWidth_-1;
        
        if(ey<kSensorHeight_/2)
          correct_coordinate[1]=0;
        else
          correct_coordinate[1]=kSensorHeight_-1;
      }
  
  }
  else{
    correct_coordinate[0]=ex;
    correct_coordinate[1]=ey;
  }

  return correct_coordinate;
}

bool ArcStarDetector::isFeature(double et, int ex, int ey, bool ep, const Motion_correction_value measurements) {

    // double t_0_event=measurements.second.second.first[0];
    // double dt_event=et-t_0_event;
    // if(dt_event<0){
    //     std::cout<<"（measurements）比第一个事件要早？？？:"<<dt_event<<std::endl;
    // }
    // if(dt_event>0.03){
    //     std::cout<<"（measurements）比第一个事件要晚大于0.03:"<<dt_event<<std::endl;
    // }

    //有后端优化，再做运动补偿
    // if(measurements.first){
      const double const_et=et;
      const double const_ex=ex;
      const double const_ey=ey;

      Eigen::Vector4d State=measurements.second.first.first;
      Eigen::Vector3f temp_v_a=measurements.second.first.second;//状态的加速度
      Eigen::Vector3f tmp_v;//状态的速度
      tmp_v[0]=State[0];
      tmp_v[1]=State[1];
      tmp_v[2]=State[2];
      double time_temp_v=State[3];

      Eigen::Vector3f accel_avg_=measurements.second.second.second.first;//当前IMU的加速度
      Eigen::Vector3f omega_avg_=measurements.second.second.second.second;//当前imu的角速度

      double t_0_event=measurements.second.second.first[0];//第一个事件的时间
      double t_0_eventstream=measurements.second.second.first[1];//当前事件流的时间
      double t_0=t_0_eventstream;
      const double dt=et-t_0;
      // if(dt<0){
      //     std::cout<<"（measurements）比第一个事件要早？？？:"<<dt<<std::endl;
      // }
      // if(dt>0.03){
      //     std::cout<<"（measurements）比第一个事件要晚大于0.03:"<<dt<<std::endl;
      // }

      // tmp_v=tmp_v+(t_0-time_temp_v)*accel_avg_;//速度更新

      if(dt>t_motion_compensation_threshold){//大于一定值的时候才进行运动补偿
        Eigen::Vector2d correct_coordinate= motioncorrection(const_ex,const_ey,tmp_v,accel_avg_,omega_avg_,dt);
        ex=correct_coordinate[0];
        ey=correct_coordinate[1];
      }
      // Eigen::Vector2d correct_coordinate= motioncorrection(const_ex,const_ey,tmp_v,accel_avg_,omega_avg_,dt);
      // ex=correct_coordinate[0];
      // ey=correct_coordinate[1];

    // }
  
    // Update Surface of Active Events
    const int pol = ep ? 1 : 0;//记录极性
    const int pol_inv = (!ep) ? 1 : 0;//记录极性的逆
    double & t_last = sae_latest_[pol](ex,ey);//当前极性所在的位置，之前事件的时间
    double & t_last_inv = sae_latest_[pol_inv](ex, ey);//当前逆极性所在的位置，之前事件的时间

    // Filter blocks redundant spikes (consecutive and in short time) of the same polarity
    // This filter is required if the detector is to operate with corners with a majority of newest elements in the circles
    if ((et > t_last + filter_threshold_) || (t_last_inv > t_last) ) {
      t_last = et;
      sae_[pol](ex, ey) = et;//更新当前极性的sae
    } else {
      t_last = et;
      return false;
    }

    // Return if too close to the border
    const int kBorderLimit = 10+2;//默认为4
    if (ex < kBorderLimit || ex >= (kSensorWidth_ - kBorderLimit) ||
        ey < kBorderLimit || ey >= (kSensorHeight_ - kBorderLimit)) {
      return false;
    }

    // Define constant and thresholds
    const int kSmallCircleSize = 16;
    const int kLargeCircleSize = 20;
    const int kSmallMinThresh = 3;
    const int kSmallMaxThresh = 6;
    const int kLargeMinThresh = 4;
    const int kLargeMaxThresh = 8;


    bool is_arc_valid = false;
    // Small Circle exploration
    // Initialize arc from newest element
    double segment_new_min_t = sae_[pol](ex+kSmallCircle_[0][0], ey+kSmallCircle_[0][1]);

    // Left and Right are equivalent to CW and CCW as in the paper
    int arc_right_idx = 0;
    int arc_left_idx;

    // Find newest
    for (int i=1; i<kSmallCircleSize; i++) {
      const double t =sae_[pol](ex+kSmallCircle_[i][0], ey+kSmallCircle_[i][1]);
      if (t > segment_new_min_t) {
        segment_new_min_t = t;
        arc_right_idx = i; // % End up in the maximum value
      }
    }
    // Shift to the sides of the newest element;
    arc_left_idx = (arc_right_idx-1+kSmallCircleSize)%kSmallCircleSize;
    arc_right_idx= (arc_right_idx+1)%kSmallCircleSize;
    double arc_left_value = sae_[pol](ex+kSmallCircle_[arc_left_idx][0], ey+kSmallCircle_[arc_left_idx][1]);
    double arc_right_value = sae_[pol](ex+kSmallCircle_[arc_right_idx][0], ey+kSmallCircle_[arc_right_idx][1]);
    double arc_left_min_t = arc_left_value;
    double arc_right_min_t = arc_right_value;

    // Expand
    // Initial expand does not require checking
    int iteration = 1; // The arc already contain the maximum
    for (; iteration<kSmallMinThresh; iteration++) {
      // Decide the most promising arc
      if (arc_right_value > arc_left_value) { // Right arc
        if (arc_right_min_t < segment_new_min_t) {
            segment_new_min_t = arc_right_min_t;
        }
        // Expand arc
        arc_right_idx= (arc_right_idx+1)%kSmallCircleSize;
        arc_right_value = sae_[pol](ex+kSmallCircle_[arc_right_idx][0], ey+kSmallCircle_[arc_right_idx][1]);
        if (arc_right_value < arc_right_min_t) { // Update minimum of the arc
          arc_right_min_t = arc_right_value;
        }
      } else { // Left arc
        // Include arc in new segment
        if (arc_left_min_t < segment_new_min_t) {
          segment_new_min_t = arc_left_min_t;
        }

        // Expand arc
        arc_left_idx= (arc_left_idx-1+kSmallCircleSize)%kSmallCircleSize;
        arc_left_value = sae_[pol](ex+kSmallCircle_[arc_left_idx][0], ey+kSmallCircle_[arc_left_idx][1]);
        if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
          arc_left_min_t = arc_left_value;
        }
      }
    }
    int newest_segment_size = kSmallMinThresh;

    // Further expand until completion of the circle
    for (; iteration<kSmallCircleSize; iteration++) {
      // Decide the most promising arc
      if (arc_right_value > arc_left_value) { // Right arc
        // Include arc in new segment
        if ((arc_right_value >=  segment_new_min_t)) {
          newest_segment_size = iteration+1; // Check
          if (arc_right_min_t < segment_new_min_t) {
            segment_new_min_t = arc_right_min_t;
          }
        }

        // Expand arc
        arc_right_idx= (arc_right_idx+1)%kSmallCircleSize;
        arc_right_value = sae_[pol](ex+kSmallCircle_[arc_right_idx][0], ey+kSmallCircle_[arc_right_idx][1]);
        if (arc_right_value < arc_right_min_t) { // Update minimum of the arc
          arc_right_min_t = arc_right_value;
        }
      } else { // Left arc
        // Include arc in new segment
        if ((arc_left_value >=  segment_new_min_t)) {
          newest_segment_size = iteration+1;
          if (arc_left_min_t < segment_new_min_t) {
            segment_new_min_t = arc_left_min_t;
          }
        }

        // Expand arc
        arc_left_idx= (arc_left_idx-1+kSmallCircleSize)%kSmallCircleSize;
        arc_left_value = sae_[pol](ex+kSmallCircle_[arc_left_idx][0], ey+kSmallCircle_[arc_left_idx][1]);
        if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
          arc_left_min_t = arc_left_value;
        }
      }
    }

    if (// Corners with newest segment of a minority of elements in the circle
        // These corners are equivalent to those in Mueggler et al. BMVC17
            (newest_segment_size <= kSmallMaxThresh) ||
        // Corners with newest segment of a majority of elements in the circle
        // This can be commented out to decrease noise at expenses of less repeatibility. If you do, DO NOT forget to comment the equilvent line in the large circle
        ((newest_segment_size >= (kSmallCircleSize - kSmallMaxThresh)) && (newest_segment_size <= (kSmallCircleSize - kSmallMinThresh)))) {
      is_arc_valid = true;
    }

    // Large Circle exploration
    if (is_arc_valid) {
    is_arc_valid = false;

      segment_new_min_t = sae_[pol](ex+kLargeCircle_[0][0], ey+kLargeCircle_[0][1]);
      arc_right_idx = 0;

      // Initialize in the newest element
      for (int i=1; i<kLargeCircleSize; i++) {
        const double t =sae_[pol](ex+kLargeCircle_[i][0], ey+kLargeCircle_[i][1]);
        if (t > segment_new_min_t) {
          segment_new_min_t = t;
          arc_right_idx = i; // % End up in the maximum value
        }
      }
      // Shift to the sides of the newest elements;
      arc_left_idx = (arc_right_idx-1+kLargeCircleSize)%kLargeCircleSize;
      arc_right_idx= (arc_right_idx+1)%kLargeCircleSize;
      arc_left_value = sae_[pol](ex+kLargeCircle_[arc_left_idx][0],
                                 ey+kLargeCircle_[arc_left_idx][1]);
      arc_right_value = sae_[pol](ex+kLargeCircle_[arc_right_idx][0],
                                  ey+kLargeCircle_[arc_right_idx][1]);
      arc_left_min_t = arc_left_value;
      arc_right_min_t = arc_right_value;

      // Expand
      // Initial expand does not require checking
      iteration = 1;
      for (; iteration<kLargeMinThresh; iteration++) {
        // Decide the most promising arc
        if (arc_right_value > arc_left_value) { // Right arc
          if (arc_right_min_t < segment_new_min_t) {
              segment_new_min_t = arc_right_min_t;
          }
          // Expand arc
          arc_right_idx= (arc_right_idx+1)%kLargeCircleSize;
          arc_right_value = sae_[pol](ex+kLargeCircle_[arc_right_idx][0],
                                      ey+kLargeCircle_[arc_right_idx][1]);
          if (arc_right_value < arc_right_min_t) { // Update minimum of the arc
            arc_right_min_t = arc_right_value;
          }
        } else { // Left arc
          // Include arc in new segment
          if (arc_left_min_t < segment_new_min_t) {
            segment_new_min_t = arc_left_min_t;
          }

          // Expand arc
          arc_left_idx= (arc_left_idx-1+kLargeCircleSize)%kLargeCircleSize;
          arc_left_value = sae_[pol](ex+kLargeCircle_[arc_left_idx][0],
                                     ey+kLargeCircle_[arc_left_idx][1]);
          if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
            arc_left_min_t = arc_left_value;
          }
        }
      }
      newest_segment_size = kLargeMinThresh;

      // Further expand until completion of the circle
      for (; iteration<kLargeCircleSize; iteration++) {
        // Decide the most promising arc
        if (arc_right_value > arc_left_value) { // Right arc
          // Include arc in new segment
          if ((arc_right_value >=  segment_new_min_t)) {
            newest_segment_size = iteration+1;
            if (arc_right_min_t < segment_new_min_t) {
              segment_new_min_t = arc_right_min_t;
            }
          }

          // Expand arc
          arc_right_idx= (arc_right_idx+1)%kLargeCircleSize;
          arc_right_value = sae_[pol](ex+kLargeCircle_[arc_right_idx][0],
                                      ey+kLargeCircle_[arc_right_idx][1]);
          if (arc_right_value < arc_right_min_t) { // Update minimum of the arc
            arc_right_min_t = arc_right_value;
          }
        } else { // Left arc
          // Include arc in new segment
          if ((arc_left_value >=  segment_new_min_t)) {
            newest_segment_size = iteration+1;
            if (arc_left_min_t < segment_new_min_t) {
              segment_new_min_t = arc_left_min_t;
            }
          }

          // Expand arc
          arc_left_idx= (arc_left_idx-1+kLargeCircleSize)%kLargeCircleSize;
          arc_left_value = sae_[pol](ex+kLargeCircle_[arc_left_idx][0],
                                    ey+kLargeCircle_[arc_left_idx][1]);
          if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
            arc_left_min_t = arc_left_value;
          }
        }
      }

      if (// Corners with newest segment of a minority of elements in the circle
          // These corners are equivalent to those in Mueggler et al. BMVC17
              (newest_segment_size <= kLargeMaxThresh) ||
          // Corners with newest segment of a majority of elements in the circle
          // This can be commented out to decrease noise at expenses of less repeatibility. If you do, DO NOT forget to comment the equilvent line in the small circle
          (newest_segment_size >= (kLargeCircleSize - kLargeMaxThresh) && (newest_segment_size <= (kLargeCircleSize - kLargeMinThresh))) ) {
        is_arc_valid = true;
      }
    }

    return is_arc_valid;
}


void ArcStarDetector::motion_compensation_function(double et, int ex, int ey, bool ep, const Motion_correction_value measurements){

  // 加入运动补偿的操作
  const double const_ex=ex;
  const double const_ey=ey;

  Eigen::Vector4d State=measurements.second.first.first;
  Eigen::Vector3f temp_v_a=measurements.second.first.second;//状态的加速度
  Eigen::Vector3f tmp_v;//状态的速度
  tmp_v[0]=State[0];
  tmp_v[1]=State[1];
  tmp_v[2]=State[2];
  double time_temp_v=State[3];

  Eigen::Vector3f accel_avg_=measurements.second.second.second.first;//当前IMU的加速度
  Eigen::Vector3f omega_avg_=measurements.second.second.second.second;//当前imu的角速度

  double t_0_event=measurements.second.second.first[0];//第一个事件的时间
  double t_0_eventstream=measurements.second.second.first[1];//当前事件流的时间
  double t_0=t_0_eventstream;//采用当前的事件信息流来进行运动补偿
  const double dt=et-t_0;

  // if(dt<0){
  //     std::cout<<"（measurements）比第一个事件要早？？？:"<<dt<<std::endl;
  // }
  // if(dt>0.03){
  //     std::cout<<"（measurements）比第一个事件要晚大于0.03:"<<dt<<std::endl;
  // }

  // if(dt>0.01){//大于一定值才做补偿
  //   Eigen::Vector2d correct_coordinate= motioncorrection(const_ex,const_ey,tmp_v,accel_avg_,omega_avg_,dt);
  //   ex=correct_coordinate[0];
  //   ey=correct_coordinate[1];
  // }

  // Eigen::Vector4d event;
  // event[0]=et;
  // event[1]=ex;
  // event[2]=ey;
  // // event[3]=ep;
  // event[3]=ep ? 1.0 : 0.0;

  // return event;

  dvs_msgs::msg::Event correct_e;
  correct_e.polarity=ep;
  correct_e.x=ex;
  correct_e.y=ey;
  correct_e.ts=et;//已经是秒为单位的浮点时间戳

  // 需要加互斥锁？？
  std::unique_lock<std::mutex> lock(events_mutex_);
  motion_correct_eventstream.events.push_back(correct_e);

}


} // acd : Asynchronous Corner Detector
