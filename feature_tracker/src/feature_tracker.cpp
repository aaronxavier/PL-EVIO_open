#include "feature_tracker.h"
#include "arc_star/arc_star_detector.h"
#include <thread>
#include <mutex>

evio::ArcStarDetector detector = evio::ArcStarDetector();//进行声明检测器

int FeatureTracker::n_id = 0;//点特征计数
int FeatureTracker::allfeature_cnt = 0;//线特征的计数（ // 用来统计整个地图中有了多少条线，它将用来赋值）
std::mutex mutex_threads;//创建互斥锁

// int save_image_num=0;
// int save_image_num1=0;
// void save_image_function(const cv::Mat &image){//存放当前帧已有的特征点

//     ostringstream path;
//     path <<  "/home/kwanwaipang/evio_result_visualization/feature_detection/"
//             <<"num:"<< save_image_num << "---"
//             << "feature_detection_old.jpg";
//     cv::imwrite( path.str().c_str(), image);
//     save_image_num++;
// }

// void save_image_function_2(const cv::Mat &image){//存放新检测的特征点

//     ostringstream path;
//     path <<  "/home/kwanwaipang/evio_result_visualization/feature_detection/"
//             <<"num:"<< save_image_num << "---"
//             << "feature_detection_new.jpg";
//     cv::imwrite( path.str().c_str(), image);
//     save_image_num++;
// }

// void save_image_function_3(const cv::Mat &image){

//     ostringstream path;
//     path <<  "/home/kwanwaipang/evio_result_visualization/event_mat/"
//             <<"num:"<< save_image_num << "---"
//             << "event_mat.jpg";
//     cv::imwrite( path.str().c_str(), image);
//     save_image_num1++;
// }

// // 定义几个buf
// queue<cv::Mat> timesurface_buf;//存放time surface
// queue<cv::Mat> event_mat_buf;//存放event mat
// // queue<cv::Mat> mask_buf;
// queue<vector<cv::Point2f>> cur_point_buf;
// queue<cv::Point2f> new_detect_point_buf;
// std::mutex m_buf_timesuface;//定义互斥锁

// void FeatureTracker::save_feature_process(){//保存feature 处理过程
//     while(1){
//         cv::Mat timesurface_save;
//         cv::Mat event_mat_save;

//         m_buf_timesuface.lock();
//         if((!timesurface_buf.empty()) && (!cur_point_buf.empty()) && (!event_mat_buf.empty())){//如果这两个不是空的话
            
//             cv::Mat timesurface_save_666=timesurface_buf.front();//拿出来
//             timesurface_buf.pop();//拿完必须马上删除
//             vector<cv::Point2f> ___cur_point=cur_point_buf.front();
//             cur_point_buf.pop();
//             event_mat_save=event_mat_buf.front().clone();//本身就是彩色的
//             event_mat_buf.pop();//删除
//             m_buf_timesuface.unlock();

//             vector<cv::Mat> channels;
//             for (int i=0;i<3;i++)
//             {
//                 channels.push_back(timesurface_save_666);
//             }
//             merge(channels,timesurface_save);//单通道转换为3通道

//             // save_image_function(timesurface_save);

//             for(auto &p:___cur_point){
//                 cv::circle(timesurface_save, p, MIN_DIST, cv::Scalar(128.0,128.0,128.0), -1);//特征点的区域变为128.0
//                 cv::circle(timesurface_save, p, MIN_DIST, cv::Scalar(255.0,255.0,255.0), 2);//特征点的区域外加白圈
//                 cv::circle(timesurface_save, p, 6, cv::Scalar(0, 0, 255),-1);//画为实心的红点

//                 cv::circle(event_mat_save, p, MIN_DIST, cv::Scalar(255.0,255.0,255.0), 2);//特征点的区域外加白圈
//                 cv::circle(event_mat_save, p, 6, cv::Scalar(0, 0, 255),-1);//画为实心的红点
//             }
//             save_image_function(timesurface_save);//保存上一帧tracking到的特征点
//             save_image_function_3(event_mat_save);//把之前tracking到的点画出来

//         }
//         else
//             m_buf_timesuface.unlock();

//         m_buf_timesuface.lock();
//         if(!new_detect_point_buf.empty() && !timesurface_save.empty() && !event_mat_save.empty()){
//             cv::Point2f new_point=new_detect_point_buf.front();
//             new_detect_point_buf.pop();
//             m_buf_timesuface.unlock();

//             cv::circle(timesurface_save, new_point, MIN_DIST, cv::Scalar(128.0,128.0,128.0), -1);//特征点的区域变为128.0
//             cv::circle(timesurface_save, new_point, MIN_DIST, cv::Scalar(255.0,255.0,255.0), 2);//特征点的区域外加白圈
//             cv::circle(timesurface_save, new_point, 6, cv::Scalar(255, 0, 0),-1);//画为实心的蓝点

//             save_image_function_2(timesurface_save);//把新检测的特征点一个一个画出来

//             //把特征点画到event mat上
//             cv::circle(event_mat_save, new_point, MIN_DIST, cv::Scalar(255.0,255.0,255.0), 2);//特征点的区域外加白圈
//             cv::circle(event_mat_save, new_point, 6, cv::Scalar(0, 0, 255),-1);//画为实心的红点
//             save_image_function_3(event_mat_save);

//         }
//         else
//             m_buf_timesuface.unlock();

//     }
//     std::chrono::milliseconds dura(2);//等待2ms
//     std::this_thread::sleep_for(dura);
// }


void good_arc_FeaturesToTrack(const cv::Mat time_surface_map, const dvs_msgs::msg::EventArray &last_event,  vector<cv::Point2f> &n_pts, const vector<cv::Point2f> &_cur_pts, const int maxCorners, const int _MIN_DIST, const cv::Mat event_mask)
{

    int ncorners = 0;//计算一下一共检测了多少
    n_pts.clear();//要清空一下，从而保证放入新的
    cv:: Mat event_mask_666=event_mask.clone();//值为255.0的点不检测

    if(maxCorners>0){//当需要检测的时候
    for (const dvs_msgs::msg::Event& e : last_event.events) { // 获取last_event中的每一个事件
        if (ncorners>=maxCorners) {//是否大于需要的点，是则退出
                break;
            }
            else{//没有大于需要的点的数目
                if(event_mask_666.at<double>(e.y,e.x)!=255.0 ){//判断此处的event_mask是不是不为255.0
                     if(time_surface_map.at<uchar>(e.y,e.x) !=TS_LK_THRESHOLD){//看看对应位置的time surface是否不为128
                         //不在之前的特征点范围内，才开始arc角点检测
                          if (detector.isCorner(e.ts, e.x, e.y, e.polarity)){//判断是否角点（SAE已经在前面更新了）
                                //若是角点，则放入n_pts中，同时计数
                                n_pts.push_back(cv::Point2f((float)e.x, (float)e.y));
                                ncorners++;//计算corners数目，若大于一定值，则退出
                                //绘制e.x与e.y处的实心圆并赋值255，那么下次这个范围就不会选了 
                                cv::circle(event_mask_666, cv::Point(e.x,e.y), _MIN_DIST, 255.0, -1);
                                 //画图
                                //  m_buf_timesuface.lock();
                                //  new_detect_point_buf.push(cv::Point2f(e.x,e.y));//放入buf中
                                //  m_buf_timesuface.unlock();
                          }
                     }
                    //  else{std::cout<<"the value of event_mask_666 is 255.0="<<event_mask_666.at<double>(e.y,e.x)<<std::endl; }                    
                }
                // else{ std::cout<<"the time surface of this point is"<<TS_LK_THRESHOLD<<std::endl; }   
            }

        }            
    }              
}

// // //画图用的
// void good_arc_FeaturesToTrack(const cv::Mat time_surface_map, const dvs_msgs::EventArray &last_event,  vector<cv::Point2f> &n_pts, const vector<cv::Point2f> &_cur_pts, const int maxCorners, const int _MIN_DIST, const cv::Mat event_mask, const cv::Mat _event_mat)
// {

//     int ncorners = 0;//计算一下一共检测了多少
//     n_pts.clear();//要清空一下，从而保证放入新的
//     cv:: Mat event_mask_666=event_mask.clone();//值为255.0的点不检测

//     m_buf_timesuface.lock();
//     timesurface_buf.push(time_surface_map);
//     cur_point_buf.push(_cur_pts);
//     event_mat_buf.push(_event_mat);
//     m_buf_timesuface.unlock();

//     if(maxCorners>0){//当需要检测的时候
//     for (const dvs_msgs::Event& e : last_event.events) { // 获取last_event中的每一个事件
//         if (ncorners>=maxCorners) {//是否大于需要的点，是则退出
//                 break;
//             } 
//             else{//没有大于需要的点的数目
//                 if(event_mask_666.at<double>(e.y,e.x)!=255.0 ){//判断此处的event_mask是不是不为255.0
//                      if(time_surface_map.at<uchar>(e.y,e.x) !=TS_LK_THRESHOLD){//看看对应位置的time surface是否不为128
//                          //不在之前的特征点范围内，才开始arc角点检测
//                           if (detector.isCorner(e.ts.toSec(), e.x, e.y, e.polarity)){//判断是否角点（SAE已经在前面更新了）
//                                 //若是角点，则放入n_pts中，同时计数
//                                 n_pts.push_back(cv::Point2f((float)e.x, (float)e.y));
//                                 ncorners++;//计算corners数目，若大于一定值，则退出
//                                 //绘制e.x与e.y处的实心圆并赋值255，那么下次这个范围就不会选了 
//                                 cv::circle(event_mask_666, cv::Point(e.x,e.y), _MIN_DIST, 255.0, -1);
//                                  //画图
//                                  m_buf_timesuface.lock();
//                                  new_detect_point_buf.push(cv::Point2f(e.x,e.y));//新检测的点放入buf中
//                                  m_buf_timesuface.unlock();
//                           }
//                      }
//                     //  else{std::cout<<"the value of event_mask_666 is 255.0="<<event_mask_666.at<double>(e.y,e.x)<<std::endl; }                    
//                 }
//                 // else{ std::cout<<"the time surface of this point is"<<TS_LK_THRESHOLD<<std::endl; }   
//             }

//         }            
//     }              
// }



bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

//根据状态位进行处理
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(std::vector<cv::DMatch> &v, vector<uchar> status)//处理good_matches
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


FeatureTracker::FeatureTracker()
{
    // allfeature_cnt = 0;//线特征的计数（ // 用来统计整个地图中有了多少条线，它将用来赋值）
}

// 给现有的特征点设置mask，目的为了特征点的均匀化
void FeatureTracker::setMask()
{
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));
     // 利用光流特点，追踪多的稳定性好，排前面
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
             // 把挑选剩下的特征点重新放进容器
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST_IMG, 0, -1);// opencv函数，把周围一个圆内全部置0,这个区域不允许别的特征点存在，避免特征点过于集中
        }
    }
}

void FeatureTracker::stereo_setMask()
{
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));


    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < cur_pts.size(); i++){
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));
    }

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    cur_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            cur_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

//在good_arc_FeaturesToTrack中，会传入mask_arc，在mask_arc为255.0的地方不检测特征点。
void FeatureTracker::setevent_Mask()
{
    mask_arc = cv::Mat(ROW, COL, CV_64FC1, cv::Scalar(0.0));//mask初始化为0.0，有feature point的地方设置为255.0

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;//跟踪的次数+跟踪的次数与id（相当于一个容器先放着）

    for (unsigned int i = 0; i < cur_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));//将当前的特征点都放入cnt_pts_id中
    // 利用光流特点，追踪多的稳定性好，排前面
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {//排序（按被跟踪的次数排序）
            return a.first > b.first;
         });

    cur_pts.clear();//清空，再放回去
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask_arc.at<double>(it.second.first) == 0.0)//如果mask在某个特征点上是0的话
        {
            //  把挑选剩下的特征点重新放进容器（而且还经过了排序！）
            cur_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            // // opencv函数，把周围一个圆内全部置0,这个区域不允许别的特征点存在，避免特征点过于集中 cv::circle(mask, it.second.first, MIN_DIST, 0, -1)
            cv::circle(mask_arc, it.second.first, MIN_DIST, 255.0, -1);//绘制给定中心和半径的空心或者实心圆
            //center - Center of the circle cv::Point (x, y)
            //void cv::circle (InputOutputArray img, Point center, int radius, const Scalar &color, int thickness=1, int lineType=LINE_8, int shift=0)
            //MIN_DIST为半径
            //thickness为负值 (如 FILLED) 表示要绘制实心圆
            //这些地方都置0了
        }
    }
}

void FeatureTracker::addPoints()// 把新的点加入容器，id给-1作为区分
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);//加到forw_pts中
        ids.push_back(-1);//id的值给-1
        track_cnt.push_back(1);
    }
}


/**
 * @brief 
 * 
 * @param[in] _img 输入图像
 * @param[in] _cur_time 图像的时间戳
 * 1、图像均衡化预处理
 * 2、光流追踪
 * 3、提取新的特征点（如果发布）
 * 4、所有特征点去畸变，计算速度
 */
void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)//真正进行跟踪处理
{
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;

    if (EQUALIZE)//均衡处理
    {  // 图像太暗或者太亮，提特征点比较难，所以均衡化一下（采用对比度受限的直方图均衡）
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;

    if(!( img.rows==ROW &&  img.cols==COL)){//对付image有时大小不一致的问题
         resize(img,img, cv::Size(COL, ROW));
    }

 // 这里forw表示当前，cur表示上一帧
    if (forw_img.empty()) // 第一次输入图像，prev_img这个没用
    {
        prev_img = cur_img = forw_img = img;//全部初始化
    }
    else//不是第一帧时，把当前帧给forw_img
    {
        forw_img = img;//把当前帧给forw_img
    }

    forw_pts.clear();//光流检测的特征点

    if (cur_pts.size() > 0)//当前已有的特征点，也就是上一帧来的特征点
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
         // Step 1 通过opencv光流追踪给的状态位剔除outlier
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);//光流计算      

        // reverse check
        // image不需要加逆向光流
        // if(FLOW_BACK){
        //     vector<uchar> reverse_status;
        //     vector<cv::Point2f> reverse_pts = cur_pts;//之前的特征点
        //     cv::calcOpticalFlowPyrLK(forw_img, cur_img, forw_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
        //     cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);//当前帧映射回上一帧
        //     for(size_t i = 0; i < status.size(); i++)
        //     {
        //         if(status[i] && reverse_status[i] && distance(cur_pts[i], reverse_pts[i]) <= 0.5)
        //         {
        //             status[i] = 1;
        //         }
        //         else
        //             status[i] = 0;
        //     }
        // }

        for (int i = 0; i < int(forw_pts.size()); i++)// Step 2 通过图像边界剔除outlier
            if (status[i] && !inBorder(forw_pts[i]))// 追踪状态好检查在不在图像范围
                status[i] = 0;
        //根据状态向量status去除没有被跟踪到的特征点
        reduceVector(prev_pts, status);// 没用到
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);// 特征点的id
        reduceVector(cur_un_pts, status);// 去畸变后的坐标
        reduceVector(track_cnt, status);// 追踪次数
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }

    for (auto &n : track_cnt) // 被追踪到的是上一帧就存在的，因此追踪数+1
        n++;

    if (PUB_THIS_FRAME)//初始化为false
    {
        rejectWithF(); // Step 3 通过对级约束来剔除outlier（fusion没有用），去除误匹配的点  （利用基本矩阵剔除外点）
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        //将已经有特征点的区域灰度置0，该区域就不再检测特征点了，提供给特征检测的函数用goodFeaturesToTrack
        setMask();//很重要，让特征点分布稀疏一些
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());//额外需要检测的点(最大的特征-当前跟踪到的特征)
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;
            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST_IMG, mask);//检测新的特征点
        }
        else
            n_pts.clear();//清空
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        //将新提取的特征添加到向量中去
        addPoints();//把检测到的点，加入当前点
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts; // 以上三个量无用
    cur_img = forw_img; // 实际上是上一帧的图像
    cur_pts = forw_pts;// 上一帧的特征点
    //根据相机模型，去除视觉特征畸变，并计算特征点运动的速度
    undistortedPoints();
    prev_time = cur_time;
}

//进行双目的feature tracking
void FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
{
    TicToc t_r;
    cur_time = _cur_time;
    int row = cur_img.rows;
    int col = cur_img.cols;
    cv::Mat rightImg = _img1;

    if(cur_img.empty()){
        prev_img = cur_img = _img; 
    }else{
        cur_img = _img; 
    }
    cur_pts.clear();

    if (prev_pts.size() > 0)//当上一帧有特征点的时候
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);

        // reverse check，进行逆向光流。对于双目可以保留不？
        if(FLOW_BACK)
        {
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts;
            cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            //cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3); 
            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }
        
        for (int i = 0; i < int(cur_pts.size()); i++)
            if (status[i] && !inBorder(cur_pts[i]))
                status[i] = 0;
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(ids, status );
        reduceVector(prev_un_pts, status);
        reduceVector(track_cnt, status);
    }

    for (auto &n : track_cnt)// 被追踪到的是上一帧就存在的，因此追踪数+1
        n++;

    if (PUB_THIS_FRAME)
    {
        setreo_rejectWithF();

        stereo_setMask();

        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());//额外需要检测的
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);

        }
        else
            n_pts.clear();

        for (auto &p : n_pts)
        {
            cur_pts.push_back(p);
            ids.push_back(n_id++);//进行ID的更新，此处跟mono版本不一样
            track_cnt.push_back(1);
        }
    }

    cur_un_pts = undistortedPts(cur_pts, setro_m_camera[0]);
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

    if(!_img1.empty())//当右边的相机也不为空
    {
        ids_right.clear();//右相机特征点的id
        cur_right_pts.clear();//当前右相机的特征点
        cur_un_right_pts.clear();
        right_pts_velocity.clear();
        cur_un_right_pts_map.clear();//右相机。id+特征点
        if(!cur_pts.empty())//当前左目的特征点不为0
        {
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;
            // cur left ---- cur right
            cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(21, 21), 3);

            // reverse check cur right ---- cur left
            if(FLOW_BACK)
            {
                cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
                    {
                        status[i] = 1;
                    }
                    else
                        status[i] = 0;
                }
            }

            //fundamental matrix check 
            
                vector<cv::Point2f> un_l_pts, un_r_pts; 
                un_l_pts.reserve(ids.size()); 
                un_r_pts.reserve(ids.size()); 

                for(int i=0; i<ids.size(); i++){
                    if(status[i]){
                        Eigen::Vector3d tmp_p;
                        setro_m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
                        // ouf2 << i<<" l: "<< tmp_p.transpose()<<" "; 
                        // tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                        // tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                        un_l_pts.push_back(cv::Point2f(tmp_p.x(), tmp_p.y())); 
                        // ouf2<<" proj: "<<tmp_p.x()<<" "<<tmp_p.y()<<" "<<endl; 

                        setro_m_camera[1]->liftProjective(Eigen::Vector2d(cur_right_pts[i].x, cur_right_pts[i].y), tmp_p);
                        // ouf2 << i<<" r: "<< tmp_p.transpose()<<" "; 
                        // tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                        // tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                        un_r_pts.push_back(cv::Point2f(tmp_p.x(), tmp_p.y()));
                        // ouf2<<" proj: "<<tmp_p.x()<<" "<<tmp_p.y()<<" "<<endl; 
                        // ouf<<i<<" "<<cur_pts[i].x<<" "<<cur_pts[i].y<<" "<<cur_right_pts[i].x<<" "<<cur_right_pts[i].y<<endl;
                    }
                }

            // use Tlr to to verification 

            ids_right = ids;//右边点的id跟左边的一样
            reduceVector(cur_right_pts, status);
            reduceVector(ids_right, status);

            assert(cur_right_pts.size() == un_l_pts.size()); 
            vector<uchar> lr_fund_status(cur_right_pts.size(), 0); 

            int cnt_right_inlier = 0; 
            for(int i=0; i<lr_fund_status.size(); i++){
                
                // check epipilar distance 
                Vector3d p3d0(un_l_pts[i].x, un_l_pts[i].y, 1.);
                Vector3d p3d1(un_r_pts[i].x, un_r_pts[i].y, 1.);  
                const double epipolar_error =
                std::abs(p3d1.transpose() * Eeesntial_matrix * p3d0);
                if(epipolar_error < 0.005) // epipolar distance 
                {
                    lr_fund_status[i] = 1; 
                    ++cnt_right_inlier;
                }
            }

            reduceVector(cur_right_pts, lr_fund_status);
            reduceVector(ids_right, lr_fund_status);

            cur_un_right_pts = undistortedPts(cur_right_pts, setro_m_camera[1]);

            right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
            ROS_DEBUG("feature_tracker.cpp: found %d inlier stereo matches!", cnt_right_inlier); 
        }
        prev_un_right_pts_map = cur_un_right_pts_map;
    }
    if(SHOW_TRACK)
        drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);

    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;

    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];
}

cv::Mat FeatureTracker::getTrackImage()
{
    return imTrack;
}

cv::Mat FeatureTracker::getLoopImage()
{
    return Image_loop;
}

cv::Mat FeatureTracker::getTrackImage_two()
{
    return imTrack_two;
}

cv::Mat FeatureTracker::gettimesurface()
{
    return time_surface_visualization;
}

cv::Mat FeatureTracker::getTrackImage_two_line()
{
    return imTrack_two_line;
}

void multi_thread_create_SAE(std::vector<dvs_msgs::msg::Event> e, int beginIndex, int length){
    for(int i=beginIndex;i<beginIndex+length;i++){
        detector.createSAE(e[i].ts,e[i].x,e[i].y,e[i].polarity);
    }
}

void multi_thread_create_SAE_motion(std::vector<dvs_msgs::msg::Event> e, int beginIndex, int length, const Motion_correction_value measurements){
    for(int i=beginIndex;i<beginIndex+length;i++){

        // double t_0=measurements.second.second.first[0];
        // double dt_0=e[i].ts-t_0;
        // if(dt_0<0){
        //     std::cout<<"比第一个事件要早？？？:"<<dt_0<<std::endl;
        // }
        // if(dt_0>0.03){
        //     std::cout<<"比第一个事件要晚大于0.03:"<<dt_0<<std::endl;
        // }

        detector.createSAE(e[i].ts,e[i].x,e[i].y,e[i].polarity,measurements);
    }
}


void process_linefeature(FeatureTracker *this_object, const cv::Mat img_line, bool first_img, const cv::Mat event_mat)
{
    mutex_threads.lock();

    //push 当前信息
    this_object->forwframe_.reset(new FrameLines);  // 初始化一个新的帧
    this_object->forwframe_->img = img_line;
    this_object->forwframe_->event_img=event_mat;

    // 先对time surface进行中值滤波处理
    cv::Mat line_process_blur=img_line.clone();//用time surface来提取特征与匹配
    cv::medianBlur(line_process_blur, line_process_blur, 3);//中值滤波


    cv::Ptr<cv::line_descriptor::LSDDetectorC> lsd_ = cv::line_descriptor::LSDDetectorC::createLSDDetectorC();//线特征检测器

    //设置线特征的参数
    cv::line_descriptor::LSDDetectorC::LSDOptions opts;//检测线特征的一些参数
    opts.refine       = 1;     //1     	The way found lines will be refined
    // opts.scale        = 0.1; //0.5;   //0.8   	The scale of the image that will be used to find the lines. Range (0..1].
    opts.scale          =Scale_image;
    opts.sigma_scale  = 0.6;	//0.6  	Sigma for Gaussian filter. It is computed as sigma = _sigma_scale/_scale.
    opts.quant        = 2.0;	//2.0   Bound to the quantization error on the gradient norm
    opts.ang_th       = 22.5;	//22.5	Gradient angle tolerance in degrees
    opts.log_eps      = 0.0; //1.0;	//0		Detection threshold: -log10(NFA) > log_eps. Used only when advance refinement is chosen
    opts.density_th   = 0.7; //0.6;	//0.7	Minimal density of aligned region points in the enclosing rectangle.
    opts.n_bins       = 1024;	//1024 	Number of bins in pseudo-ordering of gradient modulus.
    double min_line_length = 0.125; //0.025; //0.125;  // Line segments shorter than that are rejected
    opts.min_length   = min_line_length*(std::min(img_line.cols,img_line.rows));//最小的线段长度
    // opts.min_length=MIN_length;

    std::vector<cv::line_descriptor::KeyLine> lsd, keylsd;//产生的线
    lsd_->detect(line_process_blur, lsd, 1.2, 1, opts);//检测线段(从time surface上提取)
    // （另外两个参数是）
    // detect_scale	计算当前图像的金字塔下采样倍率，默认1.2，即下采样到原来1/1.2
    // detect_numOctaves	计算当前图像需要提取几层高斯金字塔，默认值1

    // lsd_->detect(event_mat, lsd, 2, 1, opts);//检测线段(从事件帧中提取)
    // （线特征的检测与提取。此处需要更换为提取event的线特征，关键为获得什么样的std::vector<cv::line_descriptor::KeyLine> lsd）
    // 匹配跟特征提取应该保持一致性，才合理



    cv::Mat lbd_descr, keylbd_descr;//产生描述子进行匹配
    //线特征的描述子仍然采用lsd的
    cv::Ptr<cv::line_descriptor::BinaryDescriptor> bd_ = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();
    bd_->compute(line_process_blur, lsd, lbd_descr );//由线特征，img_line产生线描述子
    // ROS_ERROR("number of keyline %d",lsd.size());


    for ( int i = 0; i < (int) lsd.size(); i++ )//从lsd中选一些出来成为keylsd
    {
        // if( lsd[i].octave == 0 && lsd[i].lineLength >= 60)//如果在第0层。且线的长度大于60为关键线
        // if( lsd[i].octave == 0 && lsd[i].lineLength >= 10)//如果在第0层。且线的长度大于60为关键线
        if( lsd[i].octave == 0 && lsd[i].lineLength >= MIN_length)//如果在第0层。且线的长度大于60为关键线
        {
            keylsd.push_back(lsd[i]);//就会放入关键线中
            keylbd_descr.push_back(lbd_descr.row(i));//对应关键线的描述子
        }
    }

    this_object->forwframe_->keylsd = keylsd;//当前帧的线特征
    this_object->forwframe_->lbd_descr = keylbd_descr;//当前帧线特征的描述子

    //ID更新
    for (size_t i = 0; i < this_object->forwframe_->keylsd.size(); ++i) {
        if(first_img)//如果是第一帧的话，把id加入
            this_object->forwframe_->lineID.push_back(this_object->allfeature_cnt++);
        else//若不是第一帧的话，先赋予-1（后面会进行更新处理）
            this_object->forwframe_->lineID.push_back(-1);   // give a negative id  新检测的id全部置为-1
    }
    
    //当前已有的关键线的size大于0的时候，就开始进行匹配处理
    if(this_object->curframe_->keylsd.size() > 0)
    {
        std::vector<cv::DMatch> lsd_matches;//匹配器(存放匹配的结果)
        cv::Ptr<cv::line_descriptor::BinaryDescriptorMatcher> bdm_;//线特征的匹配子
        bdm_ = cv::line_descriptor::BinaryDescriptorMatcher::createBinaryDescriptorMatcher();//产生描述子匹配器
        bdm_->match(this_object->forwframe_->lbd_descr, this_object->curframe_->lbd_descr, lsd_matches);//最新的，跟当前已有的进行匹配
        //注意，此时lsd_matches中的queryIdx指的是forwframe_,trainIdx指的是curframe_,

        std::vector<cv::DMatch> good_matches;//保存匹配效果比较好的
        // std::vector<cv::line_descriptor::KeyLine> good_Keylines;//匹配效果比较好的线
        good_matches.clear();
        for (int i=0; i<lsd_matches.size();i++){
            if(lsd_matches[i].distance<30){//当匹配的距离少于30认为是比较好的匹配
            // if(lsd_matches[i].distance<60){//把约束要求降低
                cv::DMatch mt=lsd_matches[i];
                cv::line_descriptor::KeyLine line1=this_object->forwframe_->keylsd[mt.queryIdx];//queryIdx指的是forwframe_
                cv::line_descriptor::KeyLine line2=this_object->curframe_->keylsd[mt.trainIdx];//trainIdx指的是curframe_,
                cv::Point2f serr = line1.getStartPoint() - line2.getEndPoint();//起始点的误差
                cv::Point2f eerr = line1.getEndPoint() - line2.getEndPoint();//终止点的误差
                if((serr.dot(serr) < 200 * 200) && (eerr.dot(eerr) < 200 * 200)&&abs(line1.angle-line2.angle)<0.1){ // 线段在图像里不会跑得特别远
                // if((serr.dot(serr) < 400 * 400) && (eerr.dot(eerr) < 400 * 400)&&abs(line1.angle-line2.angle)<1){ // 把约束要求降低
                    good_matches.push_back( lsd_matches[i] );//将匹配结果较好的存放
                } 
            }
        }
        this_object->rejectWithF_line(this_object->curframe_->keylsd, this_object->forwframe_->keylsd, good_matches);//实际上是对good_matches进行处理

        // if(good_matches.size()!=0)
            // ROS_ERROR("number of good_matches %d",good_matches.size());

        // vector< int > success_id;//获取成功匹配的id，匹配成功的才会进行id赋值
        for (int k = 0; k < good_matches.size(); ++k) {
            cv::DMatch mt = good_matches[k];
            this_object->forwframe_->lineID[mt.queryIdx] = this_object->curframe_->lineID[mt.trainIdx];//匹配上的id进行赋值,没匹配上的，仍然是-1
            // success_id.push_back(this_object->curframe_->lineID[mt.trainIdx]);//获取成功匹配的id
        }


        if(SHOW_TRACK)//读入参数是否show跟踪，如果是的话，就运行下面函数，将线特征显示出来！ （并且要不是第一帧）
        {
            // FeatureTracker::event_drawTrack_two_line(forwframe_->img.clone(), curframe_->img.clone(), forwframe_->keylsd, curframe_->keylsd, good_matches);//前后帧线特征匹配的结果输出
            // this_object->event_drawTrack_two_line(this_object->forwframe_->img.clone(), this_object->curframe_->img.clone(), this_object->forwframe_->keylsd, this_object->curframe_->keylsd, good_matches);//只把匹配好的输出
            this_object->event_drawTrack_two_line(this_object->forwframe_->event_img.clone(), this_object->curframe_->event_img.clone(), this_object->forwframe_->keylsd, this_object->curframe_->keylsd, good_matches);//只把匹配好的输出(画在event mat上)
            // this_object->event_drawTrack_two_line_all(this_object->forwframe_->img.clone(), this_object->curframe_->img.clone(), this_object->forwframe_->keylsd, keylsd);//全部检测的线画出来
        }


        //将所有的都保存下来
        vector<cv::line_descriptor::KeyLine> vecLine_tracked;//跟踪上的线
        // vector<cv::line_descriptor::KeyLine> vecLine_new;//新产生的线
        vector< int > lineID_tracked;//跟踪上的线的id
        // vector< int > lineID_new;//新产生的线的id
        cv::Mat DEscr_tracked;//跟踪上的描述子
        // cv::Mat Descr_new;//新产生的描述子
        // 将跟踪的线和没跟踪上的线进行区分
        int num_new=0;
        int num_old=0;
        for (size_t i = 0; i < this_object->forwframe_->keylsd.size(); ++i)//遍历当前所有的线特征（关键的，经过选择后的）
        {
            if( this_object->forwframe_->lineID[i] == -1)//没匹配上的
            {
                this_object->forwframe_->lineID[i] = this_object->allfeature_cnt++;//对ID进行更新，这部分很重要！
                // vecLine_new.push_back(this_object->forwframe_->keylsd[i]);
                // lineID_new.push_back(this_object->forwframe_->lineID[i]);
                // Descr_new.push_back(this_object->forwframe_->lbd_descr.row( i ) );
                
                //将新产生的ID也保存下来
                vecLine_tracked.push_back(this_object->forwframe_->keylsd[i]);
                lineID_tracked.push_back(this_object->forwframe_->lineID[i]);
                DEscr_tracked.push_back(this_object->forwframe_->lbd_descr.row( i ) );
                num_new++;
            }else//匹配上的
            {
                vecLine_tracked.push_back(this_object->forwframe_->keylsd[i]);
                lineID_tracked.push_back(this_object->forwframe_->lineID[i]);
                DEscr_tracked.push_back(this_object->forwframe_->lbd_descr.row( i ) );
                num_old++;
            }
        }

        //将新产生的ID也保存下来
        // for(int i=0;i<vecLine_new.size();++i){
        //     vecLine_tracked.push_back(vecLine_new[i]);
        //     lineID_tracked.push_back(lineID_new[i]);
        //     DEscr_tracked.push_back(Descr_new.row(i));
        // }

        //保存下来的存放一下
        this_object->forwframe_->keylsd = vecLine_tracked;
        this_object->forwframe_->lineID = lineID_tracked;
        this_object->forwframe_->lbd_descr = DEscr_tracked;

        if(good_matches.size()!=num_old){
            ROS_ERROR("number of good_matches %d",good_matches.size());
            std::cout<<"first_img="<<first_img<<std::endl;
            ROS_ERROR("number of new %d",num_new);
            ROS_ERROR("number of old %d",num_old);
            ROS_ERROR("number of lines %d",vecLine_tracked.size());
            ROS_ERROR("******************************************");
        }

        // if(num_old==0){
        //    ROS_ERROR("num_old==0!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"); 
        // }

        // ***********************************************************************//

        // //把没追踪到的线存起来（当当前的线少于一定阈值的时候。就保留，不然就丢弃）
        // vector<cv::line_descriptor::KeyLine> vecLine_tracked;//跟踪上的线
        // vector<cv::line_descriptor::KeyLine> vecLine_new;//新产生的线
        // vector< int > lineID_tracked;//跟踪上的线的id
        // vector< int > lineID_new;//新产生的线的id
        // cv::Mat DEscr_tracked;//跟踪上的描述子
        // cv::Mat Descr_new;//新产生的描述子
        // // 将跟踪的线和没跟踪上的线进行区分
        // for (size_t i = 0; i < this_object->forwframe_->keylsd.size(); ++i)//遍历当前所有的线特征（关键的，经过选择后的）
        // {
        // if( this_object->forwframe_->lineID[i] == -1)//若id为-1，就是最新出现的
        // {
        //     this_object->forwframe_->lineID[i] = this_object->allfeature_cnt++;//对这个id进行赋值
        //     vecLine_new.push_back(this_object->forwframe_->keylsd[i]);//最新产生的线
        //     lineID_new.push_back(this_object->forwframe_->lineID[i]);//最新产生的线的id
        //     Descr_new.push_back( this_object->forwframe_->lbd_descr.row( i ) );//最新产生的线的描述子
        // }      
        // else//之前已经出现过的，就是一直有被跟踪的
        // {
        //     vecLine_tracked.push_back(this_object->forwframe_->keylsd[i]);
        //     lineID_tracked.push_back(this_object->forwframe_->lineID[i]);
        //     DEscr_tracked.push_back( this_object->forwframe_->lbd_descr.row( i ) );
        // }
        // }

        // //对于新产生的线分为垂直的与水平的
        // vector<cv::line_descriptor::KeyLine> h_Line_new;//新产生的水平的线
        // vector<cv::line_descriptor::KeyLine> v_Line_new;//新产生的垂直的线
        // vector< int > h_lineID_new,v_lineID_new;
        // cv::Mat h_Descr_new,v_Descr_new;
        // for (size_t i = 0; i < vecLine_new.size(); ++i)//对于新出现的线
        // {
        //     if((((vecLine_new[i].angle >= 3.14/4 && vecLine_new[i].angle <= 3*3.14/4))||(vecLine_new[i].angle <= -3.14/4 && vecLine_new[i].angle >= -3*3.14/4)))
        //     {//对于大于90度同时小于270度，或者大于-270而小于-90的线
        //         h_Line_new.push_back(vecLine_new[i]);
        //         h_lineID_new.push_back(lineID_new[i]);
        //         h_Descr_new.push_back(Descr_new.row( i ));
        //     }
        //     else
        //     {
        //         v_Line_new.push_back(vecLine_new[i]);
        //         v_lineID_new.push_back(lineID_new[i]);
        //         v_Descr_new.push_back(Descr_new.row( i ));
        //     }      
        // }

        // int h_line,v_line;
        // h_line = v_line =0;
        // for (size_t i = 0; i < vecLine_tracked.size(); ++i)
        // {
        //     if((((vecLine_tracked[i].angle >= 3.14/4 && vecLine_tracked[i].angle <= 3*3.14/4))||(vecLine_tracked[i].angle <= -3.14/4 && vecLine_tracked[i].angle >= -3*3.14/4)))
        //     {
        //         h_line ++;
        //     }
        //     else
        //     {
        //         v_line ++;
        //     }
        // }
        // //两个方向若都小于35条，就新增
        // // int diff_h = 35 - h_line;
        // // int diff_v = 35 - v_line;
        // int diff_h = Maximun_lines/2 - h_line;
        // int diff_v = Maximun_lines/2 - v_line;

        // if( diff_h > 0)    // 当少于35条的时候，补充线条
        // {
        //     int kkk = 1;
        //     if(diff_h > h_Line_new.size())//要增加diff_h条线
        //         diff_h = h_Line_new.size();
        //     else 
        //         kkk = int(h_Line_new.size()/diff_h);

        //     for (int k = 0; k < diff_h; ++k) 
        //     {
        //         vecLine_tracked.push_back(h_Line_new[k]);
        //         lineID_tracked.push_back(h_lineID_new[k]);
        //         DEscr_tracked.push_back(h_Descr_new.row(k));
        //     }
        // }

        // if( diff_v > 0)    // 补充线条
        // {
        //     int kkk = 1;
        //     if(diff_v > v_Line_new.size())//要增加diff_v条线
        //         diff_v = v_Line_new.size();
        //     else 
        //         kkk = int(v_Line_new.size()/diff_v);

        //     for (int k = 0; k < diff_v; ++k)  
        //     {
        //         vecLine_tracked.push_back(v_Line_new[k]);
        //         lineID_tracked.push_back(v_lineID_new[k]);
        //         DEscr_tracked.push_back(v_Descr_new.row(k));
        //     }        
        // }

        // //这是为了让当前最新帧，的线的数目保持一定的数量
        // this_object->forwframe_->keylsd = vecLine_tracked;
        // this_object->forwframe_->lineID = lineID_tracked;
        // this_object->forwframe_->lbd_descr = DEscr_tracked;
    }
    else{//当上一次没有检测新的特征线，会导致当前的特征线为0；那么对最新的检测出的特征线赋值id
        for(int i=0;i<this_object->forwframe_->keylsd.size();++i){//
            if(this_object->forwframe_->lineID[i] == -1)
                this_object->forwframe_->lineID[i] = this_object->allfeature_cnt++;//对ID进行更新
        }

    }


    //将keline格式的线进行转换
    for (int j = 0; j < this_object->forwframe_->keylsd.size(); ++j){
        Line l;
        cv::line_descriptor::KeyLine lsd = this_object->forwframe_->keylsd[j];
        // l.StartPt = lsd.getStartPoint();
        // l.EndPt = lsd.getEndPoint();
        cv::Point2f start_point=lsd.getStartPoint();
        cv::Point2f end_point= lsd.getEndPoint();
        //去除失真
        if(this_object->m_camera){//若不是空的话，就使用
            l.StartPt=this_object->undistortedPts(start_point,this_object->m_camera);//获得未失真的点，输入的为当前跟踪的点以及相机的参数
            l.EndPt=this_object->undistortedPts(end_point,this_object->m_camera);//获得未失真的点，输入的为当前跟踪的点以及相机的参数
        }
        else{
            ROS_ERROR("without undistortedPts !!!");
            l.StartPt=start_point;
            l.EndPt=end_point;
        }
        
        l.length = lsd.lineLength;
        this_object->forwframe_->vecLine.push_back(l);
    }

    // if(this_object->forwframe_->keylsd.size()!=0)//当当前帧检测的线不为0时，才赋值，不然已有的会被清空
        this_object->curframe_ = this_object->forwframe_;

    // // curframe_->vecLine
    // this_object->curframe_->vecLine=this_object->undistortedLineEndPoints();//去除失真(但每次使用就会存在内存益处的问题)
    mutex_threads.unlock();
}

void process_pointfeature(FeatureTracker *this_object,const cv::Mat time_surface, const dvs_msgs::msg::EventArray &last_event, const cv::Mat event_mat)
{
    // forw_pts.clear();//光流检测的特征点
    cv::Mat rightImg;//空的，用来辅助画图而已
    this_object->cur_pts.clear();//当前帧的特征点清

    
    if (this_object->prev_pts.size() > 0)////若上一帧的特征点大于0则实行光流跟踪
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
         // Step 1 通过opencv光流追踪给的状态位剔除outlier
        cv::calcOpticalFlowPyrLK(this_object->prev_img, this_object->cur_img, this_object->prev_pts, this_object->cur_pts, status, err, cv::Size(21, 21), 2);

        if(FLOW_BACK)//这是读入的参数，是否需要进行二次的光流检测
        {
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = this_object->prev_pts;

            cv::calcOpticalFlowPyrLK(this_object->cur_img, this_object->prev_img, this_object->cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 2); 
            // cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
            // cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && this_object->distance(this_object->prev_pts[i], reverse_pts[i]) <= 1)//原本要求distance 要小于0.5(设置为6)
                // if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 1 && time_surface.at<uchar>(cur_pts[i].y,cur_pts[i].x)!=TS_LK_THRESHOLD)//原本要求distance 要小于0.5(设置为6)
                {
                    status[i] = 1;//为1的时候会保留
                }
                else
                    status[i] = 0;
            }
        }

        for (int i = 0; i < int(this_object->cur_pts.size()); i++)
            if (status[i] && !inBorder(this_object->cur_pts[i]))
                status[i] = 0;
        reduceVector(this_object->prev_pts, status);//外点剔除，剔除无用的特征点后，还是得到prev_pts
        reduceVector(this_object->cur_pts, status);
        reduceVector(this_object->ids, status);// 特征点的id
        reduceVector(this_object->track_cnt, status);//被跟踪到的特征点的次数
        // reduceVector(cur_un_pts, status);// 去畸变后的坐标

        // reduceVector(forw_pts, status);
        // ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
        // printf("number of feature after LK-time_surface tracking %d\n", (int)cur_pts.size());
    }
   

    for (auto &n : this_object->track_cnt) // 被追踪到的是上一帧就存在的，因此追踪数+1
        n++;//上面已经追踪到了，把被追踪的次数++

    // TicToc t_detect_point;
    if (PUB_THIS_FRAME)//原来设置是：PUB_THIS_FRAME
    {
        // rejectWithF(); // Step 3 通过对级约束来剔除outlier（之前没有用），此处函数是基于flow point的，所以没有意义
        this_object->rejectWithF_event();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        this_object->setevent_Mask();//很重要，让特征点分布稀疏一些
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        // TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(this_object->cur_pts.size());//额外需要检测的点
        if (n_max_cnt > 0)
        {
            if(this_object->mask_arc.empty())
                cout << "mask is empty!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
            if (this_object->mask_arc.type() != CV_64FC1)
                cout << "mask type wrong!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
            if (this_object->mask_arc.size() != time_surface.size())
                cout << "wrong size!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
            // cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);//检测新的特征点
            good_arc_FeaturesToTrack(time_surface, last_event,this_object->n_pts, this_object->cur_pts, n_max_cnt, MIN_DIST, this_object->mask_arc);
            //画图用
            // good_arc_FeaturesToTrack(time_surface, last_event,n_pts, cur_pts, n_max_cnt, MIN_DIST, mask_arc, event_mat);
        }
        else
            this_object->n_pts.clear();//清空
        // ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        // ROS_DEBUG("add feature begins");
        // TicToc t_a;
        // addPoints();//把检测到的点，加入当前点
         for (auto &p : this_object->n_pts)//将n_pts给cur_pts，注意n_pts是进一步detected的点
        {
            this_object->cur_pts.push_back(p);//检测到的n_pts会放入当前的cur_pts里面
            // ids.push_back(n_id++);//可能是size还是那么多，就是number不一样？
            this_object->ids.push_back(-1);//新检测的特征点的id的值给-1
            this_object->track_cnt.push_back(1);
        }

        // printf("number of feature after event-corner detection %d\n", (int)cur_pts.size());

        // ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }
    // ROS_INFO("time cost of detecting event-feature: %f", t_detect_point.toc());
    // std::cout<<"PUB_THIS_FRAME="<<PUB_THIS_FRAME<<std::endl;

    this_object->cur_un_pts = this_object->undistortedPts(this_object->cur_pts, this_object->m_camera);//获得未失真的点，输入的为当前跟踪的点以及相机的参数
    this_object->pts_velocity = this_object->ptsVelocity(this_object->ids, this_object->cur_un_pts, this_object->cur_un_pts_map, this_object->prev_un_pts_map);//计算速度
    //得到的速度主要是给时间偏移用的

    if(SHOW_TRACK)//读入参数是否show跟踪，如果是的话，就运行下面函数，将特征点显示出来！
   {
        //  event_drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);//在tracking的time surface上画的
        this_object->event_drawTrack(event_mat, rightImg, this_object->ids, this_object->cur_pts, this_object->cur_right_pts, this_object->prevLeftPtsMap);
        this_object->event_drawTrack_two(this_object->cur_img, this_object->prev_img, this_object->ids, this_object->cur_pts, this_object->prev_pts, this_object->prevLeftPtsMap);//前后帧跟踪的结果输出
   }

   //执行完后，将当前帧的一些数据变为上一帧
    this_object->prev_img = this_object->cur_img;
    this_object->prev_pts = this_object->cur_pts;//上一帧的点由当前帧的点给出来，而当前帧的点由n_pts给出来，而n_pts由goodevent_FeaturesToTrack给出来
    this_object->prev_un_pts = this_object->cur_un_pts;
    this_object->prev_un_pts_map = this_object->cur_un_pts_map;//来自ptsVelocity，将特征点的id以及不失真的点坐标放到了一起，后面可以再次用于计算特征点的速率
    this_object->prev_time = this_object->cur_time;//同步很重要！！

    this_object->prevLeftPtsMap.clear();
    for(size_t i = 0; i < this_object->cur_pts.size(); i++)
        this_object->prevLeftPtsMap[this_object->ids[i]] = this_object->cur_pts[i];//这个跟cur_un_pts_map或者说prev_un_pts_map很类似？（画图需要用到）

}



void FeatureTracker::readEvent(const dvs_msgs::msg::EventArray &last_event, double _cur_time)//真正进行跟踪处理
{
    cv::Mat img;
    // TicToc t_r;
    cur_time = _cur_time;//当前的时间

    //基于event生成SAE mat（同时提取time surface）
    //初始化角点检测器
    if(FLAG_DETECTOR_NOSTART){
        FLAG_DETECTOR_NOSTART=false;
        detector.init(COL,ROW);//注意输入的值
    }

    detector.cur_event_mat=cv::Mat::zeros(cv::Size(COL, ROW), CV_8UC3);//先清空一下
    // TicToc t_create_sae;
    // 把所有的event放入SAE中(好像采用多线程，帮助不大)
    for (const dvs_msgs::msg::Event& e:last_event.events){
        detector.createSAE(e.ts, e.x, e.y, e.polarity);
    }
    // ROS_INFO("time cost of creating SAE: %f", t_create_sae.toc());
    cv::Mat event_mat=detector.cur_event_mat;//获得当前event的mat矩阵
    

    // TicToc t_ts;
    const cv::Mat time_surface_map=detector.SAEtoTimeSurface(cur_time);//产生的time surface（用sae_）特征点是基于SAE产生的，故此跟踪也应该采用SAE
    // ROS_INFO("time cost of creating time surface: %f", t_ts.toc());

    //用于回环检测的image
    // Image_loop=detector.SAE_toTimeSurface_withoutP_multi_thread(cur_time);//最原始的，不带极性的归一化TS


    time_surface_visualization=time_surface_map.clone();//用于可视化,输出time surface
    //   time_surface_visualization=ChangeEventstream2MatWithP(last_event,1);//用于可视化,输出event frame
    cv::Mat time_surface = time_surface_map.clone();//开始可以使用(用于辅助提取特征点以及二次光流检测)


    if (EQUALIZE)//均衡处理
    {  // 
        // TicToc t_c;
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();//默认参数
        clahe->apply(time_surface, img);
        cv::normalize(img, img, 0, 255, CV_MINMAX);
        // ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = time_surface;//将time surface赋给img


    // pubLoopImage(img,cur_time);//用作回环检测的


     //给线特征用的
    cv::Mat img_line;//给线特征用的
    img_line=img.clone();//保证点特征的tracking与线特征的描述子均在同一个环境下
    bool first_img = false;//是否第一帧
    if (cur_img.empty()) // 第一次输入图像，prev_img这个没用
    {
        prev_img = cur_img= img;//全部初始化
        
        // 也对线特征进行初始化
        mutex_threads.lock();
        curframe_.reset(new FrameLines);//当前已经有的，上一帧的
        curframe_->img = img_line;
        curframe_->event_img=event_mat;
        first_img = true;
        mutex_threads.unlock();
    }
    else//不是第一帧时，把当前帧给forw_img
    {
        // forw_img = img;//把当前帧给forw_img（光流跟踪的图像）原来代码用forw_img
        cur_img=img;//整个处理都是用cur_img(最后把当前的cur_img给prev_img)
    }

    // forw_pts.clear();//光流检测的特征点
    cv::Mat rightImg;//空的，用来辅助画图而已
    cur_pts.clear();//当前帧的特征点清


    //处理线特征
    std::thread process_linefeature_thread(process_linefeature,this,time_surface,first_img, event_mat);//img_line就是time_surface
    if (process_linefeature_thread.joinable())
        process_linefeature_thread.detach();

    
    if (prev_pts.size() > 0)////若上一帧的特征点大于0则实行光流跟踪
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
         // Step 1 通过opencv光流追踪给的状态位剔除outlier
        cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);//之前一直为2

        if(FLOW_BACK)//这是读入的参数，是否需要进行二次的光流检测
        {
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts;

            // cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 2); //之前一直用这个
            cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)//原本要求distance 要小于0.5(一直以来设置为1)
                // if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 1 && time_surface.at<uchar>(cur_pts[i].y,cur_pts[i].x)!=TS_LK_THRESHOLD)//原本要求distance 要小于0.5(设置为6)
                {
                    status[i] = 1;//为1的时候会保留
                }
                else
                    status[i] = 0;
            }
        }

        for (int i = 0; i < int(cur_pts.size()); i++)
            if (status[i] && !inBorder(cur_pts[i]))
                status[i] = 0;
        reduceVector(prev_pts, status);//外点剔除，剔除无用的特征点后，还是得到prev_pts
        reduceVector(cur_pts, status);
        reduceVector(ids, status);// 特征点的id
        reduceVector(track_cnt, status);//被跟踪到的特征点的次数
        // reduceVector(cur_un_pts, status);// 去畸变后的坐标

        // reduceVector(forw_pts, status);
        // ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
        //  printf("number of feature after LK-time_surface tracking %d\n", (int)cur_pts.size());
    }
   

    for (auto &n : track_cnt) // 被追踪到的是上一帧就存在的，因此追踪数+1
        n++;//上面已经追踪到了，把被追踪的次数++

    // TicToc t_detect_point;
    if (PUB_THIS_FRAME)//原来设置是：PUB_THIS_FRAME
    {
        // rejectWithF(); // Step 3 通过对级约束来剔除outlier（之前没有用），此处函数是基于flow point的，所以没有意义
        rejectWithF_event();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setevent_Mask();//很重要，让特征点分布稀疏一些
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        // TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());//额外需要检测的点
        if (n_max_cnt > 0)
        {
            if(mask_arc.empty())
                cout << "mask is empty!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
            if (mask_arc.type() != CV_64FC1)
                cout << "mask type wrong!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
            if (mask_arc.size() != time_surface.size())
                cout << "wrong size!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
            // cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);//检测新的特征点
            good_arc_FeaturesToTrack(time_surface, last_event,n_pts, cur_pts, n_max_cnt, MIN_DIST, mask_arc);
            //画图用
            // good_arc_FeaturesToTrack(time_surface, last_event,n_pts, cur_pts, n_max_cnt, MIN_DIST, mask_arc, event_mat);
        }
        else
            n_pts.clear();//清空
        // ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        // ROS_DEBUG("add feature begins");
        // TicToc t_a;
        // addPoints();//把检测到的点，加入当前点
         for (auto &p : n_pts)//将n_pts给cur_pts，注意n_pts是进一步detected的点
        {
            cur_pts.push_back(p);//检测到的n_pts会放入当前的cur_pts里面
            // ids.push_back(n_id++);//可能是size还是那么多，就是number不一样？
            ids.push_back(-1);//新检测的特征点的id的值给-1（第一个特征点给为-1）
            track_cnt.push_back(1);
        }

        // printf("number of feature after event-corner detection %d\n", (int)cur_pts.size());

        // ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }
    // ROS_INFO("time cost of detecting event-feature: %f", t_detect_point.toc());
    // std::cout<<"PUB_THIS_FRAME="<<PUB_THIS_FRAME<<std::endl;

    cur_un_pts.clear();
    cur_un_pts = undistortedPts(cur_pts, m_camera);//获得未失真的点，输入的为当前跟踪的点以及相机的参数
    pts_velocity.clear();
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);//计算速度
    //得到的速度主要是给时间偏移用的

    if(SHOW_TRACK)//读入参数是否show跟踪，如果是的话，就运行下面函数，将特征点显示出来！
   {
        //  event_drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);//在tracking的time surface上画的
        event_drawTrack(event_mat, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);
        event_drawTrack_two(cur_img, prev_img, ids, cur_pts, prev_pts, prevLeftPtsMap);//前后帧跟踪的结果输出
   }

   //执行完后，将当前帧的一些数据变为上一帧
    prev_img = cur_img;
    prev_pts = cur_pts;//上一帧的点由当前帧的点给出来，而当前帧的点由n_pts给出来，而n_pts由goodevent_FeaturesToTrack给出来
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;//来自ptsVelocity，将特征点的id以及不失真的点坐标放到了一起，后面可以再次用于计算特征点的速率
    prev_time = cur_time;//同步很重要！！

    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];//这个跟cur_un_pts_map或者说prev_un_pts_map很类似？（画图需要用到）

}


void FeatureTracker::readEvent(const dvs_msgs::msg::EventArray &last_event, double _cur_time, const Motion_correction_value measurements)//真正进行跟踪处理
{
    cv::Mat img;
    // TicToc t_r;
    cur_time = _cur_time;//当前的时间

    //基于event生成SAE mat（同时提取time surface）
    //初始化角点检测器
    if(FLAG_DETECTOR_NOSTART){
        FLAG_DETECTOR_NOSTART=false;
        // detector.init(COL,ROW);//注意输入的值
        detector.init(COL,ROW,fx,fy,cx,cy);;//注意初始化需要加入内参矩阵
    }

    // 需要对初始时间进行修正
    Motion_correction_value correct_time_measurement=measurements;
    correct_time_measurement.second.second.first[0]=last_event.events[0].ts;//当前事件流的时间进行修正处理
    correct_time_measurement.second.second.first[1]=rclcpp::Time(last_event.header.stamp).seconds();//当前事件流的时间进行修正处理

    detector.cur_event_mat=cv::Mat::zeros(cv::Size(COL, ROW), CV_8UC3);;//先清空一下
    // TicToc t_create_sae;
    // 把所有的event放入SAE中(好像采用多线程，帮助不大)(但加上运动补偿需要多线程)
    // for (const dvs_msgs::Event& e:last_event.events){
    //     // detector.createSAE(e.ts.toSec(), e.x, e.y, e.polarity);
    //     detector.createSAE(e.ts.toSec(), e.x, e.y, e.polarity, correct_time_measurement);//进行运动补偿
    // }

    int threadCount = Num_of_thread/2; //2;//采用2个线程
    std::thread threads_create_SAE[threadCount];   
    for (int i = 0; i < threadCount; i++)
    {
        //为每个线程分配任务
        int beginIndex = i*last_event.events.size()/threadCount;
        int length=last_event.events.size()/threadCount;
        threads_create_SAE[i] = std::thread(multi_thread_create_SAE_motion,last_event.events,beginIndex,length, correct_time_measurement);
    }
    //等待所有线程结束
    for(auto& thread_tmp:threads_create_SAE)
        if(thread_tmp.joinable())
            thread_tmp.join();


    // ROS_INFO("time cost of creating SAE: %f", t_create_sae.toc());
    cv::Mat event_mat=detector.cur_event_mat;//获得当前event的mat矩阵

    // TicToc t_ts;
    const cv::Mat time_surface_map=detector.SAEtoTimeSurface(cur_time);//产生的time surface（用sae_）特征点是基于SAE产生的，故此跟踪也应该采用SAE
    // ROS_INFO("time cost of creating time surface: %f", t_ts.toc());

    time_surface_visualization=time_surface_map.clone();//用于可视化,输出time surface
    //   time_surface_visualization=ChangeEventstream2MatWithP(last_event,1);//用于可视化,输出event frame
    cv::Mat time_surface = time_surface_map.clone();//开始可以使用(用于辅助提取特征点以及二次光流检测)


    if (EQUALIZE)//均衡处理
    {  // 
        // TicToc t_c;
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();//默认参数
        clahe->apply(time_surface, img);
        cv::normalize(img, img, 0, 255, CV_MINMAX);
        // ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = time_surface;//将time surface赋给img


    // pubLoopImage(img,cur_time);//用作回环检测的


     //给线特征用的
    cv::Mat img_line;//给线特征用的
    img_line=img.clone();//保证点特征的tracking与线特征的描述子均在同一个环境下
    bool first_img = false;//是否第一帧
    if (cur_img.empty()) // 第一次输入图像，prev_img这个没用
    {
        prev_img = cur_img= img;//全部初始化
        
        // 也对线特征进行初始化
        mutex_threads.lock();
        curframe_.reset(new FrameLines);//当前已经有的，上一帧的
        curframe_->img = img_line;
        curframe_->event_img=event_mat;
        first_img = true;
        mutex_threads.unlock();
    }
    else//不是第一帧时，把当前帧给forw_img
    {
        // forw_img = img;//把当前帧给forw_img（光流跟踪的图像）原来傻逼代码用forw_img
        cur_img=img;//整个处理都是用cur_img(最后把当前的cur_img给prev_img)
    }

    // forw_pts.clear();//光流检测的特征点
    cv::Mat rightImg;//空的，用来辅助画图而已
    cur_pts.clear();//当前帧的特征点清


    //处理线特征
    std::thread process_linefeature_thread(process_linefeature,this,time_surface,first_img, event_mat);//img_line就是time_surface
    if (process_linefeature_thread.joinable())
        process_linefeature_thread.detach();

    
    if (prev_pts.size() > 0)////若上一帧的特征点大于0则实行光流跟踪
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
         // Step 1 通过opencv光流追踪给的状态位剔除outlier
        cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 2);

        if(FLOW_BACK)//这是读入的参数，是否需要进行二次的光流检测
        {
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts;

            cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 2); 
            // cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
            // cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 1)//原本要求distance 要小于0.5(设置为6)
                // if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 1 && time_surface.at<uchar>(cur_pts[i].y,cur_pts[i].x)!=TS_LK_THRESHOLD)//原本要求distance 要小于0.5(设置为6)
                {
                    status[i] = 1;//为1的时候会保留
                }
                else
                    status[i] = 0;
            }
        }

        for (int i = 0; i < int(cur_pts.size()); i++)
            if (status[i] && !inBorder(cur_pts[i]))
                status[i] = 0;
        reduceVector(prev_pts, status);//外点剔除，剔除无用的特征点后，还是得到prev_pts
        reduceVector(cur_pts, status);
        reduceVector(ids, status);// 特征点的id
        reduceVector(track_cnt, status);//被跟踪到的特征点的次数
        // reduceVector(cur_un_pts, status);// 去畸变后的坐标

        // reduceVector(forw_pts, status);
        // ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
        //  printf("number of feature after LK-time_surface tracking %d\n", (int)cur_pts.size());
    }
   

    for (auto &n : track_cnt) // 被追踪到的是上一帧就存在的，因此追踪数+1
        n++;//上面已经追踪到了，把被追踪的次数++

    // TicToc t_detect_point;
    if (PUB_THIS_FRAME)//原来设置是：PUB_THIS_FRAME
    {
        // rejectWithF(); // Step 3 通过对级约束来剔除outlier（之前没有用），此处函数是基于flow point的，所以没有意义
        rejectWithF_event();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setevent_Mask();//很重要，让特征点分布稀疏一些
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        // TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());//额外需要检测的点
        if (n_max_cnt > 0)
        {
            if(mask_arc.empty())
                cout << "mask is empty!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
            if (mask_arc.type() != CV_64FC1)
                cout << "mask type wrong!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
            if (mask_arc.size() != time_surface.size())
                cout << "wrong size!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
            // cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);//检测新的特征点
            good_arc_FeaturesToTrack(time_surface, last_event,n_pts, cur_pts, n_max_cnt, MIN_DIST, mask_arc);
            //画图用
            // good_arc_FeaturesToTrack(time_surface, last_event,n_pts, cur_pts, n_max_cnt, MIN_DIST, mask_arc, event_mat);
        }
        else
            n_pts.clear();//清空
        // ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        // ROS_DEBUG("add feature begins");
        // TicToc t_a;
        // addPoints();//把检测到的点，加入当前点
         for (auto &p : n_pts)//将n_pts给cur_pts，注意n_pts是进一步detected的点
        {
            cur_pts.push_back(p);//检测到的n_pts会放入当前的cur_pts里面
            // ids.push_back(n_id++);//可能是size还是那么多，就是number不一样？
            ids.push_back(-1);//新检测的特征点的id的值给-1
            track_cnt.push_back(1);
        }

        // printf("number of feature after event-corner detection %d\n", (int)cur_pts.size());

        // ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }
    // ROS_INFO("time cost of detecting event-feature: %f", t_detect_point.toc());
    // std::cout<<"PUB_THIS_FRAME="<<PUB_THIS_FRAME<<std::endl;

    cur_un_pts = undistortedPts(cur_pts, m_camera);//获得未失真的点，输入的为当前跟踪的点以及相机的参数
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);//计算速度
    //得到的速度主要是给时间偏移用的

    if(SHOW_TRACK)//读入参数是否show跟踪，如果是的话，就运行下面函数，将特征点显示出来！
   {
        //  event_drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);//在tracking的time surface上画的
        event_drawTrack(event_mat, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);
        event_drawTrack_two(cur_img, prev_img, ids, cur_pts, prev_pts, prevLeftPtsMap);//前后帧跟踪的结果输出
   }

   //执行完后，将当前帧的一些数据变为上一帧
    prev_img = cur_img;
    prev_pts = cur_pts;//上一帧的点由当前帧的点给出来，而当前帧的点由n_pts给出来，而n_pts由goodevent_FeaturesToTrack给出来
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;//来自ptsVelocity，将特征点的id以及不失真的点坐标放到了一起，后面可以再次用于计算特征点的速率
    prev_time = cur_time;//同步很重要！！

    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];//这个跟cur_un_pts_map或者说prev_un_pts_map很类似？（画图需要用到）

}

void FeatureTracker::rejectWithF()//根据2D特征匹配关系计算基本矩阵（对极几何），并利用rasanc算法去除外点
{
    if (forw_pts.size() >= 8)//首先跟踪到的点的数目要大于8 （此时forw_pts为当前点）
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);//根据上一时刻和当前时刻，同时基于ransac计算
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        // ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        // ROS_INFO("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

void FeatureTracker::setreo_rejectWithF()
{
    if (cur_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_prev_pts(prev_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            setro_m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            setro_m_camera[0]->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_prev_pts, un_cur_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        reduceVector(prev_un_pts, status);
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, cur_pts.size(), 1.0 * cur_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

void FeatureTracker::rejectWithF_line(const std::vector<cv::line_descriptor::KeyLine> octave0_1, const std::vector<cv::line_descriptor::KeyLine>octave0_2,std::vector<cv::DMatch> good_matches)//根据2D特征匹配关系计算基本矩阵（对极几何），并利用rasanc算法去除外点(line feature 的两个角点)
{
    if (good_matches.size()*2 >= 8)//首先匹配到的线的两端的点的数目要大于8 
    {
        vector<cv::Point2f> un_cur_pts, un_forw_pts;
        for (int k = 0; k < good_matches.size(); ++k)
        {
            cv::DMatch mt = good_matches[k];

            cv::line_descriptor::KeyLine line1 = octave0_1[mt.queryIdx];  // trainIdx
            cv::line_descriptor::KeyLine line2 = octave0_2[mt.trainIdx];  //queryIdx

            cv::Point startPoint1 = cv::Point(int(line1.startPointX), int(line1.startPointY));
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(startPoint1.x, startPoint1.y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts.push_back( cv::Point2f(tmp_p.x(), tmp_p.y()));

            cv::Point endPoint1 = cv::Point(int(line1.endPointX), int(line1.endPointY));
            m_camera->liftProjective(Eigen::Vector2d(endPoint1.x, endPoint1.y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts.push_back( cv::Point2f(tmp_p.x(), tmp_p.y()));


            cv::Point startPoint2 = cv::Point(int(line2.startPointX), int(line2.startPointY));
            m_camera->liftProjective(Eigen::Vector2d(startPoint2.x, startPoint2.y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts.push_back( cv::Point2f(tmp_p.x(), tmp_p.y()) );
            
            cv::Point endPoint2 = cv::Point(int(line2.endPointX), int(line2.endPointY));
            m_camera->liftProjective(Eigen::Vector2d(endPoint2.x, endPoint2.y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts.push_back( cv::Point2f(tmp_p.x(), tmp_p.y()) );
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);//根据上一时刻和当前时刻，同时基于ransac计算

        int size_a = good_matches.size();//上一时刻的点（去除status之前的点）
        reduceVector(good_matches, status);
        // ROS_WARN("FM ransac: %d -> %lu: %f", size_a, good_matches.size(), 1.0 * good_matches.size() / size_a);
    }
}

//前端的外点剔除（fusion版本）
void FeatureTracker::rejectWithF_event()//fusion 版本
{
    if (cur_pts.size() >= 8)// 当前被追踪到的光流至少8个点
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_prev_pts(prev_pts.size());
        for (unsigned int i = 0; i < prev_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            //开始处理上一时刻的点
            m_camera->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);//先通过将像素坐标系转到归一化坐标系下并去畸变
            //得到归一化平面上的点
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());


            //开始处理当前时刻的点
            // 得到相机归一化坐标系的值
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);//先通过将像素坐标系转到归一化坐标系下并去畸变
            // 这里用一个虚拟相机，原因同样参考https://github.com/HKUST-Aerial-Robotics/VINS-Mono/issues/48
            // 这里有个好处就是对F_THRESHOLD和相机无关
            // 投影到虚拟相机的像素坐标系
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        // opencv接口计算本质矩阵，某种意义也是一种对级约束的outlier剔除
        cv::findFundamentalMat(un_prev_pts, un_cur_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = prev_pts.size();//上一时刻的点（去除status之前的点）
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        // ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, cur_pts.size(), 1.0 * cur_pts.size() / size_a);
        // if(size_a!=cur_pts.size())
        //     ROS_INFO("FM ransac: %d -> %lu: %f%", size_a, cur_pts.size(), 100.0*cur_pts.size()/size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}



/**
 * @brief 
 * 
 * @param[in] i 
 * @return true 
 * @return false 
 *  给新的特征点赋上id,越界就返回false
 */
bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);

    K_= m_camera->initUndistortRectifyMap(undist_map1_,undist_map2_);    
    fx = K_.at<float>(0, 0);
    fy = K_.at<float>(1, 1);
    cx = K_.at<float>(0, 2);
    cy = K_.at<float>(1, 2);
}

void FeatureTracker::stereo_readIntrinsicParameter(vector<string>& calib_file)
{
    for(int i=0; i<calib_file.size(); i++){
        ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
        camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
        setro_m_camera.push_back(camera); 
    }
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}


// 当前帧所有点统一去畸变，同时计算特征点速度，用来后续时间戳标定
void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        // // 有的之前去过畸变了，这里连同新的重新做一次
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }
    // caculate points velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}

cv::Point2f FeatureTracker::undistortedPts(cv::Point2f &pts, camodocal::CameraPtr cam)
{//计算去除失真的影响
    cv::Point2f un_pts;

    Eigen::Vector2d a(pts.x, pts.y);
    Eigen::Vector3d b;
    cam->liftProjective(a, b);//输入2d点，获得3d点
    un_pts=cv::Point2f(b.x() / b.z(), b.y() / b.z());

    return un_pts;//输出归一化平面的2d点，只有x、y
}


vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam)
{//计算去除失真的影响
    vector<cv::Point2f> un_pts;
    for (unsigned int i = 0; i < pts.size(); i++)
    {
        Eigen::Vector2d a(pts[i].x, pts[i].y);
        Eigen::Vector3d b;
        cam->liftProjective(a, b);//输入2d点，获得3d点
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }
    return un_pts;//输出归一化平面的2d点，只有x、y
}

vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                            map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts)
{//特征点的速度求解
//输入的为特征点的id，未失真的当前帧被跟踪的特征点xy，cur_un_pts_map（特征点id+特征点xy）
// pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);
    vector<cv::Point2f> pts_velocity;//每次是新建的，会返回
    cur_id_pts.clear();//也就是cur_un_pts_map
    for (unsigned int i = 0; i < ids.size(); i++)
    {
        // id->坐标的map
        cur_id_pts.insert(make_pair(ids[i], pts[i]));//将特征点的id与当前不失真的特征点组在一起（通过指针返回）
        //pts为cur_un_pts
    }

    // caculate points velocity
    if (!prev_id_pts.empty())//如果之前不失真的特征点不是空的
    {
        double dt = cur_time - prev_time;//时间的确定很重要！
        
        for (unsigned int i = 0; i < pts.size(); i++)
        {
            if(ids[i]!=-1){//不是第一个产生的特征
                std::map<int, cv::Point2f>::iterator it;
                it = prev_id_pts.find(ids[i]);//从之前特征点中的对应的位置中的->second（第二个元素）的x或y
                if (it != prev_id_pts.end())//若找到同一个特征点
                {
                    double v_x = (pts[i].x - it->second.x) / dt;
                    double v_y = (pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));//得到归一化平面上的速度
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));//设置为0
            }else{
                pts_velocity.push_back(cv::Point2f(0, 0));//设置为0
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)//第一帧的情况
        {
            pts_velocity.push_back(cv::Point2f(0, 0));//初始化为0
        }
    }
    return pts_velocity;//特征点的速度
}

// void  FeatureTracker::event_drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
//                                vector<int> &curLeftIds,
//                                vector<cv::Point2f> &curLeftPts, 
//                                vector<cv::Point2f> &curRightPts,
//                                map<int, cv::Point2f> &prevLeftPtsMap)
// {//画图的时候，imTrack是个全局变量，用于输出当前帧以及其检测的特征点
// //event_drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);
//     //int rows = imLeft.rows;
//     int cols = imLeft.cols;
//     if (!imRight.empty())
//         cv::hconcat(imLeft, imRight, imTrack);//将两张图像水平接起来
//     else
//         imTrack = imLeft.clone();
//     cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);//将黑白变为RGB

//     for (size_t j = 0; j < curLeftPts.size(); j++)
//     {
//         // double len = std::min(1.0, 1.0 * track_cnt[j] / 4);//跟踪数少于20次，len为0，这样为蓝色。当跟踪大于20次，len为1，这样点为红色
//         // cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);//BGR
//         if(track_cnt[j]>=2)
//             cv::circle(imTrack, curLeftPts[j], MIN_DIST/5, cv::Scalar(0, 0, 255),  2);
//         else
//             cv::circle(imTrack, curLeftPts[j], MIN_DIST/5, cv::Scalar(255, 0, 0),  2);
//     }

//     map<int, cv::Point2f>::iterator mapIt;
//     for (size_t i = 0; i < curLeftIds.size(); i++)
//     {
//         int id = curLeftIds[i];
//         mapIt = prevLeftPtsMap.find(id);//这是之前的特征点
//         if(mapIt != prevLeftPtsMap.end())
//         {
//             if(track_cnt[i]>=2){//大于2次的才画出来
//                 cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), MIN_DIST/5, 8, 0, 0.2);//绿色的箭头
//                 // cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);//绿色的
//             }
//         }
//     }
// }
//**************上面是在tracking上画的，这是在raw event上画的
void  FeatureTracker::event_drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap)
{//画图的时候，imTrack是个全局变量，用于输出当前帧以及其检测的特征点
//event_drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);
    //int rows = imLeft.rows;
    int cols = imLeft.cols;
    if (!imRight.empty())
        cv::hconcat(imLeft, imRight, imTrack);//将两张图像水平接起来
    else
        imTrack = imLeft.clone();
    // cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);//将黑白变为RGB

    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        // double len = std::min(1.0, 1.0 * track_cnt[j] / 4);//跟踪数少于20次，len为0，这样为蓝色。当跟踪大于20次，len为1，这样点为红色
        // cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);//BGR
        if(track_cnt[j]>=2)
            cv::circle(imTrack, curLeftPts[j], 3, cv::Scalar(0, 0, 255),  -1);
        else
            cv::circle(imTrack, curLeftPts[j], 3, cv::Scalar(0, 255, 0),  1);
    }

    map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < curLeftIds.size(); i++)
    {
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap.find(id);//这是之前的特征点
        if(mapIt != prevLeftPtsMap.end())
        {
            if(track_cnt[i]>=2){//大于2次的才画出来
                // cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 2, 8, 0, 0.2);//绿色的箭头

                Vector2d tmp_cur_un_pts (cur_un_pts[i].x, cur_un_pts[i].y);
                Vector2d tmp_pts_velocity (pts_velocity[i].x, pts_velocity[i].y);
                Vector3d tmp_prev_un_pts;
                tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                tmp_prev_un_pts.z() = 1;
                Vector2d tmp_prev_uv;
                m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                cv::arrowedLine(imTrack, curLeftPts[i], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(0, 255, 0), 2 , 8, 0);
                // cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);//绿色的
            }
        }
    }
}


void  FeatureTracker::event_drawTrack_two(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap)
{//画图的时候，imTrack是个全局变量，用于输出当前帧以及其检测的特征点
//event_drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);
    //int rows = imLeft.rows;
    int cols = imLeft.cols;
    if (!imRight.empty())
        cv::hconcat(imLeft, imRight, imTrack_two);//将两张图像水平接起来
    else
        imTrack_two = imLeft.clone();
    cv::cvtColor(imTrack_two, imTrack_two, CV_GRAY2RGB);//变为黑白

    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        if(track_cnt[j]>=2)
            cv::circle(imTrack_two, curLeftPts[j], MIN_DIST/2, cv::Scalar(0, 0, 255), -1);
        // else
        //     cv::circle(imTrack_two, curLeftPts[j], MIN_DIST/2, cv::Scalar(255, 0, 0), -1);
    }
    
    if (!imRight.empty())
    {
        map<int, cv::Point2f>::iterator mapIt;
        for (size_t i = 0; i < curLeftIds.size(); i++)
        {
            int id = curLeftIds[i];
            mapIt = prevLeftPtsMap.find(id);//这是之前的特征点
            cv::Point2f rightPt = mapIt->second;
            // cv::Point2f rightPt = curRightPts[i];
            rightPt.x += cols;
            if(mapIt != prevLeftPtsMap.end()){
                cv::circle(imTrack_two, rightPt, MIN_DIST/2, cv::Scalar(0, 255,255), -1);//将其点画出来，黄色
                cv::Point2f leftPt = curLeftPts[i];
                if(track_cnt[i]>=2){//大于4次的才画出来
                    cv::arrowedLine(imTrack_two, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0, 0.02);//绿色的
                }
            }
        }
    }
}


void FeatureTracker::event_drawTrack_two_line(const cv::Mat imageMat1, const cv::Mat imageMat2,
                          const std::vector<cv::line_descriptor::KeyLine> octave0_1, const std::vector<cv::line_descriptor::KeyLine>octave0_2,
                          const std::vector<cv::DMatch> good_matches)
{
    //会返回全局变量 imTrack_two_line
    cv::Mat img1,img2;
    //判断是否为三通道的，如果不是，那么就变为三通道的
    if (imageMat1.channels() != 3){
        cv::cvtColor(imageMat1, img1, cv::COLOR_GRAY2BGR);
    }
    else{
        img1 = imageMat1;//forwframe_
    }

    if (imageMat2.channels() != 3){
        cv::cvtColor(imageMat2, img2, cv::COLOR_GRAY2BGR);
    }
    else{
        img2 = imageMat2;//curframe_
    }

    cv::hconcat(img1, img2, imTrack_two_line);//将两张图像水平接起来

    for (int k = 0; k < good_matches.size(); ++k) {

        cv::DMatch mt = good_matches[k];

        cv::line_descriptor::KeyLine line1 = octave0_1[mt.queryIdx];  // trainIdx
        cv::line_descriptor::KeyLine line2 = octave0_2[mt.trainIdx];  //queryIdx

        cv::Point startPoint = cv::Point(int(line1.startPointX), int(line1.startPointY));
        cv::Point endPoint = cv::Point(int(line1.endPointX), int(line1.endPointY));
        // cv::line(img1, startPoint, endPoint, cv::Scalar(0, 255, 255),2 ,8);//画线（黄色）
        cv::line(imTrack_two_line, startPoint, endPoint, cv::Scalar(0, 255, 255),2 ,8);//画线（黄色）

        cv::Point startPoint2 = cv::Point(int(line2.startPointX)+img1.cols, int(line2.startPointY));
        cv::Point endPoint2 = cv::Point(int(line2.endPointX)+img1.cols, int(line2.endPointY));
        // cv::line(img2, startPoint2, endPoint2, cv::Scalar(0, 255, 0),2, 8);//画线(绿色)
        cv::line(imTrack_two_line, startPoint2, endPoint2, cv::Scalar(0, 255, 255),2, 8);//画线(黄色)

        // // 下面是画点
        // cv::circle(img2, startPoint2, MIN_DIST/2, cv::Scalar(0, 255,255), -1);//起点
        // cv::circle(img2, endPoint2, MIN_DIST/2, cv::Scalar(0, 255,255), -1);//终点
        
        //用箭头可视化一一对应的关系
        cv::arrowedLine(imTrack_two_line, startPoint, startPoint2, cv::Scalar(0, 255, 0), 1, 8, 0, 0.02);//绿色线
        cv::arrowedLine(imTrack_two_line, endPoint, endPoint2, cv::Scalar(0, 255, 0), 1, 8, 0, 0.02);//绿色线
    }

    // cv::hconcat(img1, img2, imTrack_two_line);//将两张图像水平接起来
}

void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap)
{
    //int rows = imLeft.rows;
    // ROS_WARN("left pts num: %d right num: %d", curLeftPts.size(), curRightPts.size()); 
    int cols = imLeft.cols;
    if (!imRight.empty())
        cv::hconcat(imLeft, imRight, imTrack);
    else
        imTrack = imLeft.clone();
    cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);

    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
        cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }
    if (!imRight.empty())
    {
        for (size_t i = 0; i < curRightPts.size(); i++)
        {
            cv::Point2f rightPt = curRightPts[i];
            rightPt.x += cols;
            cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
            //cv::Point2f leftPt = curLeftPtsTrackRight[i];
            //cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
        }
    }
    
    map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < curLeftIds.size(); i++)
    {
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap.find(id);
        if(mapIt != prevLeftPtsMap.end())
        {
            cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
        }
    }

    //draw prediction
    /*
    for(size_t i = 0; i < predict_pts_debug.size(); i++)
    {
        cv::circle(imTrack, predict_pts_debug[i], 2, cv::Scalar(0, 170, 255), 2);
    }
    */
    //printf("predict pts size %d \n", (int)predict_pts_debug.size());

    //cv::Mat imCur2Compress;
    //cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));
}

double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}
