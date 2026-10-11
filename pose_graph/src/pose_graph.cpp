#include "pose_graph.h"


// 第一部分主要是为了通过回环检测找到当前帧和候选帧的联系，并通过简单的紧耦合重定位将局部滑动窗口移动与过去的位姿对齐。
// 重定位模块主要包含回环检测,回环候选帧之间的特征匹配,紧耦合重定位三个部分
// 第二部分是为了保证基于重定位结果对过去的所有位姿进行全局优化。

PoseGraph::PoseGraph()//后端初始化的时候，就开启了4-DOF的位姿图优化。
{
     // 用于rviz的可视化（注意后端die 掉会导致相机框不动）
    posegraph_visualization = new CameraPoseVisualization(1.0, 0.0, 1.0, 1.0);
    posegraph_visualization->setScale(0.1);
    posegraph_visualization->setLineWidth(0.01);

// 生成一个线程，该线程用于进行4自由度全局优化
	t_optimization = std::thread(&PoseGraph::optimize4DoF, this);//4-DOF的位姿图优化。里面要处理的buf在（addKeyFrame）中添加了

 // 初始化一些变量
    earliest_loop_index = -1;
    t_drift = Eigen::Vector3d(0, 0, 0);
    yaw_drift = 0;
    r_drift = Eigen::Matrix3d::Identity();
    w_t_vio = Eigen::Vector3d(0, 0, 0);
    w_r_vio = Eigen::Matrix3d::Identity();
    global_index = 0;
    sequence_cnt = 0;
    sequence_loop.push_back(0);
    base_sequence = 1;
}

PoseGraph::~PoseGraph()
{
	t_optimization.join();
}

/**
 * @brief 注册一些发布publisher
 * 
 * @param[in] n 
 */
void PoseGraph::registerPub(rclcpp::Node::SharedPtr n)
{
    // "~/..." replicates ROS1's `ros::NodeHandle n("~")` private-namespace topics.
    pub_pg_path = n->create_publisher<nav_msgs::msg::Path>("~/pose_graph_path", 1000);
    pub_base_path = n->create_publisher<nav_msgs::msg::Path>("~/base_path", 1000);
    pub_pose_graph = n->create_publisher<visualization_msgs::msg::MarkerArray>("~/pose_graph", 1000);
    pub_loop_evio = n->create_publisher<geometry_msgs::msg::PoseStamped>("~/evio_loop", 1000);//发布回环的结果

    for (int i = 1; i < 10; i++)
        pub_path[i] = n->create_publisher<nav_msgs::msg::Path>("~/path_" + to_string(i), 1000);
}

void PoseGraph::loadVocabulary(std::string voc_path)// 加载二进制词袋库
{
    voc = new BriefVocabulary(voc_path);
    db.setVocabulary(*voc, false, 0);
}


//添加关键帧，完成了回环检测与闭环的过程
void PoseGraph::addKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop)// 添加关键帧，进行回环检测与闭环。回环检测核心入口函数
{
    //shift to base frame
    Vector3d vio_P_cur;
    Matrix3d vio_R_cur;

    // 建一个新的图像序列
    if (sequence_cnt != cur_kf->sequence) // 发生了sequence的跳变
    {
        sequence_cnt++;
         // 复位一些变量
        sequence_loop.push_back(0);
        w_t_vio = Eigen::Vector3d(0, 0, 0);
        w_r_vio = Eigen::Matrix3d::Identity();
        m_drift.lock();
        t_drift = Eigen::Vector3d(0, 0, 0);
        r_drift = Eigen::Matrix3d::Identity();
        m_drift.unlock();
    }
    
      // 更新一下VIO位姿
    //   getVioPose（）获取当前帧的位姿vio_P_cur、vio_R_cur并更新updateVioPose
    cur_kf->getVioPose(vio_P_cur, vio_R_cur);// 得到VIO节点的位姿（得到当前回环关键帧的R与P）
     // 回环修正后的消除累计误差的位姿（进行回环修正获得P与R）
    vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
    vio_R_cur = w_r_vio *  vio_R_cur;
    cur_kf->updateVioPose(vio_P_cur, vio_R_cur); // 更新VIO位姿
    cur_kf->index = global_index; // 赋值索引
    global_index++;
	int loop_index = -1;

    if (flag_detect_loop)//输入就是1了
    {
        TicToc tmp_t;
        // detectLoop（）回环检测，返回回环候选帧的索引loop_index
        loop_index = detectLoop(cur_kf, cur_kf->index);//// 进行回环检测，寻找候选的回环帧（获得回环帧的index）
    }
    else
    {
        addKeyFrameIntoVoc(cur_kf);//哪怕没有进行回环检测，也需要把当前帧放入词袋
    }
    // 若存在回环候选帧
	if (loop_index != -1) // 代表找到了有效的回环帧
	{
        //printf(" %d detect loop with %d \n", cur_kf->index, loop_index);
        // 获取回环候选帧
        KeyFrame* old_kf = getKeyFrame(loop_index); // 得到回环帧的指针（也就是当前帧对应的之前的回环检测的那个帧）

        // 当前帧与回环候选帧进行描述子匹配,如果成功则确定存在回环
        if (cur_kf->findConnection(old_kf)) // 如果确定两者回环（上面只是获得要计算回环的两帧，这里才是确定是否真的回环了）
        {   //earliest_loop_index为最早的回环候选帧
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)// 更新最早回环帧，用来确定全局优化的范围
                earliest_loop_index = loop_index;

// 计算当前帧与回环帧的相对位姿，纠正当前帧位姿w_P_cur、w_R_cur
            Vector3d w_P_old, w_P_cur, vio_P_cur;
            Matrix3d w_R_old, w_R_cur, vio_R_cur;
            old_kf->getVioPose(w_P_old, w_R_old);
            cur_kf->getVioPose(vio_P_cur, vio_R_cur);

            // 获取当前帧与回环帧的相对位姿relative_q、relative_t
            Vector3d relative_t;
            Quaterniond relative_q;
            relative_t = cur_kf->getLoopRelativeT();
            relative_q = (cur_kf->getLoopRelativeQ()).toRotationMatrix();
            
            //重新计算当前帧位姿w_P_cur、w_R_cur.  这就是回环矫正后当前帧的位姿
            w_P_cur = w_R_old * relative_t + w_P_old;
            w_R_cur = w_R_old * relative_q;

            //回环得到的位姿和VIO位姿之间的偏移量shift_r、shift_t
            double shift_yaw;
            Matrix3d shift_r;
            Vector3d shift_t; 
            // 回环矫正前的位姿认为是T_w'_cur
            shift_yaw = Utility::R2ypr(w_R_cur).x() - Utility::R2ypr(vio_R_cur).x();
            shift_r = Utility::ypr2R(Vector3d(shift_yaw, 0, 0));
            shift_t = w_P_cur - w_R_cur * vio_R_cur.transpose() * vio_P_cur; 

            // shift vio pose of whole sequence to the world frame
            // 如果存在多个图像序列，则将所有图像序列都合并到世界坐标系下
            if (old_kf->sequence != cur_kf->sequence && sequence_loop[cur_kf->sequence] == 0) // 如果这两个不是同一个sequence，并且当前sequence没有跟之前合并
            {  
                w_r_vio = shift_r;
                w_t_vio = shift_t;
                vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
                vio_R_cur = w_r_vio *  vio_R_cur;
                cur_kf->updateVioPose(vio_P_cur, vio_R_cur); // 更新当前帧位姿
                list<KeyFrame*>::iterator it = keyframelist.begin();
                for (; it != keyframelist.end(); it++)   // 同时把这个序列当前帧之间的位姿都更新过来
                {
                    if((*it)->sequence == cur_kf->sequence)
                    {
                        Vector3d vio_P_cur;
                        Matrix3d vio_R_cur;
                        (*it)->getVioPose(vio_P_cur, vio_R_cur);
                        vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
                        vio_R_cur = w_r_vio *  vio_R_cur;
                        (*it)->updateVioPose(vio_P_cur, vio_R_cur);
                    }
                }
                sequence_loop[cur_kf->sequence] = 1; // 代表这个序列已经跟之前的序列合并过了，这里实现的也就是一个序列的合并
            }

            // 将当前帧放入优化队列中(将会在4dof优化线程中处理(optimize4DoF()))
            m_optimize_buf.lock();
            optimize_buf.push(cur_kf->index); // 相当于通知4dof优化线程开始干活
            m_optimize_buf.unlock();
        }
	}
	
    
    m_keyframelist.lock();
    Vector3d P;
    Matrix3d R;

    // 获取VIO当前帧的位姿P、R，根据偏移量得到实际位姿
    cur_kf->getVioPose(P, R);
    P = r_drift * P + t_drift; // 根据全局优化后进行位姿调整
    R = r_drift * R;
    cur_kf->updatePose(P, R);//  更新当前帧的位姿P、R

     // 下面是可视化部分.发布path[sequence_cnt]
    Quaterniond Q{R};
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = secondsToRosTime(cur_kf->time_stamp);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
    pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
    pose_stamped.pose.position.z = P.z();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.orientation.w = Q.w();
    path[sequence_cnt].poses.push_back(pose_stamped);
    path[sequence_cnt].header = pose_stamped.header;

    //将这个结果发布
    pub_loop_evio->publish(pose_stamped);

// 保存闭环轨迹到VINS_RESULT_PATH
    if (SAVE_LOOP_PATH)//将回环的结果保存
    {
        // ofstream loop_path_file(VINS_RESULT_PATH, ios::app);
        // loop_path_file.setf(ios::fixed, ios::floatfield);
        // loop_path_file.precision(0);
        // loop_path_file << cur_kf->time_stamp * 1e9 << ",";
        // loop_path_file.precision(5);
        // loop_path_file  << P.x() << ","
        //       << P.y() << ","
        //       << P.z() << ","
        //     //   << Q.w() << ","
        //       << Q.x() << ","
        //       << Q.y() << ","
        //       << Q.z() << ","
        //       << Q.w() << ","
        //       << endl;
        // loop_path_file.close();
    }
    //draw local connection
    if (SHOW_S_EDGE)
    {
        list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
        for (int i = 0; i < 4; i++)
        {
            if (rit == keyframelist.rend())
                break;
            Vector3d conncected_P;
            Matrix3d connected_R;
            if((*rit)->sequence == cur_kf->sequence)
            {
                (*rit)->getPose(conncected_P, connected_R);
                posegraph_visualization->add_edge(P, conncected_P);
            }
            rit++;
        }
    }

        //当前帧与其回环帧连线
    if (SHOW_L_EDGE)
    {
        if (cur_kf->has_loop)
        {
            //printf("has loop \n");
            KeyFrame* connected_KF = getKeyFrame(cur_kf->loop_index);
            Vector3d connected_P,P0;
            Matrix3d connected_R,R0;
            connected_KF->getPose(connected_P, connected_R);
            //cur_kf->getVioPose(P0, R0);
            cur_kf->getPose(P0, R0);
            if(cur_kf->sequence > 0)
            {
                //printf("add loop into visual \n");
                posegraph_visualization->add_loopedge(P0, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0));
            }
            
        }
    }
    //posegraph_visualization->add_pose(P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0), Q);

	keyframelist.push_back(cur_kf); // 当前帧送进KF容器中
    publish();//发布topic：pub_pg_path、pub_path、pub_base_path
	m_keyframelist.unlock();
}


/**
 * @brief 加载KF
 * 
 * @param[in] cur_kf 
 * @param[in] flag_detect_loop 是否做回环检测
 */
void PoseGraph::loadKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop)
{
    cur_kf->index = global_index;
    global_index++;
    int loop_index = -1;
    if (flag_detect_loop)
       loop_index = detectLoop(cur_kf, cur_kf->index);// 进行回环检测
    else
    {
        addKeyFrameIntoVoc(cur_kf);
    }
    if (loop_index != -1)
    {
        printf(" %d detect loop with %d \n", cur_kf->index, loop_index);
        KeyFrame* old_kf = getKeyFrame(loop_index);
        if (cur_kf->findConnection(old_kf))
        {
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
                earliest_loop_index = loop_index;
            m_optimize_buf.lock();
            optimize_buf.push(cur_kf->index);
            m_optimize_buf.unlock();
        }
    }
    m_keyframelist.lock();
    Vector3d P;
    Matrix3d R;
    cur_kf->getPose(P, R);
    Quaterniond Q{R};
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = secondsToRosTime(cur_kf->time_stamp);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
    pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
    pose_stamped.pose.position.z = P.z();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.orientation.w = Q.w();
    base_path.poses.push_back(pose_stamped);
    base_path.header = pose_stamped.header;

    //draw local connection
    if (SHOW_S_EDGE)
    {
        list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
        for (int i = 0; i < 1; i++)
        {
            if (rit == keyframelist.rend())
                break;
            Vector3d conncected_P;
            Matrix3d connected_R;
            if((*rit)->sequence == cur_kf->sequence)
            {
                (*rit)->getPose(conncected_P, connected_R);
                posegraph_visualization->add_edge(P, conncected_P);
            }
            rit++;
        }
    }
    /*
    if (cur_kf->has_loop)
    {
        KeyFrame* connected_KF = getKeyFrame(cur_kf->loop_index);
        Vector3d connected_P;
        Matrix3d connected_R;
        connected_KF->getPose(connected_P,  connected_R);
        posegraph_visualization->add_loopedge(P, connected_P, SHIFT);
    }
    */

    keyframelist.push_back(cur_kf);
    //publish();
    m_keyframelist.unlock();
}

KeyFrame* PoseGraph::getKeyFrame(int index)//根据index找到对应的KF对象的指针
{
//    unique_lock<mutex> lock(m_keyframelist);
    list<KeyFrame*>::iterator it = keyframelist.begin();
    for (; it != keyframelist.end(); it++)   
    {
        if((*it)->index == index)
            break;
    }
    if (it != keyframelist.end())
        return *it;
    else
        return NULL;
}

//回环检测返回候选帧索引
// 从之前所有(关键)帧中,寻找候选的回环帧
//用于检测当前帧与先前帧是否可能存在回环，若存在则返回回环候选帧的索引。
int PoseGraph::detectLoop(KeyFrame* keyframe, int frame_index)// 进行回环检测，寻找候选的回环帧
{
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    if (DEBUG_IMAGE)
    {
        int feature_num = keyframe->keypoints.size();//特征点的数目
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        image_pool[frame_index] = compressed_image;//把全部放入一个image_pool中用于可视化
    }
    TicToc tmp_t;
    //first query; then add this frame into database!
    QueryResults ret;
    TicToc t_query;

    // 1.查询字典数据库，得到与每一帧的相似度评分ret
    db.query(keyframe->brief_descriptors, ret, 4, frame_index - 50);// 调用词袋查询接口，查询结果是ret，最多返回4个备选KF，查找距离当前至少50帧的KF
    //printf("query time: %f", t_query.toc());
    //cout << "Searching for Image " << frame_index << ". " << ret << endl;

    // 2.添加当前关键帧到字典数据库中
    TicToc t_add;
    db.add(keyframe->brief_descriptors);// 当然也会把当前帧送进数据库中，便于后续帧的查询
    //printf("add feature time: %f", t_add.toc());
    // ret[0] is the nearest neighbour's score. threshold change with neighour score

      // 3.通过相似度评分判断是否存在回环候选帧
    bool find_loop = false;
    cv::Mat loop_result;
    if (DEBUG_IMAGE)
    {
        loop_result = compressed_image.clone();
        if (ret.size() > 0)
            putText(loop_result, "neighbour score:" + to_string(ret[0].Score), cv::Point2f(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
    }

    // visual loop result 
    if (DEBUG_IMAGE)
    {
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            int tmp_index = ret[i].Id;
            auto it = image_pool.find(tmp_index);
            cv::Mat tmp_image = (it->second).clone();
            putText(tmp_image, "index:  " + to_string(tmp_index) + "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
            cv::hconcat(loop_result, tmp_image, loop_result);
        }
    }

        //确保与相邻帧具有好的相似度评分
    // a good match with its nerghbour
    if (ret.size() >= 1 &&ret[0].Score > 0.05) // ret按照得分大小降序排列的，这里确保返回的候选KF数目至少一个且得分满足要求
        for (unsigned int i = 1; i < ret.size(); i++)// 开始遍历其他候选帧
        {
            //if (ret[i].Score > ret[0].Score * 0.3)
            if (ret[i].Score > 0.015)// 如果有大于一个候选帧且得分满足要求
            {          
                find_loop = true;// 就认为找到回环了
                int tmp_index = ret[i].Id;
                if (DEBUG_IMAGE && 0)
                {
                    auto it = image_pool.find(tmp_index);
                    cv::Mat tmp_image = (it->second).clone();
                    putText(tmp_image, "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
                    cv::hconcat(loop_result, tmp_image, loop_result);
                }
            }

        }

//可视化检测回环候选帧的效果（应该检测还是比较多的）
    // if (DEBUG_IMAGE)
    // {
    //     cv::imshow("loop_result", loop_result);
    //     cv::waitKey(20);//若设置为0，则无限等待用户的按键事件，不然就是20ms
    // }

    // 4.对于索引值大于50的关键帧才考虑进行回环,即前50帧不回环
    //返回评分大于0.015的最早的关键帧索引min_index
    if (find_loop && frame_index > 50) // 认为找到了回环并且当前帧是第50帧以后
    {
        int min_index = -1;
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            if (min_index == -1 || (ret[i].Id < min_index && ret[i].Score > 0.015))// 寻找得分大于0.015的idx最小的那个帧，这样可以尽可能调整更多帧的位姿
                min_index = ret[i].Id;//返回最早的索引（对于检测到的一系列回环候选帧选择最早的）
        }
        return min_index; // 当前帧和min_index形成了回环
    }
    else
        return -1;// -1代表回环失败

}

void PoseGraph::addKeyFrameIntoVoc(KeyFrame* keyframe)
{
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    if (DEBUG_IMAGE)
    {
        int feature_num = keyframe->keypoints.size();
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        image_pool[keyframe->index] = compressed_image;
    }

    db.add(keyframe->brief_descriptors);
}

/**
 * @brief 如果找到回环，该线程就执行位姿优化
 * 
 */
void PoseGraph::optimize4DoF()//四自由度位姿图优化 (额外的线程运行的)
{
    while(true)
    {
        int cur_index = -1;
        int first_looped_index = -1;
        m_optimize_buf.lock();
        // 取出最新的形成回环的当前帧
        while(!optimize_buf.empty())
        {
            cur_index = optimize_buf.front(); //取出最新一个待优化帧作为当前帧
            first_looped_index = earliest_loop_index; // 找到最早的回环帧
            optimize_buf.pop();
        }
        m_optimize_buf.unlock();
        if (cur_index != -1)
        {
            printf("optimize pose graph \n");
            TicToc tmp_t;
            m_keyframelist.lock();
            KeyFrame* cur_kf = getKeyFrame(cur_index);// 取出当前帧对应的KF指针

            int max_length = cur_index + 1;// 预设最大长度，总之优化帧数不可能超过这么多

            // w^t_i   w^q_i
            double t_array[max_length][3];
            Quaterniond q_array[max_length];
            double euler_array[max_length][3];
            double sequence_array[max_length];

            // 定义一个ceres优化问题，这里只优化位移和yaw角
            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //options.minimizer_progress_to_stdout = true;
            //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
            options.max_num_iterations = 5;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(0.1);
            //loss_function = new ceres::CauchyLoss(1.0);
            ceres::LocalParameterization* angle_local_parameterization =
                AngleLocalParameterization::Create(); // 由于yaw不满足简单的增量更新方式，因此需要自定义其local param形式

            list<KeyFrame*>::iterator it;

            int i = 0;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)// 遍历KF的list
            {
                //找到第一个回环候选帧，
                //回环检测到帧以前的都略过
                if ((*it)->index < first_looped_index)// idx小于最早回环帧就算了
                    continue;
                (*it)->local_index = i;// 这个是在本次优化中的idx
                Quaterniond tmp_q;
                Matrix3d tmp_r;
                Vector3d tmp_t;
                (*it)->getVioPose(tmp_t, tmp_r);//  得到位姿
                tmp_q = tmp_r;
                t_array[i][0] = tmp_t(0);
                t_array[i][1] = tmp_t(1);
                t_array[i][2] = tmp_t(2);
                q_array[i] = tmp_q;

                // 四元数转欧拉角
                Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
                euler_array[i][0] = euler_angle.x();
                euler_array[i][1] = euler_angle.y();
                euler_array[i][2] = euler_angle.z();

                sequence_array[i] = (*it)->sequence;

                problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization); // 只有yaw角参与优化，成为参数块
                problem.AddParameterBlock(t_array[i], 3);

                //回环检测到的帧参数设为固定
                if ((*it)->index == first_looped_index || (*it)->sequence == 0)// 最早回环帧以及加载进来的地图保持不变，不进行优化
                {   
                    problem.SetParameterBlockConstant(euler_array[i]);
                    problem.SetParameterBlockConstant(t_array[i]);
                }

                //add edge
                //对于每个i, 只计算它之前五个的位置和yaw残差
                for (int j = 1; j < 5; j++)
                {
                  if (i - j >= 0 && sequence_array[i] == sequence_array[i-j])
                  {
                    Vector3d euler_conncected = Utility::R2ypr(q_array[i-j].toRotationMatrix());
                    Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
                    relative_t = q_array[i-j].inverse() * relative_t;
                    double relative_yaw = euler_array[i][0] - euler_array[i-j][0];
                    ceres::CostFunction* cost_function = FourDOFError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                   relative_yaw, euler_conncected.y(), euler_conncected.z());
                    problem.AddResidualBlock(cost_function, NULL, euler_array[i-j], 
                                            t_array[i-j], 
                                            euler_array[i], 
                                            t_array[i]);
                  }
                }

                //add loop edge
                
                if((*it)->has_loop)// 如果检测到回环
                {                     //必须回环检测的帧号大于或者等于当前帧的回环检测匹配帧号
                    assert((*it)->loop_index >= first_looped_index);
                    int connected_index = getKeyFrame((*it)->loop_index)->local_index;
                    Vector3d euler_conncected = Utility::R2ypr(q_array[connected_index].toRotationMatrix());
                    Vector3d relative_t;
                    relative_t = (*it)->getLoopRelativeT();
                    double relative_yaw = (*it)->getLoopRelativeYaw();
                    ceres::CostFunction* cost_function = FourDOFWeightError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                                               relative_yaw, euler_conncected.y(), euler_conncected.z());
                    problem.AddResidualBlock(cost_function, loss_function, euler_array[connected_index], 
                                                                  t_array[connected_index], 
                                                                  euler_array[i], 
                                                                  t_array[i]);
                    
                }
                
                if ((*it)->index == cur_index)
                    break;
                i++;
            }
            m_keyframelist.unlock();

            ceres::Solve(options, &problem, &summary);
            //std::cout << summary.BriefReport() << "\n";
            
            //printf("pose optimization time: %f \n", tmp_t.toc());
            /*
            for (int j = 0 ; j < i; j++)
            {
                printf("optimize i: %d p: %f, %f, %f\n", j, t_array[j][0], t_array[j][1], t_array[j][2] );
            }
            */
            m_keyframelist.lock();
            i = 0;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)            //根据优化后的参数更新参与优化的关键帧的位姿
            {
                if ((*it)->index < first_looped_index)
                    continue;
                Quaterniond tmp_q;
                tmp_q = Utility::ypr2R(Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
                Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
                Matrix3d tmp_r = tmp_q.toRotationMatrix();
                (*it)-> updatePose(tmp_t, tmp_r); // 更新位姿

                if ((*it)->index == cur_index)
                    break;
                i++;
            }
            //根据当前帧的drift，更新全部关键帧位姿
            Vector3d cur_t, vio_t;
            Matrix3d cur_r, vio_r;
            cur_kf->getPose(cur_t, cur_r);
            cur_kf->getVioPose(vio_t, vio_r);

            // 计算当前帧的VIO位姿和优化后位姿差
            m_drift.lock();
            yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(vio_r).x();
            r_drift = Utility::ypr2R(Vector3d(yaw_drift, 0, 0));
            t_drift = cur_t - r_drift * vio_t;
            m_drift.unlock();
            //cout << "t_drift " << t_drift.transpose() << endl;
            //cout << "r_drift " << Utility::R2ypr(r_drift).transpose() << endl;
            //cout << "yaw drift " << yaw_drift << endl;

            it++;
            for (; it != keyframelist.end(); it++)// 遍历当前帧之后的所有位姿，根据算得的位姿差进行调整
            {
                Vector3d P;
                Matrix3d R;
                (*it)->getVioPose(P, R);
                P = r_drift * P + t_drift;
                R = r_drift * R;
                (*it)->updatePose(P, R);
            }
            m_keyframelist.unlock();
            updatePath();// 可视化部分
        }

        std::chrono::milliseconds dura(2000);
        std::this_thread::sleep_for(dura);
    }
}

void PoseGraph::updatePath()
{
    m_keyframelist.lock();
    list<KeyFrame*>::iterator it;
    for (int i = 1; i <= sequence_cnt; i++)
    {
        path[i].poses.clear();
    }
    base_path.poses.clear();
    posegraph_visualization->reset();

    if (SAVE_LOOP_PATH)
    {
        ofstream loop_path_file_tmp(VINS_RESULT_PATH, ios::out);
        loop_path_file_tmp.close();
    }

    for (it = keyframelist.begin(); it != keyframelist.end(); it++)
    {
        Vector3d P;
        Matrix3d R;
        (*it)->getPose(P, R);
        Quaterniond Q;
        Q = R;
//        printf("path p: %f, %f, %f\n",  P.x(),  P.z(),  P.y() );

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = secondsToRosTime((*it)->time_stamp);
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
        pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
        pose_stamped.pose.position.z = P.z();
        pose_stamped.pose.orientation.x = Q.x();
        pose_stamped.pose.orientation.y = Q.y();
        pose_stamped.pose.orientation.z = Q.z();
        pose_stamped.pose.orientation.w = Q.w();
        //在此将回环的结果发布出来
        if((*it)->sequence == 0)
        {
            base_path.poses.push_back(pose_stamped);
            base_path.header = pose_stamped.header;
        }
        else
        {
            path[(*it)->sequence].poses.push_back(pose_stamped);
            path[(*it)->sequence].header = pose_stamped.header;
        }

        if (SAVE_LOOP_PATH)
        {
            ofstream loop_path_file(VINS_RESULT_PATH, ios::app);
            loop_path_file.setf(ios::fixed, ios::floatfield);
            // loop_path_file.precision(20);
            loop_path_file << (*it)->time_stamp << " ";
            loop_path_file.precision(20);
            loop_path_file  << P.x() << " "
                  << P.y() << " "
                  << P.z() << " "
                //   << Q.w() << ","
                  << Q.x() << " "
                  << Q.y() << " "
                  << Q.z() << " "
                  << Q.w()
                  << endl;
            loop_path_file.close();
        }
        //draw local connection
        if (SHOW_S_EDGE)
        {
            list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
            list<KeyFrame*>::reverse_iterator lrit;
            for (; rit != keyframelist.rend(); rit++)  
            {  
                if ((*rit)->index == (*it)->index)
                {
                    lrit = rit;
                    lrit++;
                    for (int i = 0; i < 4; i++)
                    {
                        if (lrit == keyframelist.rend())
                            break;
                        if((*lrit)->sequence == (*it)->sequence)
                        {
                            Vector3d conncected_P;
                            Matrix3d connected_R;
                            (*lrit)->getPose(conncected_P, connected_R);
                            posegraph_visualization->add_edge(P, conncected_P);
                        }
                        lrit++;
                    }
                    break;
                }
            } 
        }
        if (SHOW_L_EDGE)
        {
            if ((*it)->has_loop && (*it)->sequence == sequence_cnt)
            {
                
                KeyFrame* connected_KF = getKeyFrame((*it)->loop_index);
                Vector3d connected_P;
                Matrix3d connected_R;
                connected_KF->getPose(connected_P, connected_R);
                //(*it)->getVioPose(P, R);
                (*it)->getPose(P, R);
                if((*it)->sequence > 0)
                {
                    posegraph_visualization->add_loopedge(P, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0));
                }
            }
        }

    }
    publish();
    m_keyframelist.unlock();
}


void PoseGraph::savePoseGraph()
{
    m_keyframelist.lock();
    TicToc tmp_t;
    FILE *pFile;
    printf("pose graph path: %s\n",POSE_GRAPH_SAVE_PATH.c_str());
    printf("pose graph saving... \n");
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    pFile = fopen (file_path.c_str(),"w");
    //fprintf(pFile, "index time_stamp Tx Ty Tz Qw Qx Qy Qz loop_index loop_info\n");
    list<KeyFrame*>::iterator it;
    for (it = keyframelist.begin(); it != keyframelist.end(); it++)
    {
        std::string image_path, descriptor_path, brief_path, keypoints_path;
        if (DEBUG_IMAGE)
        {
            image_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_image.png";
            imwrite(image_path.c_str(), (*it)->image);
        }
        Quaterniond VIO_tmp_Q{(*it)->vio_R_w_i};
        Quaterniond PG_tmp_Q{(*it)->R_w_i};
        Vector3d VIO_tmp_T = (*it)->vio_T_w_i;
        Vector3d PG_tmp_T = (*it)->T_w_i;

        fprintf (pFile, " %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %f %f %f %f %f %f %f %f %d\n",(*it)->index, (*it)->time_stamp, 
                                    VIO_tmp_T.x(), VIO_tmp_T.y(), VIO_tmp_T.z(), 
                                    PG_tmp_T.x(), PG_tmp_T.y(), PG_tmp_T.z(), 
                                    VIO_tmp_Q.w(), VIO_tmp_Q.x(), VIO_tmp_Q.y(), VIO_tmp_Q.z(), 
                                    PG_tmp_Q.w(), PG_tmp_Q.x(), PG_tmp_Q.y(), PG_tmp_Q.z(), 
                                    (*it)->loop_index, 
                                    (*it)->loop_info(0), (*it)->loop_info(1), (*it)->loop_info(2), (*it)->loop_info(3),
                                    (*it)->loop_info(4), (*it)->loop_info(5), (*it)->loop_info(6), (*it)->loop_info(7),
                                    (int)(*it)->keypoints.size());

        // write keypoints, brief_descriptors   vector<cv::KeyPoint> keypoints vector<BRIEF::bitset> brief_descriptors;
        assert((*it)->keypoints.size() == (*it)->brief_descriptors.size());
        brief_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_briefdes.dat";
        std::ofstream brief_file(brief_path, std::ios::binary);
        keypoints_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_keypoints.txt";
        FILE *keypoints_file;
        keypoints_file = fopen(keypoints_path.c_str(), "w");
        for (int i = 0; i < (int)(*it)->keypoints.size(); i++)
        {
            brief_file << (*it)->brief_descriptors[i] << endl;
            fprintf(keypoints_file, "%f %f %f %f\n", (*it)->keypoints[i].pt.x, (*it)->keypoints[i].pt.y, 
                                                     (*it)->keypoints_norm[i].pt.x, (*it)->keypoints_norm[i].pt.y);
        }
        brief_file.close();
        fclose(keypoints_file);
    }
    fclose(pFile);

    printf("save pose graph time: %f s\n", tmp_t.toc() / 1000);
    m_keyframelist.unlock();
}

void PoseGraph::loadPoseGraph()//加载先前位姿图
{
    TicToc tmp_t;
    FILE * pFile;
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    printf("lode pose graph from: %s \n", file_path.c_str());
    printf("pose graph loading...\n");
    pFile = fopen (file_path.c_str(),"r");
    if (pFile == NULL)
    {
        printf("lode previous pose graph error: wrong previous pose graph path or no previous pose graph \n the system will start with new pose graph \n");
        return;
    }
    int index;
    double time_stamp;
    double VIO_Tx, VIO_Ty, VIO_Tz;
    double PG_Tx, PG_Ty, PG_Tz;
    double VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz;
    double PG_Qw, PG_Qx, PG_Qy, PG_Qz;
    double loop_info_0, loop_info_1, loop_info_2, loop_info_3;
    double loop_info_4, loop_info_5, loop_info_6, loop_info_7;
    int loop_index;
    int keypoints_num;
    Eigen::Matrix<double, 8, 1 > loop_info;
    int cnt = 0;
    while (fscanf(pFile,"%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d", &index, &time_stamp, 
                                    &VIO_Tx, &VIO_Ty, &VIO_Tz, 
                                    &PG_Tx, &PG_Ty, &PG_Tz, 
                                    &VIO_Qw, &VIO_Qx, &VIO_Qy, &VIO_Qz, 
                                    &PG_Qw, &PG_Qx, &PG_Qy, &PG_Qz, 
                                    &loop_index,
                                    &loop_info_0, &loop_info_1, &loop_info_2, &loop_info_3, 
                                    &loop_info_4, &loop_info_5, &loop_info_6, &loop_info_7,
                                    &keypoints_num) != EOF) 
    {
        /*
        printf("I read: %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d\n", index, time_stamp, 
                                    VIO_Tx, VIO_Ty, VIO_Tz, 
                                    PG_Tx, PG_Ty, PG_Tz, 
                                    VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz, 
                                    PG_Qw, PG_Qx, PG_Qy, PG_Qz, 
                                    loop_index,
                                    loop_info_0, loop_info_1, loop_info_2, loop_info_3, 
                                    loop_info_4, loop_info_5, loop_info_6, loop_info_7,
                                    keypoints_num);
        */
        cv::Mat image;
        std::string image_path, descriptor_path;
        if (DEBUG_IMAGE)
        {
            image_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_image.png";
            image = cv::imread(image_path.c_str(), 0);
        }

        Vector3d VIO_T(VIO_Tx, VIO_Ty, VIO_Tz);
        Vector3d PG_T(PG_Tx, PG_Ty, PG_Tz);
        Quaterniond VIO_Q;
        VIO_Q.w() = VIO_Qw;
        VIO_Q.x() = VIO_Qx;
        VIO_Q.y() = VIO_Qy;
        VIO_Q.z() = VIO_Qz;
        Quaterniond PG_Q;
        PG_Q.w() = PG_Qw;
        PG_Q.x() = PG_Qx;
        PG_Q.y() = PG_Qy;
        PG_Q.z() = PG_Qz;
        Matrix3d VIO_R, PG_R;
        VIO_R = VIO_Q.toRotationMatrix();
        PG_R = PG_Q.toRotationMatrix();
        Eigen::Matrix<double, 8, 1 > loop_info;
        loop_info << loop_info_0, loop_info_1, loop_info_2, loop_info_3, loop_info_4, loop_info_5, loop_info_6, loop_info_7;

        if (loop_index != -1)
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
            {
                earliest_loop_index = loop_index;
            }

        // load keypoints, brief_descriptors   
        string brief_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_briefdes.dat";
        std::ifstream brief_file(brief_path, std::ios::binary);
        string keypoints_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_keypoints.txt";
        FILE *keypoints_file;
        keypoints_file = fopen(keypoints_path.c_str(), "r");
        vector<cv::KeyPoint> keypoints;
        vector<cv::KeyPoint> keypoints_norm;
        vector<BRIEF::bitset> brief_descriptors;
        for (int i = 0; i < keypoints_num; i++)
        {
            BRIEF::bitset tmp_des;
            brief_file >> tmp_des;
            brief_descriptors.push_back(tmp_des);
            cv::KeyPoint tmp_keypoint;
            cv::KeyPoint tmp_keypoint_norm;
            double p_x, p_y, p_x_norm, p_y_norm;
            if(!fscanf(keypoints_file,"%lf %lf %lf %lf", &p_x, &p_y, &p_x_norm, &p_y_norm))
                printf(" fail to load pose graph \n");
            tmp_keypoint.pt.x = p_x;
            tmp_keypoint.pt.y = p_y;
            tmp_keypoint_norm.pt.x = p_x_norm;
            tmp_keypoint_norm.pt.y = p_y_norm;
            keypoints.push_back(tmp_keypoint);
            keypoints_norm.push_back(tmp_keypoint_norm);
        }
        brief_file.close();
        fclose(keypoints_file);

        KeyFrame* keyframe = new KeyFrame(time_stamp, index, VIO_T, VIO_R, PG_T, PG_R, image, loop_index, loop_info, keypoints, keypoints_norm, brief_descriptors);
        loadKeyFrame(keyframe, 0);
        if (cnt % 20 == 0)
        {
            publish();
        }
        cnt++;
    }
    fclose (pFile);
    printf("load pose graph time: %f s\n", tmp_t.toc()/1000);
    base_sequence = 0;
}

void PoseGraph::publish()
{
    for (int i = 1; i <= sequence_cnt; i++)
    {
        //if (sequence_loop[i] == true || i == base_sequence)
        if (1 || i == base_sequence)
        {
            pub_pg_path->publish(path[i]);
            pub_path[i]->publish(path[i]);
            posegraph_visualization->publish_by(pub_pose_graph, path[sequence_cnt].header);
        }
    }
    base_path.header.frame_id = "world";
    pub_base_path->publish(base_path);
    //posegraph_visualization->publish_by(pub_pose_graph, path[sequence_cnt].header);
}

void PoseGraph::updateKeyFrameLoop(int index, Eigen::Matrix<double, 8, 1 > &_loop_info)//进行回环更新
{
    KeyFrame* kf = getKeyFrame(index);//获取关键帧
    kf->updateLoop(_loop_info);
    if (abs(_loop_info(7)) < 30.0 && Vector3d(_loop_info(0), _loop_info(1), _loop_info(2)).norm() < 20.0)
    //  if (abs(_loop_info(7)) < 60.0 && Vector3d(_loop_info(0), _loop_info(1), _loop_info(2)).norm() < 100.0)//根keyframe中写到的对应
    {
        if (FAST_RELOCALIZATION)//开启回环的时候才触发
        {
            KeyFrame* old_kf = getKeyFrame(kf->loop_index);// 得到回环帧信息
            Vector3d w_P_old, w_P_cur, vio_P_cur;
            Matrix3d w_R_old, w_R_cur, vio_R_cur;
            old_kf->getPose(w_P_old, w_R_old);//老的关键帧的pose
            kf->getVioPose(vio_P_cur, vio_R_cur);//新的关键帧的pose

            //然后求他们之间的（R与T）
            Vector3d relative_t;
            Quaterniond relative_q;
            relative_t = kf->getLoopRelativeT();
            relative_q = (kf->getLoopRelativeQ()).toRotationMatrix();
            // 得到T_loop_cur
            w_P_cur = w_R_old * relative_t + w_P_old;
            w_R_cur = w_R_old * relative_q;
            double shift_yaw;
            Matrix3d shift_r;
            Vector3d shift_t; 
             // 更新VIO位姿和修正位姿的delta pose
            shift_yaw = Utility::R2ypr(w_R_cur).x() - Utility::R2ypr(vio_R_cur).x();
            shift_r = Utility::ypr2R(Vector3d(shift_yaw, 0, 0));
            shift_t = w_P_cur - w_R_cur * vio_R_cur.transpose() * vio_P_cur; 

            m_drift.lock();
            yaw_drift = shift_yaw;
            r_drift = shift_r;
            t_drift = shift_t;
            m_drift.unlock();
        }
    }
}