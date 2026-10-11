#include "keyframe.h"

template <typename Derived>




static void reduceVector(vector<Derived> &v, vector<uchar> status)//用于剔除status为0的点
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

KeyFrame::KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, cv::Mat &_image,
		           vector<cv::Point3f> &_point_3d, vector<cv::Point2f> &_point_2d_uv, vector<cv::Point2f> &_point_2d_norm,
		           vector<double> &_point_id, int _sequence, dvs_msgs::msg::EventArray &_event_feature_point)
{ //时间，第几个窗口，当前关键帧的T与R，跟踪的图片，当前关键帧的地图点（每一个地图点的世界坐标系3D点，相机坐标系下的归一化坐标以及像素坐标+特征点的id），序列
	time_stamp = _time_stamp;//时间
	index = _index;//第几个窗口
	vio_T_w_i = _vio_T_w_i;//当前关键帧的T与R
	vio_R_w_i = _vio_R_w_i;
	//这里应该就是用于记录原本的T与R以及回环后的
	T_w_i = vio_T_w_i;
	R_w_i = vio_R_w_i;
	origin_vio_T = vio_T_w_i;		
	origin_vio_R = vio_R_w_i;
	image = _image.clone();//计算回环时，用来匹配的图片，可以用time surface也可以用event map
	cv::resize(image, thumbnail, cv::Size(80, 60));// 这个缩小尺寸应该是为了可视化(后面是可视化thumbnail的)
	point_3d = _point_3d;
	point_2d_uv = _point_2d_uv;
	point_2d_norm = _point_2d_norm;
	point_id = _point_id;
	has_loop = false;// 默认还没有检测到回环
	loop_index = -1;//初始化回环的索引为-1
	has_fast_point = false;
	loop_info << 0, 0, 0, 0, 0, 0, 0, 0;//记录两帧之间相对位姿（x,y,z,qw,qx,qy,qz,yaw）
	sequence = _sequence;//序列号
	computeWindowBRIEFPoint();//计算窗口中的描述子(已有特征的描述子)
	// computeBRIEFPoint();//里面计算了当前匹配图像中的关键点（额外通过FAST检测来提取的）以及描述子
	computeBRIEFPointfromevent(_event_feature_point);

	if(!DEBUG_IMAGE)//如果不可视化的话，就清空掉（	//不在位姿图中保存该图像，则将图像释放掉）
		image.release();
}

// create keyframe online
/**
 * @brief Construct a new Key Frame:: Key Frame object，创建一个KF对象，计算已有特征点的描述子，同时额外提取fast角点并计算描述子
 * 
 * @param[in] _time_stamp KF的时间戳
 * @param[in] _index KF的索引
 * @param[in] _vio_T_w_i vio节点中的位姿
 * @param[in] _vio_R_w_i 
 * @param[in] _image 对应的原图
 * @param[in] _point_3d KF对应VIO节点中的世界坐标
 * @param[in] _point_2d_uv 像素坐标
 * @param[in] _point_2d_norm 归一化相机坐标
 * @param[in] _point_id 地图点的idx
 * @param[in] _sequence 序列号
 */
KeyFrame::KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, cv::Mat &_image,
		           vector<cv::Point3f> &_point_3d, vector<cv::Point2f> &_point_2d_uv, vector<cv::Point2f> &_point_2d_norm,
		           vector<double> &_point_id, int _sequence)
{ //时间，第几个窗口，当前关键帧的T与R，跟踪的图片，当前关键帧的地图点（每一个地图点的世界坐标系3D点，相机坐标系下的归一化坐标以及像素坐标+特征点的id），序列
	time_stamp = _time_stamp;//时间
	index = _index;//第几个窗口
	vio_T_w_i = _vio_T_w_i;//当前关键帧的T与R
	vio_R_w_i = _vio_R_w_i;
	//这里应该就是用于记录原本的T与R以及回环后的
	T_w_i = vio_T_w_i;
	R_w_i = vio_R_w_i;
	origin_vio_T = vio_T_w_i;		
	origin_vio_R = vio_R_w_i;
	image = _image.clone();//计算回环时，用来匹配的图片，可以用time surface也可以用event map
	cv::resize(image, thumbnail, cv::Size(80, 60));// 这个缩小尺寸应该是为了可视化(后面是可视化thumbnail的)
	point_3d = _point_3d;
	point_2d_uv = _point_2d_uv;
	point_2d_norm = _point_2d_norm;
	point_id = _point_id;
	has_loop = false;// 默认还没有检测到回环
	loop_index = -1;//初始化回环的索引为-1
	has_fast_point = false;
	loop_info << 0, 0, 0, 0, 0, 0, 0, 0;//记录两帧之间相对位姿（x,y,z,qw,qx,qy,qz,yaw）
	sequence = _sequence;//序列号
	computeWindowBRIEFPoint();//计算窗口中的描述子(已有特征的描述子)
	computeBRIEFPoint();//里面计算了当前匹配图像中的关键点（额外通过FAST检测来提取的）以及描述子
	//现在问题就在于，这个函数是通过fast获取当前匹配图像中的关键点及其描述子
	//而computeWindowBRIEFPoint函数，则是根据前端获得的特征点，提取其描述子
	// 就相当于用ARC*的特征及其描述子与fast特征及其描述子进行匹配。这是严重有问题的
	// 改进：
	// 1、这里也要用前端的特征点（savecomputeWindowBRIEFPoint();）
	// 2、基于time surface的描述子提取改进

	// savecomputeWindowBRIEFPoint();

	if(!DEBUG_IMAGE)//如果不可视化的话，就清空掉（	//不在位姿图中保存该图像，则将图像释放掉）
		image.release();
}

/**
 * @brief Construct a new Key Frame:: Key Frame object从已有的地图中加载KF，就是一些简单的赋值操作
 * 
 * @param[in] _time_stamp 
 * @param[in] _index 
 * @param[in] _vio_T_w_i 
 * @param[in] _vio_R_w_i 
 * @param[in] _T_w_i 
 * @param[in] _R_w_i 
 * @param[in] _image 
 * @param[in] _loop_index 
 * @param[in] _loop_info 
 * @param[in] _keypoints 
 * @param[in] _keypoints_norm 
 * @param[in] _brief_descriptors 
 */
// load previous keyframe
KeyFrame::KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, Vector3d &_T_w_i, Matrix3d &_R_w_i,
					cv::Mat &_image, int _loop_index, Eigen::Matrix<double, 8, 1 > &_loop_info,
					vector<cv::KeyPoint> &_keypoints, vector<cv::KeyPoint> &_keypoints_norm, vector<BRIEF::bitset> &_brief_descriptors)
{
	time_stamp = _time_stamp;
	index = _index;
	//vio_T_w_i = _vio_T_w_i;
	//vio_R_w_i = _vio_R_w_i;
	vio_T_w_i = _T_w_i;
	vio_R_w_i = _R_w_i;
	T_w_i = _T_w_i;
	R_w_i = _R_w_i;
	if (DEBUG_IMAGE)
	{
		image = _image.clone();
		cv::resize(image, thumbnail, cv::Size(80, 60));
	}
	if (_loop_index != -1)
		has_loop = true;
	else
		has_loop = false;
	loop_index = _loop_index;
	loop_info = _loop_info;
	has_fast_point = false;
	sequence = 0;
	keypoints = _keypoints;
	keypoints_norm = _keypoints_norm;
	brief_descriptors = _brief_descriptors;//这个应该是有之前的pose graph的时候才用的吧
}


/**
 * @brief 计算已有特征点的描述子
 * 这里其实计算的是没有增加新的关键点的情况下计算的关键帧中的描述子，
 * 当这里计算得到了当前关键帧中的窗口描述子（存放在window_brief_descriptors中）后，
 * 其他关键帧要计算和这个关键帧是否形成闭环，就可以用其他关键帧中的描述子和该关键帧中的窗口描述子一一一算相似度，
 * 如果相似度评分达到设置的阈值，则可以认为该帧就是闭环候选帧。
 * 相比于computeBRIEFPoint函数中对新检测的500个关键点进行描述子计算来说，
 * 这里相当于是对关键帧中老的关键点进行描述子计算。
 */
void KeyFrame::computeWindowBRIEFPoint()
{
	BriefExtractor extractor(BRIEF_PATTERN_FILE.c_str());// 定义一个描述子计算的对象（time surface的描述子是否需要额外定义？）
	//遍历关键帧中的2d像素坐标关键点
	for(int i = 0; i < (int)point_2d_uv.size(); i++)
	{
	    cv::KeyPoint key;
	    key.pt = point_2d_uv[i];// 关键帧中的像素坐标用来计算描述子
	    window_keypoints.push_back(key);// 转成opencv格式的特征点坐标
	}
	extractor(image, window_keypoints, window_brief_descriptors);// 计算VIO节点提取的特征点的描述子
	//window_keypoints为窗口中的关键点
	//窗口中关键点的描述子
}


/**
 * @brief   额外提取fast特征点并计算描述子   （在关键帧图像中检测了500个新的特征点并计算所有特征点的描述子）
 * 由于闭环是使用BRIEF描述子的DBow2词袋进行检测的
 * 而前端feature_tracker中检测到的关键点数太少，对于闭环检测远远不够。
 * 因此在posegraph当中会对新来的 KeyFrame 即后端非线性优化刚处理完的关键帧，
 * 再检测出 500 个 FAST 关键点进行闭环检测的时候使用。
 * 同时对所有新老角点进行 BRIEF描述子计算。
 * 然后，在searchByBRIEFDes中计算当前帧与词袋的相似度分数，并与关键帧数据库中所有帧进行对比,并进行闭环一致性检测，获得闭环的候选帧。
 */
void KeyFrame::computeBRIEFPoint()
{
	BriefExtractor extractor(BRIEF_PATTERN_FILE.c_str());
	const int fast_th = 20; // corner detector response threshold
	if(1)//原本设置为1，那么下面就肯定不执行啦？？？？    （threshold指的是中心像素与周围像素强度的差的阈值）
		cv::FAST(image, keypoints, fast_th, true);//提取fast 特征点    （true是nonmaxSuppression是为了去除特征点聚集的情况）
	else
	{
		vector<cv::Point2f> tmp_pts;
		/**
        * 因为后边要做闭环检测，前端提取的关键点数太少，这里从图像image中提取500个角点
        * cv::goodFeaturesToTrack参数介绍如下：
        * 第一个参数是输入图像（8位或32位单通道图）。
        * 第二个参数是检测到的所有角点，类型为vector或数组，由实际给定的参数类型而定。如果是vector，那么它应该是一个包含cv::Point2f的vector对象；如果类型是cv::Mat,那么它的每一行对应一个角点，点的x、y位置分别是两列。
        * 第三个参数用于限定检测到的点数的最大值。
        * 第四个参数表示检测到的角点的质量水平（通常是0.10到0.01之间的数值，不能大于1.0）。
        * 第五个参数用于区分相邻两个角点的最小距离（小于这个距离得点将进行合并）。
        * 第六个参数是mask，如果指定，它的维度必须和输入图像一致，且在mask值为0处不进行角点检测。
        * 第七个参数是blockSize，表示在计算角点时参与运算的区域大小，常用值为3，但是如果图像的分辨率较高则可以考虑使用较大一点的值。
        * 第八个参数用于指定角点检测的方法，如果是true则使用Harris角点检测，false则使用Shi Tomasi算法。
        * 第九个参数是在使用Harris算法时使用，最好使用默认值0.04。
        * */
		cv::goodFeaturesToTrack(image, tmp_pts, 500, 0.01, 10);
		for(int i = 0; i < (int)tmp_pts.size(); i++)//遍历图像中新提取的角点，存入keypoints当中
		{
		    cv::KeyPoint key;
		    key.pt = tmp_pts[i];
		    keypoints.push_back(key);
		}
	}
	// ROS_INFO("the size of keypoints in loop detection using FAST:%d",keypoints.size());
	extractor(image, keypoints, brief_descriptors);//计算该关键帧中对应特征点的描述子
	for (int i = 0; i < (int)keypoints.size(); i++)
	{
		Eigen::Vector3d tmp_p;
		m_camera->liftProjective(Eigen::Vector2d(keypoints[i].pt.x, keypoints[i].pt.y), tmp_p);	// 将像素坐标得到去畸变的归一化相机坐标
		cv::KeyPoint tmp_norm;
		tmp_norm.pt = cv::Point2f(tmp_p.x()/tmp_p.z(), tmp_p.y()/tmp_p.z());// 再归一化一下
		keypoints_norm.push_back(tmp_norm);
	}
}


void KeyFrame::computeBRIEFPointfromevent(const dvs_msgs::msg::EventArray &_event_feature_point)
{
	BriefExtractor extractor(BRIEF_PATTERN_FILE.c_str());//读取 构建字典时使用的相同的Brief模板文件，构造BriefExtractor
	const int fast_th = 20; // corner detector response threshold

	//设置一个mask来使得点更加均匀
	// cv::Mat time_surface_mask=cv::Mat(ROW, COL, CV_64FC1, cv::Scalar(0.0));//值为255.0的点不检测(每次重置一下)

	//开始特征点提取
	if(0)//原本设置为1，那么下面就肯定不执行啦？？？？    （threshold指的是中心像素与周围像素强度的差的阈值）
		cv::FAST(image, keypoints, fast_th, true);//提取fast 特征点    （true是nonmaxSuppression是为了去除特征点聚集的情况）
	else
	{	
		keypoints.clear();//之前获得的都是清空了的
		for(const auto& e:_event_feature_point.events){//遍历所有的feature
		//  if(image.at<uchar>(e.y,e.x)!=0){//该处的time surface要不为0
			// if(image.at<uchar>(e.y,e.x)!=TS_LK_THRESHOLD){//该处的time surface要不为128
				// if(time_surface_mask.at<double>(e.y,e.x)!=255.0){//该处的mask不在之前的点的附近
					cv::KeyPoint key;
		    		key.pt = cv::Point2f((float)e.x, (float)e.y);
		    		keypoints.push_back(key);
					// cv::circle(time_surface_mask, cv::Point(e.x,e.y), 6, 255.0, -1);		
				// }	
			// }
			// }
		}
		// ROS_INFO("the size of event keypoints in loop detection is:%d",keypoints.size());
	}
	extractor(image, keypoints, brief_descriptors);//计算该关键帧中对应特征点的描述子
	for (int i = 0; i < (int)keypoints.size(); i++)
	{
		Eigen::Vector3d tmp_p;
		m_camera->liftProjective(Eigen::Vector2d(keypoints[i].pt.x, keypoints[i].pt.y), tmp_p);	// 将像素坐标得到去畸变的归一化相机坐标
		cv::KeyPoint tmp_norm;
		tmp_norm.pt = cv::Point2f(tmp_p.x()/tmp_p.z(), tmp_p.y()/tmp_p.z());// 再归一化一下
		keypoints_norm.push_back(tmp_norm);
	}
}


// 保留旧的特征点与描述子
void KeyFrame::savecomputeWindowBRIEFPoint()
{
	//将keypoint push back
	for(int i=0;i< (int) window_keypoints.size();i++){
		keypoints.push_back(window_keypoints[i]);//关键点
		brief_descriptors.push_back(window_brief_descriptors[i]);//描述子
		// keypoints_norm.push_back(point_2d_norm[i]);//归一化的点point_2d_norm(格式不一样)
		 cv::KeyPoint temp_point_2d_norm;
		 temp_point_2d_norm.pt=point_2d_norm[i];
		 keypoints_norm.push_back(temp_point_2d_norm);
	}
}

void BriefExtractor::operator() (const cv::Mat &im, vector<cv::KeyPoint> &keys, vector<BRIEF::bitset> &descriptors) const
{
  m_brief.compute(im, keys, descriptors);// 调用dbow的接口计算描述子
}

/**
 * @brief 暴力匹配法，通过遍历所有的候选描述子得到最佳匹配
 * 
 * @param[in] window_descriptor 当前帧的一个描述子
 * @param[in] descriptors_old 回环帧的描述子集合
 * @param[in] keypoints_old 回环帧像素坐标集合
 * @param[in] keypoints_old_norm 回环帧归一化坐标集合
 * @param[out] best_match 最佳匹配的像素坐标
 * @param[out] best_match_norm 最佳匹配的归一化相机坐标
 * @return true 
 * @return false 
 */
bool KeyFrame::searchInAera(const BRIEF::bitset window_descriptor,
                            const std::vector<BRIEF::bitset> &descriptors_old,
                            const std::vector<cv::KeyPoint> &keypoints_old,
                            const std::vector<cv::KeyPoint> &keypoints_old_norm,
                            cv::Point2f &best_match,
                            cv::Point2f &best_match_norm)
{
    cv::Point2f best_pt;
    int bestDist = 128;
    int bestIndex = -1;
    for(int i = 0; i < (int)descriptors_old.size(); i++)//// 遍历回环帧所有描述子
    {

        int dis = HammingDis(window_descriptor, descriptors_old[i]);// 计算两个描述子之间的得分
        if(dis < bestDist)// 找到匹配得分最高的
        {
            bestDist = dis;
            bestIndex = i;
        }
    }
    //printf("best dist %d", bestDist);
    if (bestIndex != -1 && bestDist < 80)//被匹配了，且汉明距离少于80的点
    {
      best_match = keypoints_old[bestIndex].pt;
      best_match_norm = keypoints_old_norm[bestIndex].pt;
      return true;
    }
    else
      return false;
}

/**
 * @brief (该函数的作用是: 将此关键帧对象的描述子依次和某个回环帧描述子进行BRIEF描述子匹配，得到匹配结果
 * 
 * @param[out] matched_2d_old 匹配回环帧点的像素坐标集合（回环帧匹配后的二维坐标）
 * @param[out] matched_2d_old_norm 匹配回环帧点的归一化相机坐标集合（回环帧匹配后的二维归一化坐标）
 * @param[out] status 状态位（匹配状态，成功为1）
 * @param[in] descriptors_old 回环帧（之前的）的描述子集合
 * @param[in] keypoints_old 回环帧（之前的）的像素坐标
 * @param[in] keypoints_old_norm 回环帧（之前的）的归一化坐标
 */
void KeyFrame::searchByBRIEFDes(std::vector<cv::Point2f> &matched_2d_old,
								std::vector<cv::Point2f> &matched_2d_old_norm,
                                std::vector<uchar> &status,
                                const std::vector<BRIEF::bitset> &descriptors_old,
                                const std::vector<cv::KeyPoint> &keypoints_old,
                                const std::vector<cv::KeyPoint> &keypoints_old_norm)
{
    for(int i = 0; i < (int)window_brief_descriptors.size(); i++) // 遍历当前的光流用的角点来进行描述子匹配
    {
        cv::Point2f pt(0.f, 0.f);//要输出的像素平面的点
        cv::Point2f pt_norm(0.f, 0.f);//要输出的相机归一化平面上的点
		// 进行暴力匹配
		//对关键帧中每个特征点的描述子与回环帧的所有描述子匹配，
		// 如果能找到汉明距离小于80的最小值和索引即为该特征点的最佳匹配，相应的status置为1
        if (searchInAera(window_brief_descriptors[i], descriptors_old, keypoints_old, keypoints_old_norm, pt, pt_norm))
          status.push_back(1);// 匹配上了状态位置1
        else
          status.push_back(0);// 没匹配上状态位置0
        matched_2d_old.push_back(pt);// 对应的像素坐标和归一化坐标存起来
        matched_2d_old_norm.push_back(pt_norm);
    }

}


void KeyFrame::FundmantalMatrixRANSAC(const std::vector<cv::Point2f> &matched_2d_cur_norm,
                                      const std::vector<cv::Point2f> &matched_2d_old_norm,
                                      vector<uchar> &status)
{
	int n = (int)matched_2d_cur_norm.size();
	for (int i = 0; i < n; i++)
		status.push_back(0);
    if (n >= 8)
    {
        vector<cv::Point2f> tmp_cur(n), tmp_old(n);
        for (int i = 0; i < (int)matched_2d_cur_norm.size(); i++)
        {
            double FOCAL_LENGTH = 460.0;
            double tmp_x, tmp_y;
            tmp_x = FOCAL_LENGTH * matched_2d_cur_norm[i].x + COL / 2.0;
            tmp_y = FOCAL_LENGTH * matched_2d_cur_norm[i].y + ROW / 2.0;
            tmp_cur[i] = cv::Point2f(tmp_x, tmp_y);

            tmp_x = FOCAL_LENGTH * matched_2d_old_norm[i].x + COL / 2.0;
            tmp_y = FOCAL_LENGTH * matched_2d_old_norm[i].y + ROW / 2.0;
            tmp_old[i] = cv::Point2f(tmp_x, tmp_y);
        }
        cv::findFundamentalMat(tmp_cur, tmp_old, cv::FM_RANSAC, 3.0, 0.9, status);
    }
}

/**
 * @brief 通过PNP对当前帧和回环是否构成回环进行校验
 * 
 * @param[in] matched_2d_old_norm 回环帧2d归一化坐标
 * @param[in] matched_3d 当前帧3d地图点
 * @param[out] status 
 * @param[out] PnP_T_old 
 * @param[out] PnP_R_old 
 */
void KeyFrame::PnPRANSAC(const vector<cv::Point2f> &matched_2d_old_norm,
                         const std::vector<cv::Point3f> &matched_3d,
                         std::vector<uchar> &status,
                         Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old)
{
	//for (int i = 0; i < matched_3d.size(); i++)
	//	printf("3d x: %f, y: %f, z: %f\n",matched_3d[i].x, matched_3d[i].y, matched_3d[i].z );
	//printf("match size %d \n", matched_3d.size());
    cv::Mat r, rvec, t, D, tmp_r;
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
    Matrix3d R_inital;
    Vector3d P_inital;
    Matrix3d R_w_c = origin_vio_R * qic;  // 转成相机坐标系
    Vector3d T_w_c = origin_vio_T + origin_vio_R * tic;

    R_inital = R_w_c.inverse();
    P_inital = -(R_inital * T_w_c);

    cv::eigen2cv(R_inital, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_inital, t);

    cv::Mat inliers;
    TicToc t_pnp_ransac;

    if (CV_MAJOR_VERSION < 3)
        solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, 10.0 / 460.0, 100, inliers);
    else
    {
        if (CV_MINOR_VERSION < 2)
            solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, sqrt(10.0 / 460.0), 0.99, inliers);
        else
            solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, 10.0 / 460.0, 0.99, inliers);

    }
	// solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, 10.0 / 460.0, 0.99, inliers);
	// 参考点在世界坐标系下的点集
	// 参考点在相机像平面的坐标
	//   K 相机内参
	// D 相机畸变系数
	// 旋转矩阵
	// 平移向量
	// 若果求解PnP使用迭代算法，初始值可以使用猜测的初始值（true），也可以使用解析求解的结果作为初始值（false）
	// Ransac算法的迭代次数，这只是初始值，根据估计外点的概率，可以进一步缩小迭代次数；（此值函数内部是会不断改变的）,所以一开始可以赋一个大的值。
// Ransac筛选内点和外点的距离阈值，这个根据估计内点的概率和每个点的均方差（假设误差按照高斯分布）可以计算出此阈值。
//  confidence=0.99  此值与计算采样（迭代）次数有关。此值代表从n个样本中取s个点，N次采样可以使s个点全为内点的概率。
// inliers为返回内点的序列。为矩阵形式


    for (int i = 0; i < (int)matched_2d_old_norm.size(); i++)
        status.push_back(0);// 初始化状态位全是0

    for( int i = 0; i < inliers.rows; i++)
    {
        int n = inliers.at<int>(i);
        status[n] = 1;//确定为inliers就置为1
    }
	// 转回eigen，以及Tcw -> Twc -> Twi
    cv::Rodrigues(rvec, r);
    Matrix3d R_pnp, R_w_c_old;
    cv::cv2eigen(r, R_pnp);
    R_w_c_old = R_pnp.transpose();
    Vector3d T_pnp, T_w_c_old;
    cv::cv2eigen(t, T_pnp);
    T_w_c_old = R_w_c_old * (-T_pnp);

    PnP_R_old = R_w_c_old * qic.transpose();// 这是是回环帧在VIO坐标系下的位姿
    PnP_T_old = T_w_c_old - PnP_R_old * tic;

}

/**
 * @brief 寻找两帧之间联系，确定是否回环 （该函数的主要目的是寻找并建立关键帧与回环帧之间的匹配关系，返回True即为确定构成回环。）
 * 
 * @param[in] old_kf 
 * @return true 
 * @return false 
 */
bool KeyFrame::findConnection(KeyFrame* old_kf)///当前帧与传入的回环候选帧(old_kf)进行描述子匹配,如果成功则确定存在回环
{
	TicToc tmp_t;
	//printf("find Connection\n");
	vector<cv::Point2f> matched_2d_cur, matched_2d_old;
	vector<cv::Point2f> matched_2d_cur_norm, matched_2d_old_norm;
	vector<cv::Point3f> matched_3d;
	vector<double> matched_id;
	vector<uchar> status;

	matched_3d = point_3d;
	matched_2d_cur = point_2d_uv;
	matched_2d_cur_norm = point_2d_norm;
	matched_id = point_id;

	TicToc t_match;
	#if 0
		if (SAVE_LOOP_MATCH)    
	    {
	        cv::Mat gray_img, loop_match_img;
	        cv::Mat old_img = old_kf->image;
	        cv::hconcat(image, old_img, gray_img);
	        cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
	        for(int i = 0; i< (int)point_2d_uv.size(); i++)
	        {
	            cv::Point2f cur_pt = point_2d_uv[i];
	            cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for(int i = 0; i< (int)old_kf->keypoints.size(); i++)
	        {
	            cv::Point2f old_pt = old_kf->keypoints[i].pt;
	            old_pt.x += COL;
	            cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        ostringstream path;
	        path << "/home/kwanwaipang/catkin_ws_dvs/src/EVIO/pose_graph/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "0raw_point.jpg";
	        cv::imwrite( path.str().c_str(), loop_match_img);
	    }
	#endif
	//printf("search by des\n");
	
	// 1、将关键帧与回环帧进行BRIEF描述子匹配，并剔除匹配失败的点
	// 通过描述子来check是否构成回环，去除大部分的点
	searchByBRIEFDes(matched_2d_old, matched_2d_old_norm, status, old_kf->brief_descriptors, old_kf->keypoints, old_kf->keypoints_norm);
	//获取匹配得到的matched_2d_old与matched_2d_old_norm
	// 操作跟光流追踪类似，根据状态位进行筛选
	reduceVector(matched_2d_cur, status);// 当前帧的像素坐标
	reduceVector(matched_2d_old, status);// 回环帧的像素坐标
	reduceVector(matched_2d_cur_norm, status);// 当前帧的归一化坐标
	reduceVector(matched_2d_old_norm, status);// 回环帧的归一化坐标
	reduceVector(matched_3d, status);// 当前帧对应的VIO中3d世界坐标
	reduceVector(matched_id, status);// 当前帧对应的地图点索引
	//printf("search by des finish\n");

	#if 0 
		if (SAVE_LOOP_MATCH)
	    {
			int gap = 10;
        	cv::Mat gap_image(ROW, gap, CV_8UC1, cv::Scalar(255, 255, 255));
            cv::Mat gray_img, loop_match_img;
            cv::Mat old_img = old_kf->image;
            cv::hconcat(image, gap_image, gap_image);
            cv::hconcat(gap_image, old_img, gray_img);
            cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
	        for(int i = 0; i< (int)matched_2d_cur.size(); i++)
	        {
	            cv::Point2f cur_pt = matched_2d_cur[i];
	            cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for(int i = 0; i< (int)matched_2d_old.size(); i++)
	        {
	            cv::Point2f old_pt = matched_2d_old[i];
	            old_pt.x += (COL + gap);
	            cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for (int i = 0; i< (int)matched_2d_cur.size(); i++)
	        {
	            cv::Point2f old_pt = matched_2d_old[i];
	            old_pt.x +=  (COL + gap);
	            cv::line(loop_match_img, matched_2d_cur[i], old_pt, cv::Scalar(0, 255, 0), 1, 8, 0);
	        }

	        ostringstream path, path1, path2;
	        path <<  "/home/kwanwaipang/catkin_ws_dvs/src/EVIO/pose_graph/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "1descriptor_match.jpg";
	        cv::imwrite( path.str().c_str(), loop_match_img);
	        /*
	        path1 <<  "/home/tony-ws1/raw_data/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "1descriptor_match_1.jpg";
	        cv::imwrite( path1.str().c_str(), image);
	        path2 <<  "/home/tony-ws1/raw_data/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "1descriptor_match_2.jpg";
	        cv::imwrite( path2.str().c_str(), old_img);	        
	        */
	        
	    }
	#endif

	status.clear();
	/*
	FundmantalMatrixRANSAC(matched_2d_cur_norm, matched_2d_old_norm, status);
	reduceVector(matched_2d_cur, status);
	reduceVector(matched_2d_old, status);
	reduceVector(matched_2d_cur_norm, status);
	reduceVector(matched_2d_old_norm, status);
	reduceVector(matched_3d, status);
	reduceVector(matched_id, status);
	*/
	#if 0
		if (SAVE_LOOP_MATCH)
	    {
			int gap = 10;
        	cv::Mat gap_image(ROW, gap, CV_8UC1, cv::Scalar(255, 255, 255));
            cv::Mat gray_img, loop_match_img;
            cv::Mat old_img = old_kf->image;
            cv::hconcat(image, gap_image, gap_image);
            cv::hconcat(gap_image, old_img, gray_img);
            cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
	        for(int i = 0; i< (int)matched_2d_cur.size(); i++)
	        {
	            cv::Point2f cur_pt = matched_2d_cur[i];
	            cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for(int i = 0; i< (int)matched_2d_old.size(); i++)
	        {
	            cv::Point2f old_pt = matched_2d_old[i];
	            old_pt.x += (COL + gap);
	            cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for (int i = 0; i< (int)matched_2d_cur.size(); i++)
	        {
	            cv::Point2f old_pt = matched_2d_old[i];
	            old_pt.x +=  (COL + gap) ;
	            cv::line(loop_match_img, matched_2d_cur[i], old_pt, cv::Scalar(0, 255, 0), 1, 8, 0);
	        }

	        ostringstream path;
	        path <<  "/home/kwanwaipang/catkin_ws_dvs/src/EVIO/pose_graph/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "2fundamental_match.jpg";
	        cv::imwrite( path.str().c_str(), loop_match_img);
	    }
	#endif

	Eigen::Vector3d PnP_T_old;
	Eigen::Matrix3d PnP_R_old;
	Eigen::Vector3d relative_t;
	Quaterniond relative_q;
	double relative_yaw;
	// 2、如果能匹配的特征点能达到最小回环匹配个数，则用RANSAC PnP检测再去除误匹配的点，
	// printf("num of match point before PnPRANSAC %d \n", (int)matched_2d_cur.size());
	if ((int)matched_2d_cur.size() > MIN_LOOP_NUM)// 判断匹配上的点数目大小够不够
	{
		// printf("num of match point before PnPRANSAC %d \n", (int)matched_2d_cur.size());
		status.clear();
	    PnPRANSAC(matched_2d_old_norm, matched_3d, status, PnP_T_old, PnP_R_old);// 进行PNP几何校验，利用当前帧的地图点（3d）和回环帧的归一化相机坐标（2d）进行计算
	   // 根据状态位进行瘦身 
		reduceVector(matched_2d_cur, status);
	    reduceVector(matched_2d_old, status);
	    reduceVector(matched_2d_cur_norm, status);
	    reduceVector(matched_2d_old_norm, status);
	    reduceVector(matched_3d, status);
	    reduceVector(matched_id, status);

		// printf("num of match point after PnPRANSAC %d \n", (int)matched_2d_cur.size());

		// 3、将此关键帧和回环帧拼接起来，将对应的匹配点相连以绘制回环匹配图，并发布为pub_match_img。
	    #if 1
	    	if (DEBUG_IMAGE)//这里应该就是把回环的结果画出来吧
	        {
	        	int gap = 10;
	        	cv::Mat gap_image(ROW, gap, CV_8UC1, cv::Scalar(255, 255, 255));
	            cv::Mat gray_img, loop_match_img;
	            cv::Mat old_img = old_kf->image;
				//这里将image、gap_image、old_img水平拼接起来成为gray_img
	            cv::hconcat(image, gap_image, gap_image);
	            cv::hconcat(gap_image, old_img, gray_img);
				 //灰度图gray_img转换成RGB图loop_match_img
	            cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
				//在图片loop_match_img上标注出匹配点和之间的连线
	            for(int i = 0; i< (int)matched_2d_cur.size(); i++)
	            {
	                cv::Point2f cur_pt = matched_2d_cur[i];
	                cv::circle(loop_match_img, cur_pt, 10, cv::Scalar(0, 0, 255),-1);
	            }
	            for(int i = 0; i< (int)matched_2d_old.size(); i++)
	            {
	                cv::Point2f old_pt = matched_2d_old[i];
	                old_pt.x += (COL + gap);
	                cv::circle(loop_match_img, old_pt, 10, cv::Scalar(0, 0, 255),-1);
	            }
	            for (int i = 0; i< (int)matched_2d_cur.size(); i++)
	            {
	                cv::Point2f old_pt = matched_2d_old[i];
	                old_pt.x += (COL + gap) ;
	                cv::line(loop_match_img, matched_2d_cur[i], old_pt, cv::Scalar(0, 255, 255), 1, 8, 0);//画黄色的线
	            }
				 //在loop_match_img下面垂直拼接一个notation，写上当前帧和先前帧的索引值和序列号
	            cv::Mat notation(50, COL + gap + COL, CV_8UC3, cv::Scalar(255, 255, 255));
	            // putText(notation, "current frame: " + to_string(index) + "  sequence: " + to_string(sequence), cv::Point2f(20, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 3);//原本是CV_FONT_HERSHEY_SIMPLEX
				putText(notation, "current frame: " + to_string(index), cv::Point2f(20, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 3);//原本是CV_FONT_HERSHEY_SIMPLEX

	            // putText(notation, "previous frame: " + to_string(old_kf->index) + "  sequence: " + to_string(old_kf->sequence), cv::Point2f(20 + COL + gap, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 3);
				putText(notation, "previous frame: " + to_string(old_kf->index) , cv::Point2f(20 + COL + gap, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 3);
	            cv::vconcat(notation, loop_match_img, loop_match_img);

	            /*
	            ostringstream path;
	            path <<  "/home/tony-ws1/raw_data/loop_image/"
	                    << index << "-"
	                    << old_kf->index << "-" << "3pnp_match.jpg";
	            cv::imwrite( path.str().c_str(), loop_match_img);
	            */
			     //若达到最小回环匹配点数（25），将loop_match_img的宽和高缩小一半并发布为pub_match_img
	            if ((int)matched_2d_cur.size() > MIN_LOOP_NUM)
	            {
					// printf("detecting the loop 1 !!!!!!!!!!!!!!!!! \n");
	            	/*
	            	cv::imshow("loop connection",loop_match_img);  
	            	cv::waitKey(10);  
	            	*/
	            	// cv::Mat thumbimage;
	            	// cv::resize(loop_match_img, thumbimage, cv::Size(loop_match_img.cols / 2, loop_match_img.rows / 2));//缩小了？
	    	    	// sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", thumbimage).toImageMsg();

					sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", loop_match_img).toImageMsg();//直接发布
	                msg->header.stamp = secondsToRosTime(time_stamp);
	    	    	pub_match_img->publish(*msg);//把回环的结果发布出来
					//保存图片
					ostringstream path;
					path <<  "/home/kwanwaipang/evio_result_visualization/loop_image/"
							<<"cur_frame:"<< index << "---"
							<<"pre_frame:"<< old_kf->index 
							<< "---" << "loop_image.jpg";
					cv::imwrite( path.str().c_str(), loop_match_img);
	            }
	        }
	    #endif
	}

// 4、如果在前面PNP检验后仍能达到最小回环匹配点数则进行先对位姿检验，通过则确定构成回环，
// 将回环帧索引和相对位姿存入loop_index、loop_info，并返回True。
	if ((int)matched_2d_cur.size() > MIN_LOOP_NUM)// 根据PNP的内点数进行判断，足够才认为回环可能成功
	{
		// printf("detecting the loop 2 !!!!!!!!!!!!!!!!! \n");
		//old:回环帧 cur：当前帧
	    relative_t = PnP_R_old.transpose() * (origin_vio_T - PnP_T_old);
	    relative_q = PnP_R_old.transpose() * origin_vio_R;
	    relative_yaw = Utility::normalizeAngle(Utility::R2ypr(origin_vio_R).x() - Utility::R2ypr(PnP_R_old).x());// 计算yaw角差
	    //printf("PNP relative\n");
	    //cout << "pnp relative_t " << relative_t.transpose() << endl;
	    //cout << "pnp relative_yaw " << relative_yaw << endl;
		//相对位姿检验
	    if (abs(relative_yaw) < 30.0 && relative_t.norm() < 20.0)// 合理范围之内才认为有效
		// if (abs(relative_yaw) < 60.0 && relative_t.norm() < 100.0)
	    {

	    	has_loop = true;// 至此才认为找到了回环
	    	loop_index = old_kf->index;// 确定回环帧的idx，以及两帧之间相对位姿（loop_info）
	    	loop_info << relative_t.x(), relative_t.y(), relative_t.z(),
	    	             relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
	    	             relative_yaw;//获得回环的信息
	    	if(FAST_RELOCALIZATION)//快速重定位功能
	    	{
			    sensor_msgs::msg::PointCloud msg_match_points;
			    msg_match_points.header.stamp = secondsToRosTime(time_stamp);
			    for (int i = 0; i < (int)matched_2d_old_norm.size(); i++)// 回环帧2d归一化坐标
			    {
		            geometry_msgs::msg::Point32 p;
		            p.x = matched_2d_old_norm[i].x;
		            p.y = matched_2d_old_norm[i].y;
		            p.z = matched_id[i];// 对应的VIO地图点的id
		            msg_match_points.points.push_back(p);
			    }
			    Eigen::Vector3d T = old_kf->T_w_i; // 回环帧的pose
			    Eigen::Matrix3d R = old_kf->R_w_i;
			    Quaterniond Q(R);
			    sensor_msgs::msg::ChannelFloat32 t_q_index;
			    t_q_index.values.push_back(T.x());
			    t_q_index.values.push_back(T.y());
			    t_q_index.values.push_back(T.z());
			    t_q_index.values.push_back(Q.w());
			    t_q_index.values.push_back(Q.x());
			    t_q_index.values.push_back(Q.y());
			    t_q_index.values.push_back(Q.z());
			    t_q_index.values.push_back(index);// 当前帧的索引
			    msg_match_points.channels.push_back(t_q_index);
			    pub_match_points->publish(msg_match_points);
	    	}
	        return true;
	    }
	}
	//printf("loop final use num %d %lf--------------- \n", (int)matched_2d_cur.size(), t_match.toc());
	return false;
}


int KeyFrame::HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b)
{
    BRIEF::bitset xor_of_bitset = a ^ b;
    int dis = xor_of_bitset.count();
    return dis;
}

void KeyFrame::getVioPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i)
{
    _T_w_i = vio_T_w_i;
    _R_w_i = vio_R_w_i;
}

void KeyFrame::getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i)
{
    _T_w_i = T_w_i;
    _R_w_i = R_w_i;
}

void KeyFrame::updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i)
{
    T_w_i = _T_w_i;
    R_w_i = _R_w_i;
}

void KeyFrame::updateVioPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i)
{
	vio_T_w_i = _T_w_i;
	vio_R_w_i = _R_w_i;
	T_w_i = vio_T_w_i;
	R_w_i = vio_R_w_i;
}

Eigen::Vector3d KeyFrame::getLoopRelativeT()
{
    return Eigen::Vector3d(loop_info(0), loop_info(1), loop_info(2));
}

Eigen::Quaterniond KeyFrame::getLoopRelativeQ()
{
    return Eigen::Quaterniond(loop_info(3), loop_info(4), loop_info(5), loop_info(6));
}

double KeyFrame::getLoopRelativeYaw()
{
    return loop_info(7);
}

void KeyFrame::updateLoop(Eigen::Matrix<double, 8, 1 > &_loop_info)
{
	if (abs(_loop_info(7)) < 30.0 && Vector3d(_loop_info(0), _loop_info(1), _loop_info(2)).norm() < 20.0)
	{
		//printf("update loop info\n");
		loop_info = _loop_info;
	}
}

BriefExtractor::BriefExtractor(const std::string &pattern_file)
{
  // The DVision::BRIEF extractor computes a random pattern by default when
  // the object is created.
  // We load the pattern that we used to build the vocabulary, to make
  // the descriptors compatible with the predefined vocabulary

  // loads the pattern
  cv::FileStorage fs(pattern_file.c_str(), cv::FileStorage::READ);
  if(!fs.isOpened()) throw string("Could not open file ") + pattern_file;

  vector<int> x1, y1, x2, y2;
  fs["x1"] >> x1;
  fs["x2"] >> x2;
  fs["y1"] >> y1;
  fs["y2"] >> y2;

  m_brief.importPairs(x1, y1, x2, y2);
}


