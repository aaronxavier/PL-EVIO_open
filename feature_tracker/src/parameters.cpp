#include "parameters.h"
#include <opencv2/core/eigen.hpp>

std::string IMAGE_TOPIC;
std::string IMAGE1_TOPIC;//双目时，左相机的topic
std::string IMAGE2_TOPIC; //双目时，右相机的topic
std::string EVENT_TOPIC;//事件的话题
std::string IMU_TOPIC;
std::vector<std::string> CAM_NAMES;//导入camera 参数
std::string FISHEYE_MASK;
int use_stereo=0;//是否采用双目
int STEREO;//是否采用双目
Eigen::Matrix3d Eeesntial_matrix;
int MAX_CNT;
int MIN_DIST;//事件特征的距离
int MIN_DIST_IMG;//图像特征的距离
int WINDOW_SIZE;
int FREQ;//发布事件前端的频率
int FREQ_IMG;//发布图像前端的频率
double F_THRESHOLD;

double TS_LK_THRESHOLD;//time surface阈值
int para_ignore_polarity;//是否使用极性;
double para_decay_ms;//time surface的时延
double para_decay_loop_ms;//用于回环的时延迟
int para_median_blur_kernel_size;//模糊处理的核的大小
double para_feature_filter_threshold;//处理的间隔时间

int SHOW_TRACK;
int FLOW_BACK;//光流验证
int STEREO_TRACK;
int EQUALIZE;
int ROW;//图片的height
int COL;//图片的width
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;
int Num_of_thread;//处理大量event数组的时候采用多少个线程

//线特征相关的
double MIN_length;//提取的线特征最小的长度
double Scale_image;//The scale of the image that will be used to find the lines. Range (0..1].
double Maximun_lines;//最大数目的线
int Do_motion_correction=0;//是否去除event的运动畸变

template <typename T>
T readParam(rclcpp::Node::SharedPtr n, std::string name)
{
    T ans;
    n->declare_parameter<T>(name, T());
    if (n->get_parameter(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        rclcpp::shutdown();
    }
    return ans;
}

namespace {
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
    {
            Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
            ans << typename Derived::Scalar(0), -q(2), q(1),
                q(2), typename Derived::Scalar(0), -q(0),
                -q(1), q(0), typename Derived::Scalar(0);
            return ans;
    }
}

void readParameters(rclcpp::Node::SharedPtr n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");

    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["image1_topic"] >> IMAGE1_TOPIC;
    fsSettings["image2_topic"] >> IMAGE2_TOPIC; 
    fsSettings["event_topic"] >> EVENT_TOPIC;//事件话题的读入
    fsSettings["imu_topic"] >> IMU_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    MIN_DIST_IMG = fsSettings["min_dist_img"];////图像特征的距离
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];//发布给后端的频率
    // FREQ_IMG=fsSettings["freq_img"];//发布图像前端的频率
    F_THRESHOLD = fsSettings["F_threshold"];
    Num_of_thread=fsSettings["Num_of_thread"];

    TS_LK_THRESHOLD=fsSettings["TS_LK_threshold"];//获取time surface的阈值
    para_ignore_polarity= fsSettings["ignore_polarity"];//true;
    para_decay_ms= fsSettings["decay_ms"];//60;//延迟 30
    para_median_blur_kernel_size= fsSettings["median_blur_kernel_size"];//1;
    para_feature_filter_threshold=fsSettings["feature_filter_threshold"];

     //线特征
    MIN_length=fsSettings["MIN_length"];
    Scale_image=fsSettings["Scale_image"];
    Maximun_lines=fsSettings["Maximun_lines"];

    //是否给event做运动补偿
    Do_motion_correction=fsSettings["Do_motion_correction"];

    // para_decay_loop_ms=fsSettings["ts_decay_loop_ms"];

    SHOW_TRACK = fsSettings["show_track"];
    FLOW_BACK = fsSettings["flow_back"];//光流验证
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1)
        FISHEYE_MASK = VINS_FOLDER_PATH + "config/fisheye_mask.jpg";
    // CAM_NAMES.push_back(config_file);

     // transformation between stereo cams 
    use_stereo=fsSettings["use_stereo"];
    if(use_stereo)
    {
        int STEREO = 1;

        std::string cam0Calibfile, cam1Calibfile; 
        fsSettings["cam0_calib"] >> cam0Calibfile; 
        fsSettings["cam1_calib"] >> cam1Calibfile; 
        std::string cam0Path = VINS_FOLDER_PATH + "/" + cam0Calibfile; 
        std::string cam1Path = VINS_FOLDER_PATH + "/" + cam1Calibfile; 
        CAM_NAMES.push_back(cam0Path); 
        CAM_NAMES.push_back(cam1Path); 

        //下面获取两个相机之间的变换
        Eigen::Matrix3d Rlr;
        Eigen::Vector3d Tlr; 
        Eigen::Matrix3d Rrl;     // Trl 
        Eigen::Vector3d Trl; 
        cv::Mat cv_R, cv_T; 
        fsSettings["Rrl"] >> cv_R; 
        fsSettings["Trl"] >> cv_T; 
        cv::cv2eigen(cv_R, Rrl); 
        cv::cv2eigen(cv_T, Trl); 
        Eigen::Quaterniond qq(Rrl); 
        Rrl = qq.normalized();
        Rlr = Rrl.transpose(); 
        Tlr = - Rlr * Trl;  
        ROS_INFO_STREAM("Rrl: " << std::endl << Rrl); 
        ROS_INFO_STREAM("Trl: " << std::endl << Trl.transpose());  

        Eeesntial_matrix.setZero();
        // const Eigen::Vector3d t_0_1 = T_0_1.translation();
        // const Eigen::Matrix3d R_0_1 = T_0_1.rotationMatrix();
        // E.topLeftCorner<3, 3>() = Sophus::SO3d::hat(t_0_1.normalized()) * R_0_1;
        Eeesntial_matrix = skewSymmetric(Trl) * Rrl;
    }
    else
    {
        CAM_NAMES.push_back(config_file);//当为单目的时候，只有一个config文件，直接读入即可
        int STEREO = 0;
    }


    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)//若设置为0，那么就置为100
        FREQ = 100;

    fsSettings.release();


}
