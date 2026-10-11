#include "parameters.h"

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

std::vector<std::string> CAM_NAMES;//相机的参数
Eigen::Matrix3d Rrl; //从左相机到右相机的变换
Eigen::Vector3d Trl;  //从左相机到右相机的变换
Eigen::Matrix3d Rlr; //从右相机到左相机的变换
Eigen::Vector3d Tlr;//从右相机到左相机的变换

int temp_T=0;//用于查看是否需要转置的
int use_stereo=0;//是否采用双目

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;//把结果输出
std::string IMU_TOPIC;
double ROW, COL;
double TD, TR;//TR是用于处理由于卷帘效应带来的延时

// 双目的一些参数
double nG = 1.;
bool g_use_sampson_model = false;
bool g_use_stereo_correction = true; // false; 
bool g_opt_verbose = false; 

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

void readParameters(rclcpp::Node::SharedPtr n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["imu_topic"] >> IMU_TOPIC;

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    std::string OUTPUT_PATH;
    fsSettings["output_path"] >> OUTPUT_PATH;
    VINS_RESULT_PATH = OUTPUT_PATH + "/evio_result_no_loop.csv";//把结果输出
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;

    // create folder if not exists
    FileSystemHelper::createDirectoryIfNotExists(OUTPUT_PATH.c_str());//如果不存在就重新构建

    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.setf(std::ios::fixed, std::ios::floatfield);
    fout.precision(0);
    fout << "time" << ",";
    fout.precision(5);
    fout << "x" << ","
            << "y" << ","
            << "z" << ","
            << "qw" << ","
            << "qx" << ","
            << "qy" << ","
            << "qz" << ","
            << "vx" << ","
            << "vy" << ","
            << "vz" << "," << std::endl;
    fout.close();


    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %f COL: %f ", ROW, COL);


    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)//完全不知道camera与imu的外参
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";

    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)//对于camera与imu的外参有初始估计值
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)//固定camera与imu的外参
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();

        //此时Eigen::Matrix3d eigen_R;与Eigen::Vector3d eigen_T为需要的东西
        fsSettings["T_camera_imu"]>>temp_T;//是否输入的为imu到camera
        Eigen::Matrix3d R_imu_camera;
        Eigen::Vector3d T_imu_camera;
        if(temp_T){//需要执行转置
            ROS_INFO("give an T to the extrinsic matrix !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            R_imu_camera=eigen_R.inverse();//获取IMU到camera的变换
              // TIC[0]=-RIC[0]*TIC[0];
            T_imu_camera=-R_imu_camera*eigen_T;
        }
        else{
            R_imu_camera=eigen_R;//R矩阵
            T_imu_camera=eigen_T;//T矩阵
        }

        RIC.push_back(R_imu_camera);
        TIC.push_back(T_imu_camera);

        // RIC.push_back(eigen_R);//R矩阵
        // TIC.push_back(eigen_T);//T矩阵
        ROS_INFO_STREAM("Extrinsic_R : " << std::endl << RIC[0]);
        ROS_INFO_STREAM("Extrinsic_T : " << std::endl << TIC[0].transpose());

    } 

    //如果是双目的情况
    use_stereo=fsSettings["use_stereo"];
    if(use_stereo)
    {
        n->declare_parameter<double>("gravity_norm", nG);
        n->get_parameter("gravity_norm", nG);
        n->declare_parameter<bool>("use_sampson_model", g_use_sampson_model);
        n->get_parameter("use_sampson_model", g_use_sampson_model);
        n->declare_parameter<bool>("use_stereo_correction", g_use_stereo_correction);
        n->get_parameter("use_stereo_correction", g_use_stereo_correction);
        n->declare_parameter<bool>("opt_verbose", g_opt_verbose);
        n->get_parameter("opt_verbose", g_opt_verbose);
        ROS_INFO("parameters.cpp: gravity_norm: %lf", nG);
        std::cout <<"parameters.cpp: "<< (g_use_sampson_model?"Yes use sampson model":"Not use sampson model")<<std::endl;
        std::cout <<"parameters.cpp: "<< (g_use_stereo_correction?"Yes use geometric correction":"Not use geometric correction")<<std::endl;

        // 获取camera1到imu的内参
        cv::Mat cv_TT;
        fsSettings["body_T_cam1"] >> cv_TT;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_TT, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
        ROS_INFO_STREAM("Extrinsic_R2 : " << std::endl << RIC[1]);
        ROS_INFO_STREAM("Extrinsic_T2 : " << std::endl << TIC[1].transpose());

        // transformation between stereo cams 
        {
            cv::Mat cv_R, cv_T; 
            fsSettings["Rrl"] >> cv_R; 
            fsSettings["Trl"] >> cv_T; 
            cv::cv2eigen(cv_R, Rrl); //获取从左到右的
            cv::cv2eigen(cv_T, Trl); //获取从左到右的
            Eigen::Quaterniond qq(Rrl); 
            Rrl = qq.normalized();
            Rlr = Rrl.transpose(); 
            Tlr = - Rlr * Trl;  
            ROS_INFO_STREAM("Rrl: " << std::endl << Rrl); 
            ROS_INFO_STREAM("Trl: " << std::endl << Trl.transpose());  
        }

        //获取相机的参数
        {
            std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");

            std::string cam0Calibfile, cam1Calibfile; 
            fsSettings["cam0_calib"] >> cam0Calibfile; 
            fsSettings["cam1_calib"] >> cam1Calibfile; 

            std::string cam0Path = VINS_FOLDER_PATH + "/" + cam0Calibfile; 
            std::string cam1Path = VINS_FOLDER_PATH + "/" + cam1Calibfile; 

            ROS_DEBUG("cam0Path: %s", cam0Path.c_str()); 
            ROS_DEBUG("cam1Path: %s", cam1Path.c_str());
            CAM_NAMES.push_back(cam0Path); 
            CAM_NAMES.push_back(cam1Path); 
        }
        
    }
    else{

    }


    INIT_DEPTH = -1.0; //10.0; //5.0;///为什么设置为5.0？是否可以改为-1.0？？？？？？？？？？？？？？？？？？？
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];
        ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;
    }
    
    fsSettings.release();
}
