#include "visualization.h"

using namespace Eigen;

rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_loop_image;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_img;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match, pub_match_two;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_time_surface;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_restart;
rclcpp::Publisher<dvs_msgs::msg::EventArray>::SharedPtr corner_pub;//把当前事件帧的所有feature发布出去
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match_two_line;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_feature_line;//线特征相关的

void registerPub(rclcpp::Node::SharedPtr n)
{
    // "~/..." replicates ROS1's `ros::NodeHandle n("~")` private-namespace topics
    // (e.g. "~/feature" -> /feature_tracker/feature), which downstream evio_estimator
    // and pose_graph configs expect.
    pub_img = n->create_publisher<sensor_msgs::msg::PointCloud>("~/feature", 1000);//发布feature  (实际发出去的是 /feature_tracker/feature)
    pub_match = n->create_publisher<sensor_msgs::msg::Image>("~/feature_img", 1000);//发布feature tracking的image
    pub_match_two = n->create_publisher<sensor_msgs::msg::Image>("~/feature_img_two", 1000);
    pub_time_surface = n->create_publisher<sensor_msgs::msg::Image>("~/timesurface_map", 1000);//发布timesurface
    pub_restart = n->create_publisher<std_msgs::msg::Bool>("~/restart", 1000);

    corner_pub = n->create_publisher<dvs_msgs::msg::EventArray>("~/event_feature_loop", 1000);//把当前事件信息的所有角点发布到pose graph用于回环检测

    pub_loop_image = n->create_publisher<sensor_msgs::msg::Image>("~/loop_image", 1000);//发布timesurface

    pub_match_two_line = n->create_publisher<sensor_msgs::msg::Image>("~/linefeature_img_two", 1000);//发布两张图片叠在一起的线特征matching的结果
    pub_feature_line = n->create_publisher<sensor_msgs::msg::PointCloud>("~/linefeature", 1000);//发布线feature(当前帧的特征)
}

/**
 * @brief 发布用于回环检测的
 */
void pubLoopImage(const cv::Mat &timesurface, const double t)
{//发布回环用的图像
    std_msgs::msg::Header header;
    header.frame_id = "world";
    header.stamp = secondsToRosTime(t);

    sensor_msgs::msg::Image::SharedPtr TimeSurfaceImg = cv_bridge::CvImage(header, "mono8", timesurface).toImageMsg();
    pub_loop_image->publish(*TimeSurfaceImg);
}
