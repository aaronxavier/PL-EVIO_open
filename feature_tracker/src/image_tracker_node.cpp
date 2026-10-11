#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>

#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::msg::Image::ConstSharedPtr> img_buf;

rclcpp::Node::SharedPtr g_node;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_img;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_restart;

FeatureTracker trackerData[NUM_OF_CAM];
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;

void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = rclcpp::Time(img_msg->header.stamp).seconds();
        last_image_time = rclcpp::Time(img_msg->header.stamp).seconds();
        return;
    }
    // detect unstable camera stream
    if (rclcpp::Time(img_msg->header.stamp).seconds() - last_image_time > 1.0 || rclcpp::Time(img_msg->header.stamp).seconds() < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;
        std_msgs::msg::Bool restart_flag;
        restart_flag.data = true;
        pub_restart->publish(restart_flag);
        return;
    }
    last_image_time = rclcpp::Time(img_msg->header.stamp).seconds();
    // frequency control
    if (round(1.0 * pub_count / (rclcpp::Time(img_msg->header.stamp).seconds() - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (rclcpp::Time(img_msg->header.stamp).seconds() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = rclcpp::Time(img_msg->header.stamp).seconds();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::msg::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat show_img = ptr->image;

    //加一个判据
    if(!( show_img.rows==ROW &&  show_img.cols==COL)){//对付image有时大小不一致的问题
        resize(show_img,show_img, cv::Size(COL, ROW));
    }


    TicToc t_r;
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ROS_DEBUG("processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)
            // trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), rclcpp::Time(img_msg->header.stamp).seconds());
            trackerData[i].readImage(show_img, rclcpp::Time(img_msg->header.stamp).seconds());
        else
        {
            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

#if SHOW_UNDISTORTION
        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
    }

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }

   if (PUB_THIS_FRAME)
   {
        pub_count++;
        auto feature_points = std::make_shared<sensor_msgs::msg::PointCloud>();
        sensor_msgs::msg::ChannelFloat32 id_of_point;
        sensor_msgs::msg::ChannelFloat32 u_of_point;
        sensor_msgs::msg::ChannelFloat32 v_of_point;
        sensor_msgs::msg::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::msg::ChannelFloat32 velocity_y_of_point;

        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "world";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::msg::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
        }
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
            pub_img->publish(*feature_points);

        if (SHOW_TRACK)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;
            if(!( stereo_img.rows==ROW &&  stereo_img.cols==COL)){//对付image有时大小不一致的问题
                resize(stereo_img,stereo_img, cv::Size(COL, ROW));
            }

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                    //draw speed line

                    Vector2d tmp_cur_un_pts (trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
                    Vector2d tmp_pts_velocity (trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
                    Vector3d tmp_prev_un_pts;
                    tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                    tmp_prev_un_pts.z() = 1;
                    Vector2d tmp_prev_uv;
                    trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                    cv::arrowedLine(tmp_img, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(0, 255, 0), 2 , 8, 0);

                    //char name[10];
                    //sprintf(name, "%d", trackerData[i].ids[j]);
                    //cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                }
            }
            //cv::imshow("vis", stereo_img);
            //cv::waitKey(5);
            pub_match->publish(*ptr->toImageMsg());
            // trackerData[0].save_image_tracking(ptr->image,rclcpp::Time(img_msg->header.stamp).seconds());//保存跟踪的图片
        }

    }
    // ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("image_feature_tracker");
    readParameters(g_node);
    ROS_WARN("into image feature detection and tracking");

    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

    // "~/..." replicates ROS1's `ros::NodeHandle n("~")` private-namespace topics
    // (e.g. "~/feature" -> /image_feature_tracker/feature).
    pub_img = g_node->create_publisher<sensor_msgs::msg::PointCloud>("~/feature", 1000);
    pub_match = g_node->create_publisher<sensor_msgs::msg::Image>("~/feature_img", 1000);
    pub_restart = g_node->create_publisher<std_msgs::msg::Bool>("~/restart", 1000);

    auto sub_img = g_node->create_subscription<sensor_msgs::msg::Image>(
        IMAGE_TOPIC, rclcpp::QoS(rclcpp::KeepLast(100)).best_effort(), img_callback);

    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */
    rclcpp::spin(g_node);
    rclcpp::shutdown();
    return 0;
}


// new points velocity is 0, pub or not?
// track cnt > 1 pub?
