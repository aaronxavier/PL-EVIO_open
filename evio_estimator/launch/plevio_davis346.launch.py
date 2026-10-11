# Full PL-EVIO pipeline against a DAVIS346 on the Jetson: libcaer_driver -> event_packet_decoder
# -> feature_tracker (event) + image_feature_tracker (APS) -> evio_estimator -> pose_graph.
# Headless by design (no "gnome-terminal -x" launch-prefix like the original ROS1 launch file --
# this runs fine over a plain SSH session, ros2 launch aggregates all node stdout on its own).
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch_ros.actions import Node


def generate_launch_description():
    config_file = LaunchConfig("config_file")
    vins_folder = LaunchConfig("vins_folder")
    camera_name = LaunchConfig("camera_name")
    config_params = [{"config_file": config_file, "vins_folder": vins_folder}]

    return LaunchDescription([
        LaunchArg("config_file", default_value="/home/px4nx/evio2_ws/config/ros2_davis346/davis346.yaml",
                   description="PL-EVIO YAML config (camera intrinsics, topics, tuning)"),
        LaunchArg("vins_folder", default_value="/home/px4nx/evio2_ws/",
                   description="base folder PL-EVIO resolves a few relative paths against"),
        LaunchArg("camera_name", default_value="davis346",
                   description="namespace the DAVIS346 driver publishes under"),

        # --- DAVIS346 driver + visualization (same as ~/davis346.launch.py on the Jetson) ---
        Node(package="libcaer_driver", executable="driver_node", name=camera_name, output="screen",
             parameters=[{
                 "device_type": "davis",
                 "dvs_enabled": True,
                 "aps_enabled": True,
                 "imu_accel_enabled": True,
                 "imu_gyro_enabled": True,
                 "auto_exposure_enabled": True,
                 "aps_frame_interval": 40000,  # us -> 25 fps
                 "statistics_print_interval": 5.0,
             }]),
        Node(package="event_camera_renderer", executable="renderer_node", namespace=camera_name,
             name="renderer", output="screen",
             parameters=[{"fps": 25.0, "display_type": "time_slice"}],
             remappings=[("~/events", "events")]),
        Node(package="web_video_server", executable="web_video_server", output="screen",
             parameters=[{"port": 8080}]),

        # --- decode EventPacket -> dvs_msgs/EventArray for the ported feature_tracker ---
        Node(package="event_packet_decoder", executable="event_packet_decoder_node",
             namespace=camera_name, name="event_packet_decoder", output="screen"),

        # --- PL-EVIO front end: event-based + APS-frame-based point/line feature tracking ---
        Node(package="feature_tracker", executable="feature_tracker", name="feature_tracker",
             output="screen", parameters=config_params),
        Node(package="feature_tracker", executable="image_feature_tracker", name="image_feature_tracker",
             output="screen", parameters=config_params),

        # --- PL-EVIO back end: sliding-window VIO ---
        Node(package="evio_estimator", executable="evio_estimator", name="evio_estimator",
             output="screen", parameters=config_params),

        # --- loop closure + global pose graph ---
        Node(package="pose_graph", executable="pose_graph", name="pose_graph", output="screen",
             parameters=[{
                 "config_file": config_file,
                 "visualization_shift_x": 0,
                 "visualization_shift_y": 0,
                 "skip_cnt": 0,
                 "skip_dis": 0.0,
             }]),
    ])
