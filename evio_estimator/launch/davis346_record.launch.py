# Record-only / front-end-only PL-EVIO stack for the DAVIS346 on the Jetson:
# libcaer_driver -> event_packet_decoder -> feature_tracker (event) + image_feature_tracker (APS).
# Deliberately OMITS evio_estimator and pose_graph, whose measurement buffers grow without
# bound when VIO cannot initialize (e.g. starved image-feature stream in low light) and will
# eventually OOM the Jetson. Use this for bag collection and for exercising the front end;
# use plevio_davis346.launch.py only when you actually expect VIO to initialize (good
# lighting + motion) and want the full back end running.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch_ros.actions import Node


def generate_launch_description():
    config_file = LaunchConfig("config_file")
    vins_folder = LaunchConfig("vins_folder")
    camera_name = LaunchConfig("camera_name")
    # Auto-exposure MUST be set here at driver launch, not via runtime `ros2 param set`:
    # toggling auto_exposure_enabled live does NOT stop the driver's AE control loop, which
    # keeps driving aps_exposure back to its 1s ceiling. To force a fixed frame rate, launch
    # with auto_exposure:=false and aps_exposure:=<us> (100000 us -> ~10fps, 200000 -> ~5fps).
    auto_exposure = LaunchConfig("auto_exposure")
    aps_exposure = LaunchConfig("aps_exposure")
    config_params = [{"config_file": config_file, "vins_folder": vins_folder}]

    return LaunchDescription([
        LaunchArg("config_file", default_value="/home/px4nx/evio2_ws/config/ros2_davis346/davis346.yaml",
                   description="PL-EVIO YAML config (camera intrinsics, topics, tuning)"),
        LaunchArg("vins_folder", default_value="/home/px4nx/evio2_ws/",
                   description="base folder PL-EVIO resolves a few relative paths against"),
        LaunchArg("camera_name", default_value="davis346",
                   description="namespace the DAVIS346 driver publishes under"),
        LaunchArg("auto_exposure", default_value="false",
                   description="DAVIS346 APS auto-exposure; set false to use a fixed aps_exposure"),
        LaunchArg("aps_exposure", default_value="32145",
                   description="fixed APS exposure in microseconds (only used when auto_exposure:=false)"),

        # --- DAVIS346 driver + visualization ---
        Node(package="libcaer_driver", executable="driver_node", name=camera_name, output="screen",
             parameters=[{
                 "device_type": "davis",
                 "dvs_enabled": True,
                 "aps_enabled": True,
                 "imu_accel_enabled": True,
                 "imu_gyro_enabled": True,
                 "auto_exposure_enabled": auto_exposure,
                 "aps_exposure": aps_exposure,
                 "aps_frame_interval": 40000,  # us -> 25 fps target (actual rate is exposure-limited)
                 "statistics_print_interval": 5.0,
             }]),
        Node(package="event_camera_renderer", executable="renderer_node", namespace=camera_name,
             name="renderer", output="screen",
             parameters=[{"fps": 25.0, "display_type": "time_slice"}],
             remappings=[("~/events", "events")]),
        Node(package="web_video_server", executable="web_video_server", output="screen",
             parameters=[{"port": 8080}]),

        # --- decode EventPacket -> dvs_msgs/EventArray ---
        Node(package="event_packet_decoder", executable="event_packet_decoder_node",
             namespace=camera_name, name="event_packet_decoder", output="screen"),

        # --- front end only: event + APS feature tracking (no VIO back end) ---
        Node(package="feature_tracker", executable="feature_tracker", name="feature_tracker",
             output="screen", parameters=config_params),
        Node(package="feature_tracker", executable="image_feature_tracker", name="image_feature_tracker",
             output="screen", parameters=config_params),
    ])
