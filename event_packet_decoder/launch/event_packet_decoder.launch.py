# Decodes <camera_name>/events (event_camera_msgs/EventPacket) into
# <camera_name>/events_decoded (dvs_msgs/EventArray) for PL-EVIO.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        LaunchArg("camera_name", default_value="davis346",
                   description="namespace the DAVIS346 driver publishes under"),
        Node(
            package="event_packet_decoder",
            executable="event_packet_decoder_node",
            name="event_packet_decoder",
            namespace=LaunchConfig("camera_name"),
            output="screen",
        ),
    ])
