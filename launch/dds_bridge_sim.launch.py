from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="g1_ros2_dds_bridge",
            executable="lowstate_to_dog_odom",
            name="lowstate_to_dog_odom",
            output="screen",
            parameters=[{
                "dds_domain_id": 0,
                "dds_topic": "rt/lowstate",
                "ros_topic": "/dog_odom",
                "odom_frame": "odom",
                "base_frame": "base_link",
            }],
        ),
        Node(
            package="g1_ros2_dds_bridge",
            executable="livox_to_pointcloud2",
            name="livox_to_pointcloud2",
            output="screen",
            parameters=[{
                "dds_domain_id": 0,
                "dds_topic": "rt/utlidar/cloud_livox_mid360",
                "ros_topic": "/livox/lidar",
                "override_frame_id": "",
            }],
        ),
    ])