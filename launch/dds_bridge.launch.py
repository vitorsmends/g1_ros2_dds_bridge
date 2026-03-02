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

                # IMPORTANTE:
                # Você não tem /clock publicando, então use_sim_time:=True deixa stamp=0 e quebra TF/message_filters.
                "use_sim_time": False,
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

                # Frame: prefira o frame real do sensor (ex: "livox") se você tiver TF.
                # Para teste, manter "world" é ok, mas não é o ideal para SLAM.
                "override_frame_id": "world",

                # IMPORTANTE:
                # Sem /clock, mantenha False para stamps não-zerados.
                "use_sim_time": False,

                # Logs/diagnóstico (do código que te mandei)
                "log_every_n": 1,
                "prefer_system_time_stamp": True,      # garante stamp != 0 mesmo se alguém ligar use_sim_time
                "force_little_endian": True,
                "recompute_steps": True,
                "sanitize_frame_id": True,
                "debug_first_point_xyz": True,
                "debug_sample_points": 8,
                "debug_check_both_endians": True,
                "debug_dump_data_prefix_bytes": 48,
                "debug_warn_if_all_invalid": True,
                "warn_if_stamp_zero": True,
            }],
        ),
    ])