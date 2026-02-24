from setuptools import setup

package_name = "g1_ros2_dds_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/dds_bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vitor Mendes",
    maintainer_email="vitormendesrb@gmail.com",
    description="DDS to ROS 2 bridge for Unitree G1 (LowState odom + Livox PointCloud2)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lowstate_to_dog_odom = g1_ros2_dds_bridge.lowstate_to_dog_odom:main",
            "livox_to_pointcloud2 = g1_ros2_dds_bridge.livox_to_pointcloud2:main",
        ],
    },
)