#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /opt/ws/install/setup.bash

exec ros2 run g1_ros2_dds_bridge bridge_main --ros-args --params-file /opt/ws/src/g1_ros2_dds_bridge/config/bridges.yaml