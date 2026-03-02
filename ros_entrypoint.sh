#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /opt/ws/install/setup.bash

exec ros2 launch g1_ros2_dds_bridge dds_bridge.launch.py