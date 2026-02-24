#!/usr/bin/env bash
set -e

IMAGE_NAME=g1_ros2_dds_bridge:humble
ROS_DOMAIN_ID=0

docker run -it --rm \
  --name g1_ros2_dds_bridge \
  --net=host \
  --ipc=host \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
  ${IMAGE_NAME} \
  /opt/ws/src/g1_ros2_dds_bridge/ros_entrypoint.sh