#!/usr/bin/env bash
set -e

IMAGE_NAME=g1_ros2_dds_bridge:humble
docker build -t ${IMAGE_NAME} .