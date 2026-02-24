FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/ws/src
COPY . ./g1_ros2_dds_bridge
RUN chmod +x /opt/ws/src/g1_ros2_dds_bridge/ros_entrypoint.sh

WORKDIR /opt/ws
RUN (rosdep init 2>/dev/null || true) && \
    rosdep update && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install && \
    rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /opt/ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]