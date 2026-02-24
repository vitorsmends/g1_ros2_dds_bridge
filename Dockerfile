FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir unitree_sdk2py

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

CMD ["/opt/ws/src/g1_ros2_dds_bridge/ros_entrypoint.sh"]