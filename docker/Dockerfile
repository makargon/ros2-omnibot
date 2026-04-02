ARG ROS_BASE_IMAGE=arm64v8/ros:humble-ros-base
FROM ${ROS_BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-spidev \
    python3-smbus2 \
    i2c-tools \
    python3-libgpiod \
    libgpiod-dev \
    libgpiod2 \
    gpiod \
    ros-humble-geometry-msgs \
    ros-humble-launch-ros \
    ros-humble-rplidar-ros \
    ros-humble-slam-toolbox \
    ros-humble-nav2-map-server \
    ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspaces/omnibot/ros2_ws

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
