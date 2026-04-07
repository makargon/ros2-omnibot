ARG ROS_BASE_IMAGE=arm64v8/ros:jazzy-ros-base
FROM ${ROS_BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    ros-jazzy-ament-package \
    python3-spidev \
    python3-smbus2 \
    i2c-tools \
    python3-libgpiod \
    libgpiod-dev \
    libgpiod2 \
    gpiod \
    ros-jazzy-geometry-msgs \
    ros-jazzy-launch-ros \
    ros-jazzy-cv-bridge \
    ros-jazzy-rplidar-ros \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-map-server \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-v4l2-camera \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-camera-info-manager \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-tf2-ros \
    libopencv-dev \
    libopencv-contrib-dev \
    libopencv-core-dev \
    libopencv-calib3d-dev \
    python3-opencv \
    python3-yaml \
    build-essential \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

ENV PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:${PYTHONPATH}
WORKDIR /workspaces/omnibot/ros2_ws

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
