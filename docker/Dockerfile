# ROS 2 Jazzy + Nav2 開発環境（Ubuntu 24.04）。リポジトリは /ws/src/social_nav にマウントする
FROM ros:jazzy-ros-base
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git python3-colcon-common-extensions python3-rosdep \
      ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-collision-monitor \
      ros-jazzy-behaviortree-cpp \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc
CMD ["bash"]
