FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# ROS 2 Humble uses Gazebo Classic 11. This image deliberately installs only
# the standard TurtleBot3 + Nav2 demo stack; no custom robot is introduced in
# Phase 1.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rmw-fastrtps-cpp \
    python3-colcon-common-extensions \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY start_nav2_phase1.sh /usr/local/bin/start_nav2_phase1.sh
RUN chmod +x /usr/local/bin/start_nav2_phase1.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
