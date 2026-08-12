FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Phase 2 replaces the known-map AMCL localization pipeline with SLAM Toolbox.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-tf2-tools \
    python3-colcon-common-extensions \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY start_nav2_phase2.sh /usr/local/bin/start_nav2_phase2.sh
COPY save_phase2_map.sh /usr/local/bin/save_phase2_map.sh
COPY config/mapper_params_online_async.yaml /etc/nav2_phase2/mapper_params_online_async.yaml
RUN chmod +x /usr/local/bin/start_nav2_phase2.sh /usr/local/bin/save_phase2_map.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
