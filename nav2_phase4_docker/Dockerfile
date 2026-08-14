FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-opennav-docking \
    ros-jazzy-opennav-docking-bt \
    ros-jazzy-opennav-docking-core \
    ros-jazzy-apriltag-ros \
    ros-jazzy-apriltag-msgs \
    ros-jazzy-turtlebot3 \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-turtlebot3-msgs \
    ros-jazzy-robot-localization \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rviz-common \
    ros-jazzy-rviz-default-plugins \
    ros-jazzy-cv-bridge \
    ros-jazzy-tf2-geometry-msgs \
    python3-colcon-common-extensions \
    python3-pip \
    python3-opencv \
    python3-numpy \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir pupil-apriltags --break-system-packages || true

# Fix the Waffle Pi Gazebo model so LiDAR ignores its camera mount
RUN model=/opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf && \
    if [ -f "$model" ]; then sed -i 's/<min>0.120000<\/min>/<min>0.25<\/min>/' "$model"; fi

# Fix TurtleBot3 Gazebo Sim bridge to accept standard geometry_msgs/msg/Twist from Nav2
RUN for f in /opt/ros/jazzy/share/turtlebot3_gazebo/params/*_bridge.yaml; do \
      if [ -f "$f" ]; then sed -i 's/geometry_msgs\/msg\/TwistStamped/geometry_msgs\/msg\/Twist/' "$f"; fi; \
    done

ARG CACHEBUST=20260814_4
RUN echo "Building Phase 4 layers: $CACHEBUST"
# Copy configs, models, scripts, and startup script
COPY config /workspace_docker/config
COPY models /workspace_docker/models
COPY scripts /workspace_docker/scripts
COPY start_nav2_phase4.sh /start_nav2_phase4.sh
RUN chmod +x /start_nav2_phase4.sh /workspace_docker/scripts/*.py

WORKDIR /workspace_docker
ENTRYPOINT ["/start_nav2_phase4.sh"]
