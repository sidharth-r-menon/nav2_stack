FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-simple-commander \
    ros-humble-slam-toolbox \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-cv-bridge \
    ros-humble-tf2-geometry-msgs \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-tf2-tools \
    python3-colcon-common-extensions \
    python3-opencv \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# The stock Waffle Pi camera is RGB-only. Gazebo's camera plugin also supports
# aligned depth and PointCloud2 when the sensor type is "depth". Keep the
# official robot geometry/TF tree and make only that simulation sensor change.
RUN model=/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf && \
    sed -i 's/<sensor name="camera" type="camera">/<sensor name="camera" type="depth">/' "$model" && \
    sed -i 's/<width>640<\/width>/<width>320<\/width>/; s/<height>480<\/height>/<height>240<\/height>/' "$model" && \
    sed -i '/<\/image>/a\            <depth_camera></depth_camera>' "$model"

WORKDIR /opt/phase3_ws
COPY ros2_ws/src ./src
# m-explore-ros2 also contains an optional multi-robot map-merging package.
# Phase 3 deliberately builds only its established single-robot explore_lite
# implementation plus our perception handoff package.
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install \
    --packages-select explore_lite_msgs explore_lite nav2_phase3

COPY config/mapper_params_online_async.yaml /etc/nav2_phase3/mapper_params_online_async.yaml
COPY config/explore_lite.yaml /etc/nav2_phase3/explore_lite.yaml
COPY models/red_ball.sdf /etc/nav2_phase3/red_ball.sdf
COPY start_nav2_phase3.sh /usr/local/bin/start_nav2_phase3.sh
COPY save_phase3_map.sh /usr/local/bin/save_phase3_map.sh
RUN chmod +x /usr/local/bin/start_nav2_phase3.sh /usr/local/bin/save_phase3_map.sh

WORKDIR /ws
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
