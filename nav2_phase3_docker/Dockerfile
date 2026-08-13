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
    git \
    && rm -rf /var/lib/apt/lists/*

# The stock Waffle Pi camera is RGB-only. Gazebo's camera plugin also supports
# aligned depth and PointCloud2 when the sensor type is "depth". Keep the
# official robot geometry/TF tree and make only that simulation sensor change.
# Add the last two sed commands to your existing block:
# RUN model=/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf && \
#     sed -i 's/<sensor name="camera" type="camera">/<sensor name="camera" type="depth">/' "$model" && \
#     sed -i 's/<width>640<\/width>/<width>320<\/width>/; s/<height>480<\/height>/<height>240<\/height>/' "$model" && \
#     sed -i '/<\/image>/a\            <depth_camera><\/depth_camera>' "$model" && \
#     sed -i 's/<update_rate>30.0<\/update_rate>/<update_rate>10.0<\/update_rate>/' "$model" && \
#     sed -i 's/<min>0.120000<\/min>/<min>0.25<\/min>/' "$model"

# Fix the Waffle Pi Gazebo model so the LiDAR ignores its own camera mount
RUN model=/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf && \
    sed -i 's/<min>0.120000<\/min>/<min>0.25<\/min>/' "$model"

# Clone frontier_exploration_ros2 (v1.6.1, Humble-compatible) into the workspace src
WORKDIR /opt/phase3_ws/src
RUN git clone --depth 1 https://github.com/mertgulerx/frontier_exploration_ros2.git

# Clone the AWS RoboMaker Small House World (ros2 branch) for the multi-room
# house layout. This package is WORLD-ONLY: its small_house.launch.py starts
# gzserver/gzclient with the house world loaded, but does NOT spawn any
# robot - unlike turtlebot3_gazebo's turtlebot3_house.launch.py, which
# bundles world + robot + spawn together. The robot spawn step is handled
# separately in start_nav2_phase3.sh via spawn_entity.py, same pattern the
# script already uses as its retry fallback for turtlebot3_house.
RUN git clone --depth 1 --branch ros2 \
    https://github.com/aws-robotics/aws-robomaker-small-house-world.git \
    /opt/phase3_ws/src/aws-robomaker-small-house-world

# Copy our perception / approach package
COPY ros2_ws/src/nav2_phase3 /opt/phase3_ws/src/nav2_phase3

# Build: frontier_exploration_ros2 (C++ with MRTSP) + nav2_phase3 (Python perception)
#        + aws_robomaker_small_house_world (Gazebo world package)
WORKDIR /opt/phase3_ws
RUN source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install \
        --packages-select frontier_exploration_ros2 nav2_phase3 \
        aws_robomaker_small_house_world

# NOTE: package name inside the repo is aws_robomaker_small_house_world
# (underscored) even though the git repo name uses hyphens - this is the
# name colcon needs for --packages-select and get_package_share_directory().

# Copy runtime configuration files
COPY config/mapper_params_online_async.yaml /etc/nav2_phase3/mapper_params_online_async.yaml
COPY config/frontier_exploration.yaml /etc/nav2_phase3/frontier_exploration.yaml
COPY config/nav2_params.yaml /etc/nav2_phase3/nav2_params.yaml
COPY models/red_ball.sdf /etc/nav2_phase3/red_ball.sdf
COPY start_nav2_phase3.sh /usr/local/bin/start_nav2_phase3.sh
COPY save_phase3_map.sh /usr/local/bin/save_phase3_map.sh
RUN chmod +x /usr/local/bin/start_nav2_phase3.sh /usr/local/bin/save_phase3_map.sh

WORKDIR /ws
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]