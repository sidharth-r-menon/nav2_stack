#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/phase3_ws/install/setup.bash
name="${1:-phase3_map}"
mkdir -p /ws/maps
ros2 run nav2_map_server map_saver_cli -f "/ws/maps/${name}"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/ws/maps/${name}'}"
