#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash

map_name="${1:-phase2_map}"
output_dir="/ws/maps"
output_prefix="${output_dir}/${map_name}"

mkdir -p "$output_dir"

echo "[nav2_phase2] Saving occupancy map to ${output_prefix}.yaml and .pgm..."
ros2 run nav2_map_server map_saver_cli -f "$output_prefix"

echo "[nav2_phase2] Saving SLAM pose graph to ${output_prefix}.posegraph and .data..."
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '${output_prefix}'}"

echo "[nav2_phase2] Save request complete. Files are under ${output_dir}."
