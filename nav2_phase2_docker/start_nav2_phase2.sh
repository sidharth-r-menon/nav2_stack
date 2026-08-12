#!/usr/bin/env bash
# Do not use set -u: ROS setup scripts read optional environment variables.
set -eo pipefail

source /opt/ros/humble/setup.bash

export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:/opt/ros/humble/share/turtlebot3_gazebo/models"

startup_timeout="${NAV2_STARTUP_TIMEOUT:-180}"
NAV2_SHARE="$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup"

cleanup() {
  kill "${nav2_pid:-}" "${slam_pid:-}" "${gazebo_pid:-}" 2>/dev/null || true
  wait "${nav2_pid:-}" "${slam_pid:-}" "${gazebo_pid:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait_for_message() {
  local topic="$1"
  local description="$2"
  local deadline=$((SECONDS + startup_timeout))

  until timeout 4 ros2 topic echo "$topic" --once >/dev/null 2>&1; do
    if ! kill -0 "$gazebo_pid" 2>/dev/null; then
      echo "[nav2_phase2] ERROR: Gazebo exited while waiting for ${description}." >&2
      exit 1
    fi
    if (( SECONDS >= deadline )); then
      echo "[nav2_phase2] ERROR: No ${description} arrived within ${startup_timeout}s." >&2
      exit 1
    fi
    sleep 2
  done
}

echo "[nav2_phase2] Starting TurtleBot3 Waffle in Gazebo..."
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
gazebo_pid=$!

echo "[nav2_phase2] Waiting for Gazebo's API (up to ${startup_timeout}s)..."
deadline=$((SECONDS + startup_timeout))
until timeout 5 ros2 service call \
  /get_model_list gazebo_msgs/srv/GetModelList "{}" >/dev/null 2>&1; do
  if ! kill -0 "$gazebo_pid" 2>/dev/null; then
    echo "[nav2_phase2] ERROR: Gazebo exited before becoming ready." >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "[nav2_phase2] ERROR: Gazebo did not respond within ${startup_timeout}s." >&2
    exit 1
  fi
  sleep 2
done

# The stock TurtleBot3 spawner has a 30-second timeout. On Docker Desktop,
# Gazebo can need longer, so retry after its API has proved responsive.
if ! timeout 10 ros2 topic echo /odom --once >/dev/null 2>&1; then
  echo "[nav2_phase2] The stock robot spawn timed out; retrying..."
  ros2 run gazebo_ros spawn_entity.py \
    -entity waffle \
    -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf \
    -x -2.0 -y -0.5 -z 0.01
fi

echo "[nav2_phase2] Waiting for robot odometry and LiDAR..."
wait_for_message /odom "robot odometry"
wait_for_message /scan "laser scan"

echo "[nav2_phase2] Starting SLAM Toolbox in online asynchronous mapping mode..."
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=True \
  slam_params_file:=/etc/nav2_phase2/mapper_params_online_async.yaml &
slam_pid=$!

echo "[nav2_phase2] Waiting for the first SLAM map..."
wait_for_message /map "SLAM map"

echo "[nav2_phase2] Starting Nav2 without AMCL or map_server..."
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=True \
  autostart:=True &
nav2_pid=$!

sleep 7

echo "[nav2_phase2] Starting RViz..."
echo "[nav2_phase2] SLAM owns /map and map -> odom; do not use 2D Pose Estimate."
echo "[nav2_phase2] Explore with teleop first, then send Nav2 Goals in mapped free space."
rviz2 -d "${NAV2_SHARE}/rviz/nav2_default_view.rviz"
