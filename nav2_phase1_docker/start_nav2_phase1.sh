#!/usr/bin/env bash
# Do not use set -u: ROS setup scripts read optional environment variables.
set -eo pipefail

source /opt/ros/humble/setup.bash

export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:/opt/ros/humble/share/turtlebot3_gazebo/models"

# Gazebo can take well over the stock spawn_entity 30-second timeout to become
# ready when it is rendering through Docker Desktop and a Windows X server.
# Keep this configurable for slower hosts.
startup_timeout="${NAV2_STARTUP_TIMEOUT:-180}"

NAV2_SHARE="$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup"

cleanup() {
  kill "${nav2_pid:-}" "${gazebo_pid:-}" 2>/dev/null || true
  wait "${nav2_pid:-}" "${gazebo_pid:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[nav2_phase1] Starting TurtleBot3 Waffle in Gazebo..."
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
gazebo_pid=$!

echo "[nav2_phase1] Waiting for Gazebo's API (up to ${startup_timeout}s)..."
deadline=$((SECONDS + startup_timeout))
# Calling a service is intentional: after a quick container restart, DDS may
# briefly report the previous container's stale service endpoint.
until timeout 5 ros2 service call \
  /get_model_list gazebo_msgs/srv/GetModelList "{}" >/dev/null 2>&1; do
  if ! kill -0 "$gazebo_pid" 2>/dev/null; then
    echo "[nav2_phase1] ERROR: Gazebo exited before becoming ready." >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "[nav2_phase1] ERROR: Gazebo did not respond within ${startup_timeout}s." >&2
    exit 1
  fi
  sleep 2
done

# turtlebot3_world.launch.py starts its spawner immediately, and that spawner
# gives up after 30 seconds. Give it a chance to finish, then retry only when
# it did not produce odometry. This avoids duplicate robots on faster hosts.
if ! timeout 10 ros2 topic echo /odom --once >/dev/null 2>&1; then
  echo "[nav2_phase1] The stock robot spawn timed out; retrying now that Gazebo is ready..."
  ros2 run gazebo_ros spawn_entity.py \
    -entity waffle \
    -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf \
    -x -2.0 -y -0.5 -z 0.01
fi

echo "[nav2_phase1] Waiting for robot odometry before starting Nav2..."
deadline=$((SECONDS + startup_timeout))
until timeout 3 ros2 topic echo /odom --once >/dev/null 2>&1; do
  if ! kill -0 "$gazebo_pid" 2>/dev/null; then
    echo "[nav2_phase1] ERROR: Gazebo exited while waiting for robot odometry." >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "[nav2_phase1] ERROR: The robot did not publish /odom within ${startup_timeout}s." >&2
    exit 1
  fi
  sleep 2
done

echo "[nav2_phase1] Starting AMCL + Nav2..."
ros2 launch nav2_bringup bringup_launch.py \
  map:="${NAV2_SHARE}/maps/turtlebot3_world.yaml" \
  use_sim_time:=True \
  autostart:=True &
nav2_pid=$!

sleep 5

echo "[nav2_phase1] Starting RViz..."
echo "[nav2_phase1] In RViz: use 2D Pose Estimate first, then Nav2 Goal."
rviz2 -d "${NAV2_SHARE}/rviz/nav2_default_view.rviz"
