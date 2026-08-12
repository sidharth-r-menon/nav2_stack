#!/usr/bin/env bash
# Phase 3: unknown indoor world -> SLAM map -> frontier goals -> RGB-D ball approach.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/phase3_ws/install/setup.bash

export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:/opt/ros/humble/share/turtlebot3_gazebo/models"
startup_timeout="${NAV2_STARTUP_TIMEOUT:-300}"
robot_x="${ROBOT_X:--2.0}"; robot_y="${ROBOT_Y:--0.5}"
ball_x="${BALL_X:-1.0}"; ball_y="${BALL_Y:-2.0}"
NAV2_SHARE="$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup"

cleanup() {
  kill "${phase3_pid:-}" "${nav2_pid:-}" "${slam_pid:-}" "${gazebo_pid:-}" 2>/dev/null || true
  wait "${phase3_pid:-}" "${nav2_pid:-}" "${slam_pid:-}" "${gazebo_pid:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait_for() {
  local topic="$1" description="$2" deadline=$((SECONDS + startup_timeout))
  until timeout 4 ros2 topic echo "$topic" --once >/dev/null 2>&1; do
    if ! kill -0 "$gazebo_pid" 2>/dev/null; then echo "[phase3] Gazebo exited while waiting for $description" >&2; exit 1; fi
    if (( SECONDS >= deadline )); then echo "[phase3] Timed out waiting for $description" >&2; exit 1; fi
    sleep 2
  done
}

echo "[phase3] Starting the official TurtleBot3 multi-room house world..."
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py x_pose:="$robot_x" y_pose:="$robot_y" &
gazebo_pid=$!

echo "[phase3] Waiting for Gazebo API (Docker Desktop can take a few minutes)..."
deadline=$((SECONDS + startup_timeout))
until timeout 5 ros2 service call /get_model_list gazebo_msgs/srv/GetModelList "{}" >/dev/null 2>&1; do
  if ! kill -0 "$gazebo_pid" 2>/dev/null; then echo "[phase3] Gazebo failed during startup" >&2; exit 1; fi
  if (( SECONDS >= deadline )); then echo "[phase3] Gazebo API timeout" >&2; exit 1; fi
  sleep 2
done

# The upstream launch spawner times out after 30 s. Retry only after Gazebo is known ready.
if ! timeout 10 ros2 topic echo /odom --once >/dev/null 2>&1; then
  echo "[phase3] Retrying TurtleBot3 spawn after Gazebo is ready..."
  ros2 run gazebo_ros spawn_entity.py -entity waffle_pi \
    -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf \
    -x "$robot_x" -y "$robot_y" -z 0.01
fi

echo "[phase3] Spawning the static red target ball at ($ball_x, $ball_y)..."
ros2 run gazebo_ros spawn_entity.py -entity phase3_target_ball -file /etc/nav2_phase3/red_ball.sdf \
  -x "$ball_x" -y "$ball_y" -z 0.15 || true

wait_for /odom "robot odometry"; wait_for /scan "laser scan"
wait_for /camera/image_raw "RGB camera"; wait_for /camera/points "RGB-D point cloud"

echo "[phase3] Starting SLAM Toolbox (it owns /map and map -> odom)..."
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True \
  slam_params_file:=/etc/nav2_phase3/mapper_params_online_async.yaml &
slam_pid=$!
wait_for /map "first SLAM occupancy grid"

echo "[phase3] Starting Nav2 navigation servers without AMCL/map_server..."
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True autostart:=True &
nav2_pid=$!
sleep 8

echo "[phase3] Starting autonomous frontier exploration and ball perception..."
ros2 launch nav2_phase3 phase3.launch.py &
phase3_pid=$!

echo "[phase3] RViz opens with map/costmaps/path. Green sphere = selected frontier; red sphere = mapped ball."
echo "[phase3] Mission state: ros2 topic echo /phase3/mission_status"
rviz2 -d "${NAV2_SHARE}/rviz/nav2_default_view.rviz"
