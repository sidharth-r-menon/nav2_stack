#!/usr/bin/env bash
# Phase 3: unknown indoor world -> SLAM map -> frontier exploration (frontier_exploration_ros2)
#           -> RGB-D ball detection -> stand-off approach.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/phase3_ws/install/setup.bash

export TURTLEBOT3_MODEL=waffle_pi
# GAZEBO_MODEL_PATH needs both the turtlebot3 models (for spawning the robot)
# and this package's aws_robomaker_residential_* furniture models (referenced
# by small_house.world). Without the second path, Gazebo will load the world
# but every piece of furniture will show as a missing-mesh placeholder or
# fail to load entirely.
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:/opt/ros/humble/share/turtlebot3_gazebo/models:/opt/phase3_ws/install/aws_robomaker_small_house_world/share/aws_robomaker_small_house_world/models"
startup_timeout="${NAV2_STARTUP_TIMEOUT:-300}"
# No verified interior spawn point exists for this world yet - defaulting to
# the origin. Check in Gazebo/RViz after first launch and adjust ROBOT_X/
# ROBOT_Y if the robot spawns inside a wall or furniture.
robot_x="${ROBOT_X:-0.0}"; robot_y="${ROBOT_Y:-0.0}"
# Ball spawn position - re-check against the new floor plan once a good
# robot spawn point is confirmed; the old (4.0, 1.5) was tuned for the
# turtlebot3_house layout and has no particular meaning in this world.
ball_x="${BALL_X:-4.0}"; ball_y="${BALL_Y:-1.5}"
NAV2_SHARE="$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup"

cleanup() {
  kill "${phase3_pid:-}" "${explorer_pid:-}" "${nav2_pid:-}" "${slam_pid:-}" "${rsp_pid:-}" "${gzclient_pid:-}" "${gazebo_pid:-}" 2>/dev/null || true
  wait "${phase3_pid:-}" "${explorer_pid:-}" "${nav2_pid:-}" "${slam_pid:-}" "${rsp_pid:-}" "${gzclient_pid:-}" "${gazebo_pid:-}" 2>/dev/null || true
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

echo "[phase3] Starting the AWS RoboMaker Small House world..."
# small_house.launch.py is WORLD-ONLY: it starts gzserver (physics/world
# simulation) with the house world loaded, but does not spawn any robot and
# - confirmed via `ps aux` after a real run - does NOT start gzclient (the
# 3D viewer) either. This differs from turtlebot3_gazebo's
# turtlebot3_house.launch.py, which bundled gzserver + gzclient + robot +
# spawn all in one launch file. Robot spawn is a separate explicit step
# below, and gzclient is started explicitly further down once gzserver's
# API responds.
ros2 launch aws_robomaker_small_house_world small_house.launch.py &
gazebo_pid=$!

echo "[phase3] Waiting for Gazebo API (Docker Desktop can take a few minutes)..."
deadline=$((SECONDS + startup_timeout))
until timeout 5 ros2 service call /get_model_list gazebo_msgs/srv/GetModelList "{}" >/dev/null 2>&1; do
  if ! kill -0 "$gazebo_pid" 2>/dev/null; then echo "[phase3] Gazebo failed during startup" >&2; exit 1; fi
  if (( SECONDS >= deadline )); then echo "[phase3] Gazebo API timeout" >&2; exit 1; fi
  sleep 2
done

echo "[phase3] Starting Gazebo client (3D viewer)..."
# small_house.launch.py only starts gzserver (the physics/world simulation)
# plus the ROS plugin .so's - unlike turtlebot3_house.launch.py, which
# bundled gzserver AND gzclient together. Confirmed via `ps aux` that
# gzserver was running fine but gzclient was completely absent, which is
# why RViz loaded but the Gazebo 3D window never appeared.
gzclient &
gzclient_pid=$!

echo "[phase3] Spawning TurtleBot3 Waffle Pi at ($robot_x, $robot_y)..."
ros2 run gazebo_ros spawn_entity.py -entity waffle_pi \
  -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf \
  -x "$robot_x" -y "$robot_y" -z 0.01

echo "[phase3] Starting robot_state_publisher (publishes the URDF TF tree)..."
# turtlebot3_house.launch.py used to bundle this. It is what publishes the
# base_footprint -> base_link -> base_scan (etc.) TF chain from the URDF -
# without it, no odom/base_scan transform exists at all, SLAM Toolbox waits
# forever on that transform, and /map is never published. Confirmed missing
# in the previous run: /tf_static was empty and tf2_echo reported 'odom'
# does not exist as a frame.
ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py use_sim_time:=True &
rsp_pid=$!

echo "[phase3] Spawning the static red target ball at ($ball_x, $ball_y)..."
ros2 run gazebo_ros spawn_entity.py -entity phase3_target_ball -file /etc/nav2_phase3/red_ball.sdf \
  -x "$ball_x" -y "$ball_y" -z 0.15 || true

wait_for /odom "robot odometry"; wait_for /scan "laser scan"
# wait_for /camera/image_raw "RGB camera"; wait_for /camera/points "RGB-D point cloud"

echo "[phase3] Waiting for TF tree (base_scan present in /tf_static) from robot_state_publisher..."
tf_deadline=$((SECONDS + startup_timeout))
until timeout 3 ros2 topic echo /tf_static --once 2>/dev/null | grep -q "base_scan"; do
  if ! kill -0 "$rsp_pid" 2>/dev/null; then
    echo "[phase3] robot_state_publisher exited before publishing TF - check robot_state_publisher.launch.py args/URDF" >&2
    exit 1
  fi
  if (( SECONDS >= tf_deadline )); then
    echo "[phase3] Timed out waiting for base_scan in /tf_static. SLAM will never" >&2
    echo "[phase3] produce /map without this - check robot_state_publisher logs above." >&2
    exit 1
  fi
  sleep 1
done
echo "[phase3] TF tree confirmed."

echo "[phase3] Starting SLAM Toolbox (it owns /map and map -> odom)..."
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True \
  slam_params_file:=/etc/nav2_phase3/mapper_params_online_async.yaml &
slam_pid=$!
wait_for /map "first SLAM occupancy grid"

echo "[phase3] Starting Nav2 navigation servers without AMCL/map_server..."
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=True \
  autostart:=True \
  params_file:=/etc/nav2_phase3/nav2_params.yaml &
nav2_pid=$!
sleep 8

echo "[phase3] Starting frontier_exploration_ros2 (MRTSP-based autonomous explorer)..."
ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py \
  use_sim_time:=true \
  params_file:=/etc/nav2_phase3/frontier_exploration.yaml &
explorer_pid=$!

# echo "[phase3] Starting ball detector and approach manager..."
# ros2 launch nav2_phase3 phase3.launch.py &
# phase3_pid=$!

echo ""
echo "[phase3] =========================================================="
echo "[phase3]  Phase 3 fully started."
echo "[phase3]  World        : AWS RoboMaker Small House"
echo "[phase3]  Robot start  : ($robot_x, $robot_y)  [unverified - check Gazebo]"
echo "[phase3]  Ball target  : ($ball_x, $ball_y)  [unverified for this world]"
echo "[phase3]"
echo "[phase3]  Watch exploration: ros2 topic echo /explore/selected_frontier"
echo "[phase3]  Watch frontiers:   ros2 topic echo /explore/frontiers"
echo "[phase3]  Mission status:    ros2 topic echo /phase3/mission_status"
echo "[phase3]  Perception:        ros2 topic echo /phase3/perception_status"
echo "[phase3]  Stop explorer:     frontier_exploration_ctl stop"
echo "[phase3] =========================================================="
echo ""
rviz2 -d "${NAV2_SHARE}/rviz/nav2_default_view.rviz"