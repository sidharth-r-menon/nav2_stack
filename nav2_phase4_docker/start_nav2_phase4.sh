#!/usr/bin/env bash
# ==============================================================================
# start_nav2_phase4.sh — Autonomous Docking & Sensor Fusion with ROS 2 Jazzy
# Uses OpenNav Docking Server (opennav_docking), AprilTag ROS & Robot Localization
# ==============================================================================

set -e
source /opt/ros/jazzy/setup.bash

export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/jazzy/share/turtlebot3_gazebo/models:/workspace_docker/models
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/workspace_docker/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models
export IGN_GAZEBO_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONUNBUFFERED=1
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330
export QT_X11_NO_MITSHM=1

SPAWN_X=${ROBOT_SPAWN_X:-"-2.0"}
SPAWN_Y=${ROBOT_SPAWN_Y:-"-0.5"}
SPAWN_YAW=${ROBOT_SPAWN_YAW:-"0.0"}

startup_timeout=300

echo "======================================================================"
echo "  Nav2 Phase 4: Autonomous Docking & Sensor Fusion (ROS 2 Jazzy)     "
echo "  - opennav_docking (Open Navigation Docking Server)                  "
echo "  - apriltag_dock_detector (Visual Perception & Pose Publisher)       "
echo "  - robot_localization EKF (Fused Wheel Odometry + IMU)               "
echo "  - Nav2 Bringup (AMCL Localization & Costmaps)                       "
echo "  - RViz2 with Native Nav2 Docking & Goal Control Panels              "
echo "  - Robot Spawn Pose: ($SPAWN_X, $SPAWN_Y, yaw=$SPAWN_YAW)           "
echo "======================================================================"

# ──────────────────────────────────────────────────────────────────────────────
# 1. Start Gazebo Simulation
# ──────────────────────────────────────────────────────────────────────────────
echo "[nav2_phase4] Starting TurtleBot3 Waffle Pi in Gazebo at ($SPAWN_X, $SPAWN_Y)..."
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py \
  x_pose:=${SPAWN_X} \
  y_pose:=${SPAWN_Y} \
  yaw:=${SPAWN_YAW} \
  z_pose:=0.01 &
GAZEBO_PID=$!

# Wait for Gazebo API
echo "[nav2_phase4] Waiting for Gazebo's API (up to ${startup_timeout}s)..."
deadline=$((SECONDS + startup_timeout))
while ! ros2 topic list 2>/dev/null | grep -q '/clock'; do
  if [ $SECONDS -ge $deadline ]; then
    echo "[nav2_phase4 ERROR] Timed out waiting for Gazebo /clock."
    exit 1
  fi
  sleep 1
done
echo "[nav2_phase4] Gazebo is running!"

echo "[nav2_phase4] Waiting for robot odometry and IMU..."
deadline=$((SECONDS + startup_timeout))
while ! ros2 topic list 2>/dev/null | grep -q '/odom'; do
  if [ $SECONDS -ge $deadline ]; then
    echo "[nav2_phase4 ERROR] Timed out waiting for /odom."
    exit 1
  fi
  sleep 1
done
echo "[nav2_phase4] Robot sensors active!"

# Spawn vertical flat AprilTag charging dock at (0.5, -0.5, 0.0) facing West (Yaw=0.0)
echo "[nav2_phase4] Spawning AprilTag Charging Dock at (x=0.5, y=-0.5)..."
ros2 run ros_gz_sim create \
  -world default \
  -file /workspace_docker/models/charging_dock/model.sdf \
  -name charging_dock \
  -x 0.5 \
  -y -0.5 \
  -z 0.0 \
  -Y 0.0 || true

# ──────────────────────────────────────────────────────────────────────────────
# 2. Start Extended Kalman Filter (EKF) Sensor Fusion
# ──────────────────────────────────────────────────────────────────────────────
echo "[nav2_phase4] Starting robot_localization EKF (fusing wheel odom + IMU)..."
ros2 run robot_localization ekf_node \
  --ros-args \
  --params-file /workspace_docker/config/ekf.yaml \
  --remap /odometry/filtered:=/odometry/filtered &
EKF_PID=$!

# ──────────────────────────────────────────────────────────────────────────────
# 3. Start Nav2 Stack (AMCL Localization, Costmaps & OpenNav Docking Server)
# ──────────────────────────────────────────────────────────────────────────────
echo "[nav2_phase4] Starting Nav2 Bringup with map and docking server..."
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=True \
  map:=/workspace_docker/config/turtlebot3_world.yaml \
  params_file:=/workspace_docker/config/nav2_params.yaml \
  autostart:=True &
NAV2_PID=$!

# ──────────────────────────────────────────────────────────────────────────────
# 4. Auto-publish initial AMCL pose
# ──────────────────────────────────────────────────────────────────────────────
(
  echo "[nav2_phase4] Initializing AMCL pose broadcaster..."
  for i in {1..8}; do
    sleep 3
    ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
      "{header: {frame_id: 'map'}, pose: {pose: {position: {x: ${SPAWN_X}, y: ${SPAWN_Y}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.068]}}" >/dev/null 2>&1 || true
  done
) &

# ──────────────────────────────────────────────────────────────────────────────
# 5. Start AprilTag Visual Perception & Dock Pose Publisher
# ──────────────────────────────────────────────────────────────────────────────
echo "[nav2_phase4] Starting AprilTag Dock Detector & Pose Publisher..."
python3 /workspace_docker/scripts/apriltag_dock_detector.py &
APRILTAG_PID=$!

# ──────────────────────────────────────────────────────────────────────────────
# 6. Start RViz2 with Native Nav2 Docking & Navigation Panels
# ──────────────────────────────────────────────────────────────────────────────
echo "[nav2_phase4] Starting RViz2 with OpenNav Docking & Navigation Panels..."
rviz2 -d /workspace_docker/config/rviz_docking.rviz &
RVIZ_PID=$!

cleanup() {
  echo "[nav2_phase4] Shutting down all processes..."
  kill -TERM $RVIZ_PID $APRILTAG_PID $NAV2_PID $EKF_PID $GAZEBO_PID 2>/dev/null || true
  wait 2>/dev/null || true
  echo "[nav2_phase4] Done."
}
trap cleanup SIGINT SIGTERM
wait $RVIZ_PID $GAZEBO_PID 2>/dev/null || wait
