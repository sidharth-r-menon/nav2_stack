# Phase 4 — EKF Sensor Fusion & OpenNav AprilTag Autonomous Docking (ROS 2 Jazzy)

Phase 4 implements the **official, industry-standard autonomous docking and sensor fusion stack** used on modern Autonomous Mobile Robots (AMRs), built on **ROS 2 Jazzy Jalisco**, **OpenNav Docking Server (`opennav_docking`)**, **AprilTag Visual Perception**, and **Robot Localization (`robot_localization` EKF)**.

The robot operates in the standard `turtlebot3_world`, fuses wheel odometry and IMU into `/odometry/filtered`, autonomously navigates across the world from any spawn location to the charging station staging area, and executes precision closed-loop visual servoing to dock onto the charging contacts.

For mathematical derivations, EKF configuration, Lyapunov control laws, and robotics interview questions, read the **[Phase 4 Technical Guide](PHASE4_GUIDE.md)**.

---

## 1. Architecture Overview

OpenNav Docking operates as a **two-stage autonomous system**:

```text
                           +--------------------------------------------------+
                           |                     RViz2                        |
                           |       (nav2_rviz_plugins/Docking Panel)          |
                           +--------------------------------------------------+
                                                 |  Action Goal (dock_1)
                                                 v
+------------------------+             +--------------------------------------+
| Charging Dock (0.5,-0.5)|            |        OpenNav Docking Server        |
| (AprilTag 36h11 ID 0)  |             |          (/docking_server)           |
+------------------------+             +--------------------------------------+
            | Camera View                                 |
            v                                             |
+------------------------+                     +----------+----------+
| apriltag_dock_detector |                     |                     |
| (/camera/image_raw)    |                     v                     v
+------------------------+             +---------------+     +---------------+
            |                          |    Stage 1    |     |    Stage 2    |
            | /detected_dock_pose      | Macro Nav2    |     | Graceful Dock |
            | dock_tag TF              | to Staging    |     | Controller    |
            +------------------------> | (-0.2, -0.5)  |     | to Contacts   |
                                       +---------------+     +---------------+
                                               |                     |
                                               +----------+----------+
                                                          |
                                                          v /cmd_vel
                                              +-----------------------+
                                              |  TurtleBot3 Waffle Pi |
                                              +-----------------------+
```

---

## 2. Quick Start

### 1. Prerequisites
Ensure **VcXsrv / XLaunch** is running on Windows with **"Disable access control"** checked and **"Multiple windows"** enabled.

### 2. Build & Launch Container
```powershell
# Stop any earlier phase containers
docker stop nav2_phase1 nav2_phase2 nav2_phase3 nav2_phase4 2>$null

# Build and start Phase 4
docker compose -f nav2_phase4_docker/docker-compose.yml up --build
```

### 3. Docking in RViz
1. Gazebo Sim and RViz2 will launch automatically.
2. In the **OpenNav Docking** panel on the left side of RViz, ensure **`dock_1`** is selected.
3. Click **`Dock`**:
   - The robot plans a global path and navigates to the pre-dock staging pose `(x=-0.2, y=-0.5, yaw=0.0)`.
   - The camera locks onto AprilTag `tag36h11 ID 0` on the charging station.
   - The **Graceful Controller** drives the robot smoothly onto the charging contact pads.
4. Click **`Undock`**:
   - The robot reverses safely away from the dock and re-enters normal navigation mode.

---

## 3. Testing From Different World Positions

You can test docking from any arbitrary position in two ways:

### Option A: Interactive Testing in RViz (No Restart Needed)
1. Select the **`Nav2 Goal`** tool in the RViz top toolbar.
2. Click and drag anywhere in the room (e.g. across the room, behind an obstacle, in a far corner).
3. Wait for the robot to navigate to the goal.
4. In the **OpenNav Docking** panel, click **`Dock`**.
5. The robot will plan a path through the obstacles all the way back to the staging area and dock!

### Option B: Configure Initial Spawn Coordinates
In `nav2_phase4_docker/docker-compose.yml`, change the spawn environment variables:

```yaml
    environment:
      ROBOT_SPAWN_X: "0.0"       # Custom X spawn coordinate
      ROBOT_SPAWN_Y: "1.5"       # Custom Y spawn coordinate
      ROBOT_SPAWN_YAW: "1.57"    # Custom Yaw orientation (radians)
```

Then start the container:
```powershell
docker compose -f nav2_phase4_docker/docker-compose.yml up
```

---

## 4. Telemetry & Terminal Inspection

Open a separate PowerShell terminal to inspect live topics:

```powershell
docker exec -it nav2_phase4 bash

# 1. Inspect EKF fused odometry
ros2 topic echo /odometry/filtered

# 2. Inspect detected dock pose published by vision detector
ros2 topic echo /detected_dock_pose

# 3. Check docking server action status
ros2 action list
ros2 action info /dock_robot

# 4. View TF transform between robot camera and dock tag
ros2 run tf2_ros tf2_echo camera_rgb_optical_frame dock_tag
```

---

## 5. File Structure

```text
nav2_phase4_docker/
├── Dockerfile                   # ROS 2 Jazzy container with Nav2 & OpenNav Docking
├── docker-compose.yml           # Compose specification with dynamic spawn coords & OpenGL
├── start_nav2_phase4.sh         # Master lifecycle orchestrator & startup script
├── README.md                    # Quick start & user guide
├── PHASE4_GUIDE.md              # Technical reference & interview theory
├── config/
│   ├── dock_database.yaml       # OpenNav dock definitions (dock_1 @ 0.5, -0.5)
│   ├── opennav_docking.yaml     # SimpleChargingDock & Graceful Controller params
│   ├── nav2_params.yaml         # Complete Nav2 bringup, AMCL, & controller configuration
│   ├── ekf.yaml                 # robot_localization EKF sensor fusion parameters
│   ├── turtlebot3_world.yaml    # 2D Occupancy Grid map YAML
│   ├── turtlebot3_world.pgm     # 2D Occupancy Grid map image
│   └── rviz_docking.rviz        # Pre-configured RViz display with OpenNav Docking panel
├── models/
│   └── charging_dock/           # Flat-ground charging station with AprilTag PBR texture
└── scripts/
    └── apriltag_dock_detector.py # Real-time AprilTag 36h11 detector & dock pose publisher
```
