# Autonomous Mobile Robotics (AMR) Nav2 Stack Series

A complete, production-grade hands-on laboratory series for mastering **ROS 2 Navigation (Nav2)**, **Simultaneous Localization and Mapping (SLAM)**, **Autonomous Frontier Exploration & 3D Object Detection**, **Multi-Sensor Fusion (EKF)**, and **Precision Autonomous Docking (`opennav_docking`)**.

Each phase is containerized in its own isolated Docker environment with pre-tuned parameters, Gazebo simulation worlds, RViz visualizations, and in-depth technical interview guides.

---

## 🗺️ Project Roadmap & Phase Comparison

| Phase | Core Focus | Simulation World | Key Technologies | ROS 2 Distro | Links |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Phase 1** | **Canonical Navigation & Localization** | `turtlebot3_world` | AMCL, Costmaps (Global/Local), NavFn Planner, DWB Controller | Humble | [Run Guide](nav2_phase1_docker/README.md) • [Tech Guide](nav2_phase1_docker/PHASE1_GUIDE.md) |
| **Phase 2** | **Online Asynchronous Mapping & SLAM** | `turtlebot3_world` | SLAM Toolbox (Karto SLAM, Ceres Scan Matching, Pose Graph) | Humble | [Run Guide](nav2_phase2_docker/README.md) • [Tech Guide](nav2_phase2_docker/PHASE2_GUIDE.md) |
| **Phase 3** | **Autonomous Exploration & 3D RGB-D Perception** | `turtlebot3_house` | Frontier Exploration, OpenCV Color Segmentation, PointCloud2 3D Clustering | Humble | [Run Guide](nav2_phase3_docker/README.md) • [Tech Guide](nav2_phase3_docker/PHASE3_GUIDE.md) |
| **Phase 4** | **EKF Sensor Fusion & OpenNav Autonomous Docking** | `turtlebot3_world` | `opennav_docking`, AprilTag 36h11 Visual Servoing, `robot_localization` EKF, Graceful Controller | Jazzy | [Run Guide](nav2_phase4_docker/README.md) • [Tech Guide](nav2_phase4_docker/PHASE4_GUIDE.md) |

---

## 📦 Phase Deep Dives

### [Phase 1 — Canonical Nav2 Navigation & AMCL Localization](nav2_phase1_docker/README.md)
- **Objective:** Understand the baseline Nav2 pipeline, TF tree (`map → odom → base_footprint → base_link → base_scan`), static map server, and Adaptive Monte Carlo Localization (KLD particle filter).
- **Key Concepts:** Global and Local costmaps (Static, Obstacle, Inflation layers), Nav2 Goal lifecycle transitions, and obstacle avoidance.
- **Documentation:**
  - 📖 **[Phase 1 Run Guide](nav2_phase1_docker/README.md)**: Container setup, RViz goals, and topic inspection.
  - 🎓 **[Phase 1 Architecture & Interview Guide](nav2_phase1_docker/PHASE1_GUIDE.md)**: Deep mathematical derivation of AMCL, particle cloud resampling, and TF transform conventions.

---

### [Phase 2 — Online Asynchronous SLAM with SLAM Toolbox](nav2_phase2_docker/README.md)
- **Objective:** Navigate and explore an unknown environment from scratch without a pre-existing map using `slam_toolbox`.
- **Key Concepts:** Ceres scan matcher, covariance computation, loop closure detection, interactive pose graph optimization, and map serialization.
- **Documentation:**
  - 📖 **[Phase 2 Run Guide](nav2_phase2_docker/README.md)**: SLAM execution, live map building, and `save_phase2_map.sh` workflow.
  - 🎓 **[Phase 2 SLAM & Interview Guide](nav2_phase2_docker/PHASE2_GUIDE.md)**: Scan matching mathematics, graph-based SLAM fundamentals, and technical interview Q&A.

---

### [Phase 3 — Autonomous Frontier Exploration & RGB-D Object Detection](nav2_phase3_docker/README.md)
- **Objective:** The robot starts in an unknown multi-room house (`turtlebot3_house`), autonomously discovers frontiers using wavefront detection, navigates to unknown rooms, identifies a red target ball using 3D RGB-D point clouds, and executes an approach behavior.
- **Key Concepts:** Information-gain frontier selection, OpenCV HSV color mask segmentation, camera ray back-projection, PointCloud2 centroid estimation, and Dynamic Costmaps.
- **Documentation:**
  - 📖 **[Phase 3 Run Guide](nav2_phase3_docker/README.md)**: Multi-room exploration instructions and target approach telemetry.
  - 🎓 **[Phase 3 Technical Guide](nav2_phase3_docker/PHASE3_GUIDE.md)**: Wavefront frontier algorithms, PnP camera transforms, and perception state machines.

---

### [Phase 4 — EKF Sensor Fusion & OpenNav AprilTag Autonomous Docking](nav2_phase4_docker/README.md)
- **Objective:** Implement the official industry-standard autonomous docking stack using **ROS 2 Jazzy**, **OpenNav Docking Server (`opennav_docking`)**, **AprilTag 36h11 Visual Perception**, and **`robot_localization` Extended Kalman Filter (EKF)**.
- **Key Concepts:** 15-state EKF odometry/IMU sensor fusion, decoupled two-stage macro/micro docking, Lyapunov-stable polar Graceful Controller, PnP pose estimation, and native Qt RViz docking panel.
- **Documentation:**
  - 📖 **[Phase 4 Run Guide](nav2_phase4_docker/README.md)**: Docker setup, multi-location spawn testing, and RViz Dock/Undock controls.
  - 🎓 **[Phase 4 Technical Guide](nav2_phase4_docker/PHASE4_GUIDE.md)**: EKF Jacobian equations, Graceful Controller Lyapunov stability proofs, and AMR docking interview theory.

---

## 🛠️ Prerequisites & Setup (Windows & Linux)

### 1. Docker Desktop
Ensure Docker Desktop is installed and running with Linux containers enabled.

### 2. GUI Display Server on Windows (VcXsrv / XLaunch)
Before launching any container:
1. Start **XLaunch**.
2. Select **Multiple windows**, Display number **`0`**.
3. Select **Start no client**.
4. Check **"Disable access control"** (crucial for Docker container X11 forwarding).
5. Complete the setup and allow the firewall prompt.

---

## 🚀 Quick Launch Cheat-Sheet

```powershell
# Phase 1: Canonical AMCL Navigation
docker compose -f nav2_phase1_docker/docker-compose.yml up --build

# Phase 2: SLAM Toolbox Online Mapping
docker compose -f nav2_phase2_docker/docker-compose.yml up --build

# Phase 3: Autonomous Exploration & 3D Object Detection
docker compose -f nav2_phase3_docker/docker-compose.yml up --build

# Phase 4: EKF Sensor Fusion & OpenNav AprilTag Docking (ROS 2 Jazzy)
docker compose -f nav2_phase4_docker/docker-compose.yml up --build
```

---

## 📂 Repository Directory Layout

```text
nav2_stack/
├── README.md                    # Master Project Overview & Roadmap (This file)
│
├── nav2_phase1_docker/          # Phase 1: AMCL, Static Map & Nav2 Bringup (Humble)
│   ├── Dockerfile
│   ├── docker-compose.yml
│   ├── start_nav2_phase1.sh
│   ├── README.md
│   └── PHASE1_GUIDE.md
│
├── nav2_phase2_docker/          # Phase 2: SLAM Toolbox Online Mapping (Humble)
│   ├── Dockerfile
│   ├── docker-compose.yml
│   ├── start_nav2_phase2.sh
│   ├── save_phase2_map.sh
│   ├── config/mapper_params_online_async.yaml
│   ├── README.md
│   └── PHASE2_GUIDE.md
│
├── nav2_phase3_docker/          # Phase 3: Frontier Exploration & RGB-D Object Detection (Humble)
│   ├── Dockerfile
│   ├── docker-compose.yml
│   ├── start_nav2_phase3.sh
│   ├── README.md
│   └── PHASE3_GUIDE.md
│
└── nav2_phase4_docker/          # Phase 4: EKF Sensor Fusion & OpenNav AprilTag Docking (Jazzy)
    ├── Dockerfile
    ├── docker-compose.yml
    ├── start_nav2_phase4.sh
    ├── README.md
    ├── PHASE4_GUIDE.md
    ├── config/                  # Dock database, OpenNav docking & Nav2 params
    ├── models/                  # Charging dock 3D model with AprilTag PBR texture
    └── scripts/                 # Real-time AprilTag 36h11 detector & dock pose publisher
```
