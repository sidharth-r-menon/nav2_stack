# Nav2 Phase 1 — TurtleBot3, Gazebo Classic, RViz, and AMCL

This repository is the first Nav2 lab. It uses **one Docker container** to run
the stock ROS 2 Humble TurtleBot3 Waffle simulation, Gazebo Classic, Nav2,
AMCL localization, and RViz.

For a detailed explanation of the TF tree, topics, components, RViz colors,
2D Pose Estimate, the navigation pipeline, troubleshooting, and interview
questions, read the **[Phase 1 architecture and interview guide](nav2_phase1_docker/PHASE1_GUIDE.md)**.

Phase 2 is also implemented as a separate lab. It replaces AMCL and the saved
map with SLAM Toolbox so the robot can build the map while navigating. See the
**[Phase 2 run guide](nav2_phase2_docker/README.md)** and the detailed
**[Phase 2 SLAM and interview guide](nav2_phase2_docker/PHASE2_GUIDE.md)**.

Phase 3 is implemented: the robot starts in an unknown multi-room house,
maps it with SLAM, autonomously selects frontiers, detects a red ball using an
RGB-D PointCloud2, and navigates to the object. Read the
**[Phase 3 guide](nav2_phase3_docker/README.md)**.

There is intentionally no custom robot code, SLAM configuration, or parameter
tuning in this phase. The aim is to understand and verify the canonical Nav2
pipeline before transferring it to a custom base.

## What this phase proves

```text
Gazebo → /scan, /odom, /tf, /clock
                 ↓
     AMCL + static map → map → odom
                 ↓
     Global planner → local controller → /cmd_vel
                 ↓
               TurtleBot3
```

After completing Phase 1, you should be able to:

1. Explain `map → odom → base_link`.
2. Set an initial pose in RViz and understand why AMCL needs it.
3. Send a navigation goal and inspect the global path, local plan, and
   costmaps.
4. Relate Nav2's `/cmd_vel` output to the simulated base's execution.
5. Diagnose the basic failure classes: missing TF, no scan, no odometry,
   inactive lifecycle nodes, or an invalid goal.

## Repository layout

```text
nav2_phase1_docker/
├── Dockerfile
├── docker-compose.yml
├── start_nav2_phase1.sh
├── README.md
└── .gitignore

nav2_phase1_ws/             # created beside this repository; mounted at /ws

nav2_phase2_docker/
├── Dockerfile
├── docker-compose.yml
├── start_nav2_phase2.sh
├── save_phase2_map.sh
├── config/mapper_params_online_async.yaml
├── README.md
└── PHASE2_GUIDE.md

nav2_phase2_ws/             # saved maps and serialized pose graphs

nav2_phase3_docker/         # autonomous multi-room exploration + RGB-D target approach
nav2_phase3_ws/             # Phase 3 saved maps and serialized pose graphs
```

The workspace directory is empty in Phase 1. It is mounted now so it becomes
the natural home for the mapping, localization, configuration, and custom
robot packages added in later phases.

## Requirements on Windows

- Docker Desktop using Linux containers.
- VcXsrv/XLaunch to display Gazebo and RViz on Windows.

Start **XLaunch** before Docker Compose:

1. Select **Multiple windows**.
2. Set display number to **0**.
3. Select **Start no client**.
4. Enable **Disable access control**.
5. Finish and allow the private-network firewall prompt.

The container is configured to use `host.docker.internal:0.0`, so no WSL GUI
configuration is required.

## First run

Place this repository at, for example:

```text
D:\GitHub\nav2_lab\nav2_phase1_docker
```

Create the adjacent workspace directory once:

```powershell
mkdir D:\GitHub\nav2_lab\nav2_phase1_ws
```

Then, from PowerShell:

```powershell
cd D:\GitHub\nav2_lab\nav2_phase1_docker
docker compose up --build -d
docker compose logs -f nav2
```

Two windows should open automatically:

- **Gazebo Classic:** the ground-truth world and TurtleBot3.
- **RViz:** map, TF, LiDAR scan, AMCL particles, paths, and costmaps.

## Run a navigation goal in RViz

1. In Gazebo, note the robot's approximate start position.
2. In RViz, click **2D Pose Estimate**.
3. Click/drag on the map to place the robot at that position and orientation.
4. Wait for the particle cloud and TF/costmap displays to settle.
5. Click **Nav2 Goal**, then click/drag a free location on the map.

Nav2 should make a global route and repeatedly publish velocity commands on
`/cmd_vel`. Gazebo executes them; `/odom`, `/scan`, and TF update Nav2 as the
robot moves.

## Inspect the running system

Open a second PowerShell window:

```powershell
cd D:\GitHub\nav2_lab\nav2_phase1_docker
docker compose exec nav2 bash
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash

ros2 topic list
ros2 node list
ros2 lifecycle nodes

ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 topic echo /cmd_vel
```

The key topics are:

| Topic | Producer | Meaning |
| --- | --- | --- |
| `/scan` | Gazebo LiDAR plugin | Nearby obstacle ranges |
| `/odom` | Simulated base | Short-term motion estimate |
| `/tf` and `/tf_static` | Robot + AMCL | Robot/sensor and localization transforms |
| `/map` | Map server | Static occupancy map |
| `/cmd_vel` | Nav2 controller | Desired linear/angular velocity |
| `/plan` | Global planner | Global route to the goal |
| `/local_plan` | Controller | Short-horizon trajectory/command plan |

## Useful experiments for this phase

Perform these without modifying code:

1. Send a clear goal in open space and observe the global/local paths.
2. Send a goal around an obstacle and observe the global route go around it.
3. Drag a Gazebo obstacle into the route and inspect the local costmap and
   controller reaction.
4. Give an intentionally wrong initial pose, observe the failure, then correct
   it with **2D Pose Estimate**.
5. Stop navigation, then inspect `/cmd_vel` becoming zero.

## Lifecycle check

The Nav2 launch uses `autostart:=True`, so normally it activates its lifecycle
nodes automatically. If you need to verify the state:

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
```

Each should be `active` before it can navigate.

## Shutdown

From PowerShell:

```powershell
docker compose down
```

This removes the container but keeps the image. The next `docker compose up -d`
is therefore much faster. No named volume is used in Phase 1.

## Project phases

- **Phase 1 — implemented:** known-map navigation using map server, AMCL, and
  Nav2.
- **Phase 2 — implemented:** online asynchronous SLAM with SLAM Toolbox,
  navigation within observed space, and occupancy/pose-graph saving.
- **Phase 3 — proposed:** dynamic-obstacle experiments, replanning, recovery,
  and a Collision Monitor stop/slowdown safety layer. RGB-D PointCloud2 input
  can be added as an optional extension.

## References

- [Nav2 Getting Started](https://docs.nav2.org/getting_started/index.html)
- [Nav2 Costmaps](https://docs.nav2.org/configuration/packages/configuring-costmaps.html)
- [Nav2 Behavior Tree Navigator](https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html)
