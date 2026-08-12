# Nav2 Phase 2 — SLAM and Navigation While Mapping

Phase 2 removes the prebuilt map, map server, and AMCL from the active
pipeline. SLAM Toolbox builds `/map` from `/scan` and odometry while also
providing `map → odom`. Nav2 plans and controls within the part of the world
that has already been observed.

For the concepts behind scan matching, pose graphs, loop closure, map growth,
TF ownership, RViz interpretation, saving, testing, and interview questions,
read the **[Phase 2 SLAM and interview guide](PHASE2_GUIDE.md)**.

## What changes from Phase 1

```text
Phase 1: saved map → map_server + AMCL → map → odom
Phase 2: scan + odometry → SLAM Toolbox → /map + map → odom
```

Nav2's planner, controller, costmaps, behavior tree, velocity smoother, Gazebo,
and RViz remain. Do not run AMCL at the same time: SLAM Toolbox is now the
single owner of `map → odom`.

## Start Phase 2

Start XLaunch/VcXsrv as described in the Phase 1 README. Stop Phase 1 if it is
running because both Gazebo instances use the same default Gazebo master port:

```powershell
cd D:\GitHub\nav2_stack\nav2_phase1_docker
docker compose stop

cd D:\GitHub\nav2_stack\nav2_phase2_docker
docker compose up --build -d
docker compose logs -f nav2
```

The first launch may take several minutes on Docker Desktop while Gazebo loads.
The startup script waits for the API, retries robot spawning if required, and
does not start SLAM or Nav2 until `/odom` and `/scan` are real.

## What to do in RViz

Unlike Phase 1, **do not click 2D Pose Estimate**. SLAM begins with the robot's
current odometry pose and creates the map around it.

1. Expand the **Amcl Particle Swarm** display and disable it; AMCL is absent.
2. Confirm that `/map`, LaserScan, TF, and both costmaps are visible.
3. Drive manually to expose the environment:

   ```powershell
   docker compose exec nav2 bash
   source /opt/ros/humble/setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

4. Move slowly and revisit earlier areas so SLAM Toolbox can form loop
   closures. Avoid fast rotation and collisions.
5. Use **Nav2 Goal** only inside free space that is already mapped.
6. Observe `/map` grow and the global plan update.

SLAM does not choose exploration goals. Automatic exploration would require a
separate frontier-exploration node. Here, the operator explores with teleop or
chooses goals in observed space.

## Save the result

After exploring, save both representations:

```powershell
docker compose exec nav2 save_phase2_map.sh interview_map
```

This writes into `nav2_phase2_ws/maps/`:

- `interview_map.yaml` and `interview_map.pgm`: occupancy map for later AMCL
  navigation.
- `interview_map.posegraph` and `interview_map.data`: serialized SLAM graph
  metadata and sensor data for continuing or refining mapping later. Keep the
  pair together.

The occupancy image loses pose-graph constraints and scan history; the two-file
serialized pose graph preserves SLAM's editable mapping state.

## Verify the architecture

Inside the container:

```bash
source /opt/ros/humble/setup.bash

ros2 node list
ros2 topic info -v /map
ros2 run tf2_ros tf2_echo map odom
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
```

Expected results:

- `/slam_toolbox` exists.
- `/amcl` and `/map_server` do not exist.
- `/map` is published by `slam_toolbox`.
- SLAM Toolbox publishes `map → odom`.
- The controller and planner are active.

## Suggested demonstrations

1. Begin with only the nearby walls visible and watch `/map` expand.
2. Drive a loop and explain pose-graph loop closure.
3. Send a goal through mapped free space while mapping continues.
4. Attempt a goal in unknown space and explain why planning may fail.
5. Save the map and explain occupancy map versus serialized pose graph.
6. Restart later with the saved occupancy map and AMCL to connect Phase 2 back
   to the Phase 1 architecture.

## Shutdown

```powershell
docker compose down
```
