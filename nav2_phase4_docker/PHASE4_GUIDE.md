# Phase 4 Technical Guide: EKF Sensor Fusion & OpenNav AprilTag Autonomous Docking

## 1. Overview & Objectives

Phase 4 implements a production-grade, two-stage autonomous docking architecture for Autonomous Mobile Robots (AMRs) using **ROS 2 Jazzy Jalisco**, **Open Navigation Docking (`opennav_docking`)**, **AprilTag 36h11 Visual Perception**, and **`robot_localization` Extended Kalman Filter (EKF)**.

The robot is tasked with:
1. Fusing raw wheel encoder odometry (`/odom`) with 9-DOF IMU data (`/imu`) to eliminate angular drift and wheel slippage.
2. Navigating globally across obstacles in `turtlebot3_world` to the charging station's pre-dock staging pose (`staging_x_offset: -0.7m`).
3. Transitioning seamlessly from global costmap navigation to closed-loop visual servoing using AprilTag 36h11 detection and the Lyapunov-stable **Graceful Controller**.

---

## 2. System Architecture

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

## 3. Core Algorithms & Mathematical Formulations

### 3.1 Two-Stage Docking Concept

Standard navigation stacks (NavFn, Smac, MPPI, DWB) rely on **costmap obstacle inflation**. Because a physical charging station is mounted to or against a wall, costmap inflation layers treat the charging contacts as a collision hazard, preventing standard planners from driving directly into contact.

OpenNav Docking solves this via a decoupled two-stage strategy:

| Stage | Subsystem | Objective | Control / Planning Law |
| :--- | :--- | :--- | :--- |
| **Stage 1 (Macro Nav)** | Nav2 Planner & Controller Server | Navigate robot from any map position around obstacles to the **Staging Pose** | Global Costmap + NavFn / MPPI |
| **Stage 2 (Micro Docking)** | `opennav_docking` + AprilTag Detector | Visual servoing approach from staging pose directly into the dock contact pads | Polar Lyapunov Graceful Controller |

---

### 3.2 Extended Kalman Filter (EKF) Sensor Fusion

The `robot_localization::ekf_node` runs at 50 Hz, fusing high-rate wheel odometry and IMU measurements.

#### State Vector (15 Dimensions)
\[
\mathbf{x} = \begin{bmatrix}
x & y & z & \phi & \theta & \psi & \dot{x} & \dot{y} & \dot{z} & \dot{\phi} & \dot{\theta} & \dot{\psi} & \ddot{x} & \ddot{y} & \ddot{z}
\end{bmatrix}^T
\]

#### Prediction Step
The state is propagated forward in time using a continuous-time kinematic model:
\[
\mathbf{\hat{x}}_{k|k-1} = f(\mathbf{\hat{x}}_{k-1|k-1}, \mathbf{u}_k)
\]
\[
\mathbf{P}_{k|k-1} = \mathbf{F}_k \mathbf{P}_{k-1|k-1} \mathbf{F}_k^T + \mathbf{Q}_k
\]
Where \(\mathbf{F}_k\) is the Jacobian of the motion model and \(\mathbf{Q}_k\) is the process noise covariance matrix.

#### Measurement Update
When wheel odometry and IMU packets arrive:
\[
\mathbf{y}_k = \mathbf{z}_k - h(\mathbf{\hat{x}}_{k|k-1})
\]
\[
\mathbf{S}_k = \mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^T + \mathbf{R}_k
\]
\[
\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{S}_k^{-1}
\]
\[
\mathbf{\hat{x}}_{k|k} = \mathbf{\hat{x}}_{k|k-1} + \mathbf{K}_k \mathbf{y}_k
\]
\[
\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_{k|k-1}
\]

**Fused Signals in Phase 4:**
- **Wheel Odometry (`/odom`)**: Measures linear velocities \(\dot{x}, \dot{y}\) and position \(x, y\).
- **IMU (`/imu`)**: Measures angular velocity \(\dot{\psi}\) (yaw rate) with high precision, removing angular drift caused by wheel slip.

---

### 3.3 Graceful Dock Controller (Lyapunov Polar Control Law)

The Graceful Controller (`nav2_graceful_controller::GracefulController`) models the robot's motion in polar coordinates relative to the dock target pose \((x_d, y_d, \theta_d)\):

Let \(\rho\) be the distance to the target, \(\alpha\) be the heading angle relative to the line-of-sight vector, and \(\beta\) be the target orientation error:

\[
\rho = \sqrt{\Delta x^2 + \Delta y^2}
\]
\[
\alpha = \text{atan2}(\Delta y, \Delta x) - \theta
\]
\[
\beta = -\theta - \alpha
\]

The control velocities \((v, \omega)\) are governed by the Lyapunov control law:
\[
v = v_{\max} \cdot \cos(\alpha) \cdot \tanh(\rho / d_0)
\]
\[
\omega = k_\alpha \alpha + k_\beta \beta + k_\rho \frac{\sin(\alpha)\cos(\alpha)}{\rho}
\]

**Key Advantages for Docking:**
1. **Zero Heading Overshoot**: Smoothly transitions from orienting towards the tag to asymptotic final alignment.
2. **Velocity Scaling**: Automatically decelerates to \(v_{\min} = 0.05\text{ m/s}\) as distance \(\rho \to 0\).
3. **Collision Resistance**: Unlike pure pursuit, it maintains smooth curvature paths even if the robot approaches at an offset angle.

---

### 3.4 AprilTag 36h11 Visual Pose Estimation

The perception node uses `pupil_apriltags` to solve the Perspective-n-Point (PnP) problem for Tag ID 0:
1. **Camera Intrinsics**: Extracted from `/camera/camera_info`:
   \[
   \mathbf{K} = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}
   \]
2. **Pose Recovery**: Given 4 known coplanar corners of the 15cm tag (\(s = 0.15\text{ m}\)), PnP estimates the 3D translation \(\mathbf{t}_{c}\) and rotation matrix \(\mathbf{R}_{c}\) relative to the camera optical frame.
3. **Frame Transformation**: Converts camera-frame coordinates to the global `odom` / `map` frame using TF buffer lookups:
   \[
   \mathbf{T}_{\text{odom}}^{\text{tag}} = \mathbf{T}_{\text{odom}}^{\text{camera}} \cdot \mathbf{T}_{\text{camera}}^{\text{tag}}
   \]
4. **Publishing**: Broadcasts TF frame `dock_tag` and publishes `geometry_msgs/msg/PoseStamped` onto `/detected_dock_pose`.

---

## 4. Configuration Reference

### 4.1 Dock Database (`config/dock_database.yaml`)
```yaml
docks:
  dock_1:
    type: simple_charging_dock
    frame: map
    pose: [0.5, -0.5, 0.0]   # Global location of dock in turtlebot3_world
```

### 4.2 Docking Server (`config/opennav_docking.yaml`)
```yaml
docking_server:
  ros__parameters:
    controller_frequency: 50.0
    initial_perception_timeout: 5.0
    dock_approach_timeout: 30.0
    base_frame: "base_footprint"
    fixed_frame: "odom"
    dock_backwards: false

    dock_plugins: ['simple_charging_dock']
    simple_charging_dock:
      plugin: 'opennav_docking::SimpleChargingDock'
      docking_threshold: 0.05
      staging_x_offset: -0.7           # Stops 0.7m in front of dock for tag acquisition
      use_external_detection_pose: true # Listens to /detected_dock_pose
      filter_coef: 0.1

    controller:
      k_phi: 3.0
      k_delta: 2.0
      v_linear_min: 0.05
      v_linear_max: 0.15
      use_collision_detection: false
```

---

## 5. Robotics Interview Questions & Concepts

### Q1: Why do AMRs require a dedicated docking server instead of sending a standard Nav2 goal to the charger?
**Answer**:
1. **Costmap Obstacle Inflation**: Standard Nav2 navigation inflates obstacles by the robot's radius + safety margin (e.g. \(0.2\text{ m} - 0.5\text{ m}\)). A charging dock against a wall falls inside the lethal cost zone, causing standard path planners to reject the goal as untraversable.
2. **Precision vs General Navigation**: General navigation controllers (like MPPI or Pure Pursuit) have tolerances around \(\pm 5\text{ cm}\) and \(\pm 5^\circ\). Physical spring-loaded pogo pins require \(\pm 1\text{ cm}\) and \(\pm 1^\circ\) alignment, which requires closed-loop sensor-based visual servoing.
3. **Behavior Tree Isolation**: Docking and undocking involve specific state machines (pre-staging, sensor acquisition, contact checking, charging verification, and blind undocking reverse maneuvers) that must not trigger generic costmap recovery behaviors (such as spinning in place inside a dock).

---

### Q2: How does the Graceful Controller prevent heading oscillations when approaching a target?
**Answer**:
Pure Pursuit and standard PID controllers suffer from singular behavior at near-zero velocities when the line-of-sight vector rapidly shifts angle. The Graceful Controller formulates the control law in polar coordinates using a Lyapunov function that decouples angular velocity \(\omega\) into line-of-sight alignment (\(\alpha\)), target heading error (\(\beta\)), and curvature damping. This ensures that the robot aligns with the approach vector early and glides asymptotically into the target heading with monotonically decreasing lateral error.

---

### Q3: What is the purpose of fusing IMU and Wheel Odometry using an EKF?
**Answer**:
- **Wheel Odometry**: Provides accurate short-term linear displacement \(\Delta x\), but accumulates massive orientation error \(\Delta \theta\) due to wheel slip, uneven floor friction, and gear backlash.
- **IMU Gyroscope**: Measures angular velocity \(\omega_z\) directly via Coriolis vibrating elements, unaffected by wheel slip.
- **EKF Fusion**: By assigning a lower measurement covariance \(R\) to the IMU yaw velocity and wheel linear velocities, the EKF produces an odometry frame (`/odometry/filtered`) with near-zero angular drift, providing smooth transforms for AMCL and the local costmap.
