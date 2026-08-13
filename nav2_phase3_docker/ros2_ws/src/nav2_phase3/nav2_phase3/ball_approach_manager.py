#!/usr/bin/env python3
"""Keep exploration upstream; only hand off a confirmed perception target to Nav2.

When the ball is detected, this node:
  1. Calls the /control_exploration service (frontier_exploration_ros2) to STOP the explorer.
  2. Waits until a safe stand-off pose in the map is available.
  3. Sends a NavigateToPose goal to Nav2 to approach the ball.
"""
import math
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

# frontier_exploration_ros2 runtime control service
from frontier_exploration_ros2.srv import ControlExploration


def yaw_q(yaw):
    from geometry_msgs.msg import Quaternion
    q = Quaternion(); q.z = math.sin(yaw / 2.0); q.w = math.cos(yaw / 2.0); return q


class BallApproachManager(Node):
    def __init__(self):
        super().__init__('ball_approach_manager')
        self.declare_parameter('approach_distance', .70)
        self.standoff = self.get_parameter('approach_distance').value

        self.map = None
        self.target = None
        self.goal_sent = False
        self.explore_stopped = False
        self._approach_timer = None   # only ever one timer
        self._last_say_msg = None     # rate-limit identical log lines

        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.create_subscription(PoseStamped, '/phase3/ball_pose', self.ball_cb, 10)
        self.status_pub = self.create_publisher(String, '/phase3/mission_status', 10)
        # Client for frontier_exploration_ros2 runtime control
        self.explore_ctl = self.create_client(ControlExploration, 'control_exploration')
        self.get_logger().info('BallApproachManager ready. Waiting for /phase3/ball_pose...')

    def map_cb(self, msg): self.map = msg

    def say(self, message):
        """Publish status and log – suppresses consecutive identical messages."""
        if message != self._last_say_msg:
            self.get_logger().info(message)
            self._last_say_msg = message
        self.status_pub.publish(String(data=message))

    def robot_xy(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def free(self, x, y, clearance=4):
        if self.map is None: return False
        i = self.map.info
        c = int((x - i.origin.position.x) / i.resolution)
        r = int((y - i.origin.position.y) / i.resolution)
        data = np.asarray(self.map.data, dtype=np.int16).reshape(i.height, i.width)
        if r < clearance or c < clearance or r >= i.height - clearance or c >= i.width - clearance:
            return False
        return (data[r, c] >= 0 and data[r, c] <= 25 and
                not np.any(data[r - clearance:r + clearance + 1,
                               c - clearance:c + clearance + 1] >= 65))

    def ball_cb(self, pose):
        if self.goal_sent:
            return
        self.target = pose

        # Stop the frontier explorer once
        if not self.explore_stopped:
            self._stop_explorer()

        # Start the approach-polling timer exactly once
        if self._approach_timer is None:
            self._approach_timer = self.create_timer(1.0, self.try_approach)
            self.say('BALL_FOUND: paused frontier explorer; selecting a safe stand-off pose')

    def _stop_explorer(self):
        """Call /control_exploration ACTION_STOP to halt frontier_exploration_ros2."""
        self.explore_stopped = True  # set before async so we don't retry
        if not self.explore_ctl.service_is_ready():
            self.get_logger().warn(
                'control_exploration service not ready – explorer may still be running'
            )
            return
        req = ControlExploration.Request()
        req.action = ControlExploration.Request.ACTION_STOP
        future = self.explore_ctl.call_async(req)
        future.add_done_callback(self._stop_response)

    def _stop_response(self, future):
        try:
            resp = future.result()
            self.get_logger().info(
                f'Explorer stop response: success={resp.success}, state={resp.state}'
            )
        except Exception as e:
            self.get_logger().warn(f'Explorer stop call failed: {e}')

    def try_approach(self):
        if self.goal_sent or self.target is None or self.map is None:
            return
        if not self.nav.server_is_ready():
            return
        robot = self.robot_xy()
        if robot is None:
            return
        bx, by = self.target.pose.position.x, self.target.pose.position.y
        bearing = math.atan2(by - robot[1], bx - robot[0])
        options = []
        for radius in (self.standoff, self.standoff + .20, self.standoff + .40, self.standoff + .60):
            for offset in (0, .52, -.52, 1.05, -1.05, 1.57, -1.57):
                angle = bearing + offset
                x = bx - radius * math.cos(angle)
                y = by - radius * math.sin(angle)
                if self.free(x, y):
                    options.append((math.hypot(x - robot[0], y - robot[1]), x, y))
        if not options:
            self.say('BALL_FOUND: waiting for SLAM to map a safe stand-off pose')
            return
        _, x, y = min(options)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation = yaw_q(math.atan2(by - y, bx - x))
        self.goal_sent = True
        self.say(f'APPROACHING: Nav2 goal x={x:.2f}, y={y:.2f}')
        # Cancel the polling timer – we've dispatched the Nav2 goal
        if self._approach_timer is not None:
            self._approach_timer.cancel()
            self._approach_timer = None
        future = self.nav.send_goal_async(goal)
        future.add_done_callback(self.accepted)

    def accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.goal_sent = False
            self.say('APPROACH_FAILED: Nav2 rejected target; will retry')
            # Restart polling timer
            if self._approach_timer is None:
                self._approach_timer = self.create_timer(1.0, self.try_approach)
            return
        handle.get_result_async().add_done_callback(self.result)

    def result(self, future):
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.say('SUCCEEDED: reached a safe stand-off point for the red ball')
        else:
            self.goal_sent = False
            self.say('APPROACH_FAILED: Nav2 did not reach target; will retry')
            if self._approach_timer is None:
                self._approach_timer = self.create_timer(1.0, self.try_approach)


def main():
    rclpy.init()
    node = BallApproachManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
